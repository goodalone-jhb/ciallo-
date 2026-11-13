/**
  ******************************************************************************
  * @file    app_ears.c
  * @brief   2 自由度可动猫耳发箍 —— 姿态采集与舵机控制
  *
  *          数据流：
  *             MPU6500 --I2C(100Hz)--> 互补滤波 --> 俯仰角 pitch
  *                                        \---> 角速度 -> 漏积分 -> 转头角 yaw
  *             pitch/yaw --死区+平滑+角速度前馈--> 归一化指令 --混控--> 双路 PWM
  *
  *          总线默认走软件模拟 I2C：不依赖 I2C 外设，出错可恢复，克隆片也认。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_ears.h"
#include "tim.h"
#include "i2c.h"
#include <math.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* MPU6500 / MPU6050 通用寄存器 */
#define MPU_REG_SMPLRT_DIV       0x19U
#define MPU_REG_CONFIG           0x1AU
#define MPU_REG_GYRO_CONFIG      0x1BU
#define MPU_REG_ACCEL_CONFIG     0x1CU
#define MPU_REG_ACCEL_XOUT_H     0x3BU
#define MPU_REG_TEMP_OUT_H       0x41U
#define MPU_REG_GYRO_XOUT_H      0x43U
#define MPU_REG_PWR_MGMT_1       0x6BU
#define MPU_REG_PWR_MGMT_2       0x6CU
#define MPU_REG_WHO_AM_I         0x75U

/* 量程换算：陀螺 ±500dps -> 65.5 LSB/(deg/s)；加速度 ±2g -> 16384 LSB/g */
#define GYRO_LSB_PER_DPS         65.5f
#define ACC_LSB_PER_G            16384.0f

/* 连续读失败多少次判定传感器掉线 */
#define IMU_FAIL_LIMIT           20U
/* 掉线后重连间隔 */
#define IMU_RETRY_MS             1000U
/* 单次 I2C 事务超时（硬件 I2C 模式用） */
#define IMU_TIMEOUT_MS           20U

/* 软件 I2C 引脚 */
#define I2C_SCL_PIN              GPIO_PIN_3
#define I2C_SDA_PIN              GPIO_PIN_4
#define I2C_PORT                 GPIOB

#define SCL_HIGH()   HAL_GPIO_WritePin(I2C_PORT, I2C_SCL_PIN, GPIO_PIN_SET)
#define SCL_LOW()    HAL_GPIO_WritePin(I2C_PORT, I2C_SCL_PIN, GPIO_PIN_RESET)
#define SDA_HIGH()   HAL_GPIO_WritePin(I2C_PORT, I2C_SDA_PIN, GPIO_PIN_SET)
#define SDA_LOW()    HAL_GPIO_WritePin(I2C_PORT, I2C_SDA_PIN, GPIO_PIN_RESET)
#define SDA_LEVEL()  (HAL_GPIO_ReadPin(I2C_PORT, I2C_SDA_PIN) == GPIO_PIN_SET)
#define SCL_LEVEL()  (HAL_GPIO_ReadPin(I2C_PORT, I2C_SCL_PIN) == GPIO_PIN_SET)

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  float    pitch_deg;      /* 俯仰角（互补滤波后） */
  float    pitch_ref;      /* 上电时的俯仰角，作为零点 */
  float    yaw_deg;        /* 转头角（漏积分） */
  float    pitch_rate;     /* deg/s */
  float    yaw_rate;       /* deg/s */
  float    out1;           /* CH1 当前脉宽（浮点便于做斜率限制） */
  float    out2;           /* CH2 当前脉宽 */
  int16_t  gyro_bias[3];   /* 零偏，单位 LSB */
  uint32_t tick;           /* 上次采样时刻 */
  uint32_t retry_tick;     /* 上次重连尝试时刻 */
  uint32_t fault_tick;     /* 上次故障提示时刻 */
  uint8_t  imu_ok;         /* 1 = 传感器在线 */
  uint8_t  ref_valid;      /* 1 = 俯仰零点已锁定（只在第一次上电时取） */
  uint8_t  fail_cnt;       /* 连续失败计数 */
  uint8_t  hold;           /* 1 = 手动打角模式，暂停闭环 */
  uint8_t  imu_addr;       /* 实际使用的 7 位地址 */
  uint8_t  whoami;         /* WHO_AM_I 读回值 */
  uint8_t  bus_err;        /* 总线错误计数（饱和） */
  uint8_t  fault_count;    /* 故障码，同时就是提示动作的抽动次数，见 app_ears.h */
  uint16_t zero_cnt;       /* 连续"六轴数据全为 0"的帧数，用于识别僵尸传感器 */
} EarCtx_t;

/* Private variables ---------------------------------------------------------*/
static EarCtx_t s_ear;

/* 指令延迟线：长度覆盖 300ms，够任何合理的延迟设置；
 * 两个通道共用写指针，各读各的位置，省一半索引管理。
 * 顺带一提：这段是 .bss，上电自动清零，所以第一帧的输出就是中位。
 * 只有 FIXED_DELAY 模式才编译进来，省 248 字节 RAM。 */
#if (TURN_SYNC_MODE == EAR_SYNC_FIXED_DELAY) || (NOD_SYNC_MODE == EAR_SYNC_FIXED_DELAY)
#define EAR_DLY_SAMPLES       ((300U / IMU_PERIOD_MS) + 1U)
static float   s_dly_nod[EAR_DLY_SAMPLES];
static float   s_dly_turn[EAR_DLY_SAMPLES];
static uint8_t s_dly_wr;
#endif

/* "等动作做完再动"的状态机，每根轴一份 */
#if (TURN_SYNC_MODE == EAR_SYNC_AFTER_MOVE) || (NOD_SYNC_MODE == EAR_SYNC_AFTER_MOVE)
typedef struct
{
  float   out;      /* 当前实际输出（归一化量），冻结时就是它不动 */
  uint8_t moving;   /* 1 = 头正在动，舵机站住 */
  uint8_t still;    /* 连续静止帧计数 */
  uint8_t hold;     /* 已冻结帧数，用于超时兜底 */
  uint8_t primed;   /* 0 = 尚未初始化，首帧直接对齐，避免上电猛甩 */
} SyncGate_t;

#if (TURN_SYNC_MODE == EAR_SYNC_AFTER_MOVE)
static SyncGate_t s_sync_turn;
#endif
#if (NOD_SYNC_MODE == EAR_SYNC_AFTER_MOVE)
static SyncGate_t s_sync_nod;
#endif
#endif /* 有轴在用 AFTER_MOVE */

/* 点头的弹簧状态（位置/速度），单位都是归一化量 */
static float   s_spr_pos;
static float   s_spr_vel;

/* 常驻诊断，只记录现场、不额外访问传感器。PC 端按当前 ELF 符号读取，
 * 不使用未预留的固定 RAM 地址。每条总线各留一份，避免软件重试覆盖硬件现场。 */
typedef struct
{
  uint32_t magic;
  uint32_t seq;
  uint32_t tick;
  uint32_t hal_error;
  uint16_t sr1;
  uint16_t sr2;
  uint16_t length;
  uint16_t failures;
  uint16_t calib_ok;
  uint16_t calib_nack;
  uint16_t calib_ff;
  uint8_t phase;       /* 1 扫描 / 2 配置 / 3 选择读法 / 4 标定 / 5 就绪 / 6 运行 */
  uint8_t bus;         /* 0 软件 / 1 硬件 */
  uint8_t fault;
  uint8_t addr;
  uint8_t whoami;
  uint8_t reg;         /* 最后一次失败事务的寄存器 */
  uint8_t op;          /* 1 读 / 2 写 */
  uint8_t nack_stage;  /* 软件 I2C：1 写地址 / 2 寄存器 / 3 读地址 / 4 写数据 */
  uint8_t frame[14];   /* 最后一次事务成功的帧，包括全 FF 等异常内容 */
  uint8_t ack_map[16]; /* 每位对应一个 7 位地址；只探测地址，不写配置 */
  uint8_t id75[128];   /* 对应地址的 0x75 原始读数，必须结合 id75_ok 判断 */
  uint8_t id75_ok[16];
  uint8_t scan_count;
  uint8_t found_count;
  uint8_t scan_state;  /* 1 扫描中 / 2 完成 / 3 总线未释放 */
  uint8_t reserved;
} EarImuDiag_t;

_Static_assert(sizeof(EarImuDiag_t) == 216U, "Update tools/imu_status.py layout");
volatile EarImuDiag_t g_ear_imu_diag[2];

/* Private function prototypes -----------------------------------------------*/
static void     I2C_Delay(void);
static void     Ear_DelayMs(uint32_t ms);
static uint8_t  SW_WaitSCLHigh(void);
static void     SW_BusInit(void);
static void     SW_BusRecover(void);
static void     SW_Start(void);
static void     SW_Stop(void);
static uint8_t  SW_WriteBit(uint8_t bit);
static uint8_t  SW_ReadBit(void);
static uint8_t  SW_WriteByte(uint8_t byte);
static uint8_t  SW_ReadByte(uint8_t ack);
static uint8_t  SW_Write(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len);
static uint8_t  SW_Read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
static uint8_t  Bus_Write(uint8_t addr, uint8_t reg, uint8_t value);
static uint8_t  Bus_Read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
static uint8_t  Bus_Probe(uint8_t addr);
static void     Bus_Recover(void);
static uint8_t  IMU_WhoamiValid(uint8_t who);
static uint8_t  IMU_Scan(void);
static uint8_t  IMU_ConfigOnce(uint8_t variant);
static uint8_t  IMU_Config(void);
static uint8_t  IMU_DataProbe(void);
static uint8_t  IMU_Calibrate(void);
static uint8_t  IMU_BringUp(void);
static uint8_t  IMU_FrameUsable(const uint8_t *buf);
static uint8_t  IMU_ReadSample(float *ax, float *ay, float *az,
                               float *gx, float *gy, float *gz);
static float    ClampF(float v, float lo, float hi);
static uint16_t ClampUs(float us);
static float    ShapeNorm(float angle, float deadband, float full);
static void     Servo_Apply(float us1, float us2);
static void     Servo_RampTo(uint16_t us1, uint16_t us2, uint8_t steps);
static void     Ear_Update(float dt);
static void     Ear_Output(void);
static void     Ear_BeaconSwing(uint16_t half_swings);
static void     Ear_DelayUs(uint16_t us);

/*==============================================================================
 *  软件模拟 I2C（开漏输出 + 内部上拉，可从 IDR 回读电平）
 *  外部仍需 4.7k 上拉，常见的 GY 模块板子自带
 *============================================================================*/
static void I2C_Delay(void)
{
  volatile uint8_t i;
  for (i = 0U; i < EAR_I2C_DELAY_LOOPS; i++)
  {
    __NOP();
  }
}

/* 忙等延时：**不依赖 SysTick**。
 * 上电路径（自检、故障提示）全部用它，这样即使 HAL 的 tick 没起来、
 * HAL_Delay 永久卡死，上电动作也一定能走完 —— 否则"舵机不动"就无法
 * 区分是 PWM 坏了还是 CPU 卡在 HAL_Delay 里。精度不重要，只用来看动作。 */
/* 独立看门狗"喂狗"。
 * 有的 PY32 片子/选项字节会把 IWDG 设成硬件模式，上电就跑，
 * 固件不喂就每隔几十毫秒复位一次 —— 现象正好是"什么都不动、引脚像悬空"。
 * IWDG 没开时写这个键是空操作，开了就能把芯片从复位循环里救回来。 */
static void Wdg_Kick(void)
{
  IWDG->KR = 0xAAAAU;
}

static void Ear_DelayMs(uint32_t ms)
{
  volatile uint32_t i;
  /* 内层循环实测约 12 个周期/次（-Og 下 ldr/add/str/ldr/cmp/bcc 各占几条），
   * 因此每毫秒的循环次数 = 主频 / 12000。原来的 6000 会让所有延时偏长一倍，
   * 自检看起来"卡住"就与此有关。 */
  uint32_t loops = SystemCoreClock / 12000U;

  if (loops == 0U) { loops = 1U; }

  while (ms-- != 0U)
  {
    Wdg_Kick();
    for (i = 0U; i < loops; i++)
    {
      __NOP();
    }
  }
}

static void SW_BusInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin   = I2C_SCL_PIN | I2C_SDA_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct);

  SCL_HIGH();
  SDA_HIGH();
  I2C_Delay();
}

/* 总线恢复：从机若把 SDA 拉死在低电平，用 9 个时钟把它顶出来 */
static void SW_BusRecover(void)
{
  uint8_t i;

  SDA_HIGH();
  for (i = 0U; i < 9U; i++)
  {
    SCL_LOW();
    I2C_Delay();
    I2C_Delay();
    SCL_HIGH();
    I2C_Delay();
    I2C_Delay();
  }
  /* 收尾前先把 SCL 拉低，否则 SW_Stop 会先产生一个重复起始 */
  SCL_LOW();
  I2C_Delay();
  SW_Stop();
}

/* 等 SCL 真正变高（兼容从机时钟拉伸），超时返回 0 */
static uint8_t SW_WaitSCLHigh(void)
{
  uint16_t timeout = 400U;

  while (SCL_LEVEL() == 0U)
  {
    if (--timeout == 0U)
    {
      return 0U;
    }
  }
  return 1U;
}

static void SW_Start(void)
{
  SDA_HIGH();
  I2C_Delay();
  SCL_HIGH();
  (void)SW_WaitSCLHigh();
  I2C_Delay();
  SDA_LOW();
  I2C_Delay();
  SCL_LOW();
  I2C_Delay();
}

static void SW_Stop(void)
{
  SDA_LOW();
  I2C_Delay();
  SCL_HIGH();
  (void)SW_WaitSCLHigh();
  I2C_Delay();
  SDA_HIGH();
  I2C_Delay();
}

static uint8_t SW_WriteBit(uint8_t bit)
{
  if (bit != 0U) { SDA_HIGH(); } else { SDA_LOW(); }
  I2C_Delay();
  SCL_HIGH();
  (void)SW_WaitSCLHigh();
  I2C_Delay();
  SCL_LOW();
  I2C_Delay();

  return 0U;
}

static uint8_t SW_ReadBit(void)
{
  uint8_t bit;

  SDA_HIGH();
  I2C_Delay();
  SCL_HIGH();
  (void)SW_WaitSCLHigh();
  I2C_Delay();
  bit = SDA_LEVEL() ? 1U : 0U;
  SCL_LOW();
  I2C_Delay();

  return bit;
}

/* 返回 0 = 从机应答(ACK)，1 = 无应答(NACK) */
static uint8_t SW_WriteByte(uint8_t byte)
{
  uint8_t i;
  uint8_t ack;

  for (i = 0U; i < 8U; i++)
  {
    (void)SW_WriteBit((uint8_t)((byte & 0x80U) ? 1U : 0U));
    byte = (uint8_t)(byte << 1);
  }

  /* 第 9 个时钟读 ACK */
  SDA_HIGH();
  I2C_Delay();
  SCL_HIGH();
  (void)SW_WaitSCLHigh();
  I2C_Delay();
  ack = SDA_LEVEL() ? 1U : 0U;
  SCL_LOW();
  I2C_Delay();

  return ack;
}

static uint8_t SW_ReadByte(uint8_t ack)
{
  uint8_t i;
  uint8_t value = 0U;

  SDA_HIGH();
  for (i = 0U; i < 8U; i++)
  {
    value = (uint8_t)(value << 1);
    value |= SW_ReadBit();
  }

  /* 主机应答：ACK 拉低，NACK 释放 */
  if (ack != 0U) { SDA_LOW(); } else { SDA_HIGH(); }
  I2C_Delay();
  SCL_HIGH();
  (void)SW_WaitSCLHigh();
  I2C_Delay();
  SCL_LOW();
  I2C_Delay();
  SDA_HIGH();

  return value;
}

/* 上一次事务失败卡在哪一步：
 *   0 = 没失败   1 = 地址字节无应答   2 = 寄存器字节无应答   3 = 读地址无应答
 * 这个信息是诊断的关键：同样是"读不回来"，
 *   卡在 1 是设备根本没选中，卡在 2 是设备认了地址但不认这个寄存器。 */
static uint8_t s_nack_stage = 0U;

static uint8_t SW_Write(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len)
{
  uint16_t i;

  s_nack_stage = 0U;

  SW_Start();
  if (SW_WriteByte((uint8_t)(addr << 1)) != 0U) { s_nack_stage = 1U; SW_Stop(); return 0U; }
  if (SW_WriteByte(reg) != 0U)                  { s_nack_stage = 2U; SW_Stop(); return 0U; }

  for (i = 0U; i < len; i++)
  {
    if (SW_WriteByte(data[i]) != 0U) { s_nack_stage = 4U; SW_Stop(); return 0U; }
  }
  SW_Stop();

  return 1U;
}

static uint8_t SW_Read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  uint16_t i;

  s_nack_stage = 0U;

  SW_Start();
  if (SW_WriteByte((uint8_t)(addr << 1)) != 0U)            { s_nack_stage = 1U; SW_Stop(); return 0U; }
  if (SW_WriteByte(reg) != 0U)                             { s_nack_stage = 2U; SW_Stop(); return 0U; }
  SW_Start();                                              /* 重复起始 */
  if (SW_WriteByte((uint8_t)((addr << 1) | 0x01U)) != 0U)  { s_nack_stage = 3U; SW_Stop(); return 0U; }

  for (i = 0U; i < len; i++)
  {
    buf[i] = SW_ReadByte((uint8_t)(((i + 1U) < len) ? 1U : 0U));
  }
  SW_Stop();

  return 1U;
}

/*==============================================================================
 *  总线分发：硬件 I2C1 / 软件模拟 I2C，**运行时**选择
 *
 *  为什么改成运行时而不是编译期开关：
 *  这颗片子的表现是"配置寄存器（0x19/0x75）能读到正确值，数据寄存器
 *  （0x3B 起）却读不回来"，从电路上说不通，只能靠实测分辨到底卡在哪一层。
 *  于是一次烧录里让固件先试硬件 I2C（时序由外设硬件产生，不受延时循环
 *  精度、中断抖动影响），不通再整体退回软件 I2C，两条都失败才报故障。
 *  这样一次烧录就能把"软 I2C 实现有问题"和"芯片/板子有问题"分开。
 *============================================================================*/
#define BUS_SOFT   0U
#define BUS_HW     1U

static uint8_t s_bus = BUS_SOFT;   /* 当前在用的总线 */

static void Diag_Failure(uint8_t addr, uint8_t reg, uint16_t len, uint8_t op)
{
  volatile EarImuDiag_t *d = &g_ear_imu_diag[s_bus];

  d->tick = HAL_GetTick();
  d->addr = addr;
  d->reg = reg;
  d->length = len;
  d->op = op;
  d->nack_stage = s_nack_stage;
  if (d->failures < 0xFFFFU) { d->failures++; }
  if (s_bus == BUS_HW)
  {
    /* 必须在 Bus_HwReset 清理句柄和外设之前保存。 */
    d->hal_error = hi2c1.ErrorCode;
    d->sr1 = (uint16_t)I2C1->SR1;
    d->sr2 = (uint16_t)I2C1->SR2;
  }
  d->seq++;
}

/* PY32 这版 HAL 读失败后不恢复 State，必须自己重装外设，否则句柄永久 BUSY */
static void Bus_HwReset(void)
{
  (void)HAL_I2C_DeInit(&hi2c1);
  (void)HAL_I2C_Init(&hi2c1);
}

/* 切换总线：硬件模式走 HAL 外设，软件模式把 PB3/PB4 抢回普通开漏 GPIO。
 * MspDeInit 会关掉 I2C1 时钟并释放引脚，所以切过去之后外设不会再抢总线。 */
static void Bus_SetMode(uint8_t bus)
{
  if (bus == BUS_HW)
  {
    s_bus = BUS_HW;
    (void)HAL_I2C_Init(&hi2c1);      /* 重新装外设，MspInit 会把 PB3/PB4 配成 AF 开漏 */
    return;
  }

  s_bus = BUS_SOFT;
  (void)HAL_I2C_DeInit(&hi2c1);
  SW_BusInit();
  SW_BusRecover();
}

/* 硬件模式下 PB3/PB4 属于 AF，直接写 GPIO 并不能产生恢复时钟。
 * 临时切到开漏 GPIO，发完恢复时钟后恢复原来的总线模式。 */
static void Bus_Recover(void)
{
  uint8_t bus = s_bus;

  if (bus == BUS_HW)
  {
    Bus_SetMode(BUS_SOFT);
    Bus_SetMode(BUS_HW);
  }
  else
  {
    SW_BusRecover();
  }
}

/* 地址级探测与身份读取分开：非 MPU 器件即使不支持 0x75，也要记录 ACK。 */
static uint8_t Bus_Probe(uint8_t addr)
{
  uint8_t ack;

  if (s_bus == BUS_HW)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1U, IMU_TIMEOUT_MS) != HAL_OK)
    {
      Bus_HwReset();
      return 0U;
    }
    return 1U;
  }

  SW_Start();
  ack = (SW_WriteByte((uint8_t)(addr << 1)) == 0U) ? 1U : 0U;
  SW_Stop();
  return (SCL_LEVEL() && SDA_LEVEL()) ? ack : 0U;
}

static uint8_t Bus_Write(uint8_t addr, uint8_t reg, uint8_t value)
{
  s_nack_stage = 0U;
  if (s_bus == BUS_HW)
  {
    if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1), (uint16_t)reg,
                          I2C_MEMADD_SIZE_8BIT, &value, 1U, IMU_TIMEOUT_MS) != HAL_OK)
    {
      Diag_Failure(addr, reg, 1U, 2U);
      Bus_HwReset();
      return 0U;
    }
    return 1U;
  }
  if (SW_Write(addr, reg, &value, 1U) == 0U)
  {
    Diag_Failure(addr, reg, 1U, 2U);
    return 0U;
  }
  return 1U;
}

static uint8_t Bus_Read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (s_bus == BUS_HW)
  {
    s_nack_stage = 0U;   /* 硬件外设给不出 ACK 位置，只能报"事务失败" */
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), (uint16_t)reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len, IMU_TIMEOUT_MS) != HAL_OK)
    {
      Diag_Failure(addr, reg, len, 1U);
      Bus_HwReset();
      return 0U;
    }
    return 1U;
  }
  if (SW_Read(addr, reg, buf, len) == 0U)
  {
    Diag_Failure(addr, reg, len, 1U);
    return 0U;
  }
  return 1U;
}

/*==============================================================================
 *  整帧读取（14 字节布局：acc 6 + temp 2 + gyro 6）
 *
 *  现场实测：单字节事务 100% 可靠（WHO_AM_I、配置回读的值都对），
 *  但"一次事务连续读多个字节"在标定阶段 150 次全部失败。
 *
 *  结论：卡在"多字节事务"这一层，不是地址、供电、芯片本体的问题。
 *  与其继续猜这颗片子支持什么，不如让固件自己试：三级读法，失败就降级，
 *  降级成功后锁定，之后一直用最合适的那一级。
 *
 *    IMU_RD_BURST14 : 一次事务读 14 字节，标准做法，最快（每帧 1 次事务）
 *    IMU_RD_PAIRED  : 4 次 2 字节事务，只读 accX/accZ/gyroY/gyroZ（每帧 4 次）
 *    IMU_RD_PERBYTE : 8 次 1 字节事务，与已验证可用的单字节路径完全一致（每帧 8 次）
 *============================================================================*/
#define IMU_RD_BURST14    0U
#define IMU_RD_PAIRED     1U
#define IMU_RD_PERBYTE    2U

static uint8_t s_rd_mode = IMU_RD_BURST14;

/* 单字节事务读一个寄存器 —— 和读 WHO_AM_I、回读配置走的是同一条路径 */
static uint8_t IMU_RdReg(uint8_t reg, uint8_t *val)
{
  return Bus_Read(s_ear.imu_addr, reg, val, 1U);
}

/* 逐字节读：只取算法真正用到的 accX / accZ / gyroY / gyroZ，
 * 其余位置清零（accY、gyroX、温度本来就没参与解算）。 */
static uint8_t IMU_ReadFramePerByte(uint8_t *buf)
{
  uint8_t i;

  for (i = 0U; i < 14U; i++) { buf[i] = 0U; }

  if ((IMU_RdReg((uint8_t)(MPU_REG_ACCEL_XOUT_H + 0U), &buf[0])  == 0U) ||
      (IMU_RdReg((uint8_t)(MPU_REG_ACCEL_XOUT_H + 1U), &buf[1])  == 0U)) { return 0U; }

  if ((IMU_RdReg((uint8_t)(MPU_REG_ACCEL_XOUT_H + 4U), &buf[4])  == 0U) ||
      (IMU_RdReg((uint8_t)(MPU_REG_ACCEL_XOUT_H + 5U), &buf[5])  == 0U)) { return 0U; }

  if ((IMU_RdReg((uint8_t)(MPU_REG_GYRO_XOUT_H  + 2U), &buf[10]) == 0U) ||
      (IMU_RdReg((uint8_t)(MPU_REG_GYRO_XOUT_H  + 3U), &buf[11]) == 0U)) { return 0U; }

  if ((IMU_RdReg((uint8_t)(MPU_REG_GYRO_XOUT_H  + 4U), &buf[12]) == 0U) ||
      (IMU_RdReg((uint8_t)(MPU_REG_GYRO_XOUT_H  + 5U), &buf[13]) == 0U)) { return 0U; }

  return 1U;
}

/* 2 字节配对读：一个轴的 H/L 在一次事务里取回，
 * 不会出现"高字节来自这一帧、低字节来自下一帧"的撕裂。 */
static uint8_t IMU_ReadFramePaired(uint8_t *buf)
{
  uint8_t i;

  for (i = 0U; i < 14U; i++) { buf[i] = 0U; }

  if (Bus_Read(s_ear.imu_addr, (uint8_t)(MPU_REG_ACCEL_XOUT_H + 0U), &buf[0],  2U) == 0U) { return 0U; }
  if (Bus_Read(s_ear.imu_addr, (uint8_t)(MPU_REG_ACCEL_XOUT_H + 4U), &buf[4],  2U) == 0U) { return 0U; }
  if (Bus_Read(s_ear.imu_addr, (uint8_t)(MPU_REG_GYRO_XOUT_H  + 2U), &buf[10], 2U) == 0U) { return 0U; }
  if (Bus_Read(s_ear.imu_addr, (uint8_t)(MPU_REG_GYRO_XOUT_H  + 4U), &buf[12], 2U) == 0U) { return 0U; }

  return 1U;
}

static uint8_t IMU_ReadFrame(uint8_t *buf)
{
  uint8_t i;
  if (s_rd_mode == IMU_RD_BURST14)
  {
    /* 注意：不光要"事务成功"，还要"数据不是全 0xFF"。
     * 有些片子会乖乖回 ACK 但 SDA 全程不拉动，读出来是一整帧 0xFF。 */
    if ((Bus_Read(s_ear.imu_addr, MPU_REG_ACCEL_XOUT_H, buf, 14U) != 0U) &&
        (IMU_FrameUsable(buf) != 0U))
    {
      for (i = 0U; i < 14U; i++) { g_ear_imu_diag[s_bus].frame[i] = buf[i]; }
      return 1U;
    }
    Bus_Recover();               /* 恢复时必须匹配当前硬件/软件总线模式 */
    s_rd_mode = IMU_RD_PAIRED;   /* 永久降级，之后不再白试长事务 */
  }

  if (s_rd_mode == IMU_RD_PAIRED)
  {
    if ((IMU_ReadFramePaired(buf) != 0U) && (IMU_FrameUsable(buf) != 0U))
    {
      for (i = 0U; i < 14U; i++) { g_ear_imu_diag[s_bus].frame[i] = buf[i]; }
      return 1U;
    }
    Bus_Recover();
    s_rd_mode = IMU_RD_PERBYTE;
  }

  if (IMU_ReadFramePerByte(buf) != 0U)
  {
    for (i = 0U; i < 14U; i++) { g_ear_imu_diag[s_bus].frame[i] = buf[i]; }
    return 1U;
  }

  Bus_Recover();
  return 0U;
}

/* 用一对"已知值"寄存器探测多字节事务到底可不可靠：
 * 配置阶段刚写过 SMPLRT_DIV=0x09、CONFIG=0x03，这两个都是可以原样读回的纯数据寄存器。
 *   读不回来（NACK） -> 两字节事务直接不成立，降到逐字节
 *   读回来但值不对   -> 事务"成功"但数据不可信，同样降到逐字节
 *   读回来且值正确   -> 保持突发读，万一数据区不适合再自动降级 */
static void IMU_SelectReadMode(void)
{
  uint8_t b[2] = { 0U, 0U };

  s_rd_mode = IMU_RD_BURST14;

  if (Bus_Read(s_ear.imu_addr, MPU_REG_SMPLRT_DIV, b, 2U) == 0U)
  {
    s_rd_mode = IMU_RD_PERBYTE;
    return;
  }

  if (b[0] != 0x09U)
  {
    s_rd_mode = IMU_RD_PERBYTE;
  }
}

/* 整帧全 0xFF 说明 SDA 一直浮在高电平 —— 主机没释放总线或者从机根本没驱动。
 * 这种"假成功"（没有 NACK 但数据是垃圾）必须当失败处理，
 * 否则会被当成有效零偏标定进去，之后姿态就完全错了。
 *
 * 只检查算法真正用到的 8 个字节（accX / accZ / gyroY / gyroZ）：
 * 降级到逐字节读时，其余位置是主动清零的，不应该参与判定。 */
static uint8_t IMU_FrameUsable(const uint8_t *buf)
{
  static const uint8_t idx[8] = { 0U, 1U, 4U, 5U, 10U, 11U, 12U, 13U };
  uint8_t i;

  for (i = 0U; i < 8U; i++)
  {
    if (buf[idx[i]] != 0xFFU) { return 1U; }
  }
  return 0U;
}

/*==============================================================================
 *  传感器初始化
 *============================================================================*/
static uint8_t IMU_WhoamiValid(uint8_t who)
{
  /* MPU6050=0x68 MPU6500=0x70 MPU9250=0x71 MPU9255=0x73 克隆片常见 0x98 */
  return (uint8_t)((who == 0x68U) || (who == 0x70U) || (who == 0x71U) ||
                   (who == 0x73U) || (who == 0x98U));
}

/* 扫描 0x08..0x77 的所有普通 7 位地址；完整保存 ACK 与身份读取结果。
 * 地址不限于 0x68/0x69。认到 MPU 兼容 ID 才配置，未知器件保留故障 5
 * 等待确认寄存器协议，避免把其它芯片直接按 MPU 寄存器写入。 */
static uint8_t IMU_Scan(void)
{
  volatile EarImuDiag_t *d = &g_ear_imu_diag[s_bus];
  uint8_t addr;
  uint8_t who;
  uint8_t known = 0U;
  uint8_t first = 0U;
  uint8_t first_who = 0U;

  s_ear.fault_count = 3U;   /* 默认：地址无人应答 */
  s_ear.whoami = 0U;
  s_ear.imu_addr = 0U;
  d->scan_state = 1U;

  for (addr = IMU_SCAN_ADDR_MIN; addr <= IMU_SCAN_ADDR_MAX; addr++)
  {
    Wdg_Kick();
    if (!SCL_LEVEL() || !SDA_LEVEL())
    {
      Bus_Recover();
      if (!SCL_LEVEL() || !SDA_LEVEL())
      {
        d->scan_state = 3U;
        return 0U;
      }
    }
    d->scan_count++;
    if (Bus_Probe(addr) == 0U) { continue; }
    d->ack_map[addr >> 3] |= (uint8_t)(1U << (addr & 7U));
    d->found_count++;
    who = 0U;
    if (Bus_Read(addr, MPU_REG_WHO_AM_I, &who, 1U) != 0U)
    {
      d->id75[addr] = who;
      d->id75_ok[addr >> 3] |= (uint8_t)(1U << (addr & 7U));
      if (IMU_WhoamiValid(who) != 0U)
      {
        if ((known == 0U) || (addr == IMU_ADDR_PRIMARY) ||
            ((addr == IMU_ADDR_SECONDARY) && (s_ear.imu_addr != IMU_ADDR_PRIMARY)))
        {
          known = 1U;
          s_ear.imu_addr = addr;
          s_ear.whoami = who;
        }
      }
    }
    if (first == 0U) { first = addr; first_who = who; }
  }
  if (!SCL_LEVEL() || !SDA_LEVEL())
  {
    d->scan_state = 3U;
    return 0U;
  }
  d->scan_state = 2U;
  if (known != 0U) { return 1U; }
  if (d->found_count != 0U)
  {
    s_ear.imu_addr = first;
    s_ear.whoami = first_who;
    s_ear.fault_count = 5U;
  }
  return 0U;
}

/*==============================================================================
 *  配置：一次尝试用哪个"变体"
 *
 *  变体之间的唯一差别是时钟源和要不要软复位 —— 这两件事恰恰是克隆片
 *  最容易出问题的地方：
 *    0：软复位 + 陀螺 X 轴 PLL 时钟（官方推荐，精度最好）
 *    1：软复位 + 内部 8MHz RC 时钟（PLL 起不来时用）
 *    2：不复位 + 内部 RC + DLPF 直通（被软复位搞坏的片子用）
 *
 *  PLL 起不来时，芯片表面上照样应答 I2C、配置寄存器也写得进读得出，
 *  但数据区（0x3B 起）可能整片读不回来 —— 症状和我们遇到的一模一样，
 *  所以必须真的把三种都试一遍再下结论。
 *============================================================================*/
static uint8_t IMU_ConfigOnce(uint8_t variant)
{
  uint8_t check = 0U;
  uint8_t pwr   = (variant == 0U) ? 0x01U : 0x00U;
  uint8_t dlpf  = (variant == 2U) ? 0x00U : 0x03U;

  if (variant != 2U)
  {
    /* 软复位，等待内部寄存器恢复（MPU6500 复位后默认进 sleep） */
    if (Bus_Write(s_ear.imu_addr, MPU_REG_PWR_MGMT_1, 0x80U) == 0U) { return 0U; }
    Ear_DelayMs(100U);
  }

  /* 唤醒 + 选时钟源 */
  if (Bus_Write(s_ear.imu_addr, MPU_REG_PWR_MGMT_1, pwr) == 0U) { return 0U; }
  Ear_DelayMs(20U);

  /* 六轴全开 */
  if (Bus_Write(s_ear.imu_addr, MPU_REG_PWR_MGMT_2, 0x00U) == 0U) { return 0U; }

  /* 采样率 = 1kHz / (1+9) = 100Hz */
  if (Bus_Write(s_ear.imu_addr, MPU_REG_SMPLRT_DIV, 0x09U) == 0U) { return 0U; }

  /* 内部低通 DLPF=3：加速度 44Hz / 陀螺 42Hz，滤掉舵机与结构的抖动 */
  if (Bus_Write(s_ear.imu_addr, MPU_REG_CONFIG, dlpf) == 0U) { return 0U; }

  /* 陀螺 ±500dps */
  if (Bus_Write(s_ear.imu_addr, MPU_REG_GYRO_CONFIG, 0x08U) == 0U) { return 0U; }

  /* 加速度 ±2g */
  if (Bus_Write(s_ear.imu_addr, MPU_REG_ACCEL_CONFIG, 0x00U) == 0U) { return 0U; }

  /* 回读一个"纯读写寄存器"确认配置真的进去了。
   * 不用 GYRO_CONFIG 做校验：MPU6500 的 FCHOICE_B 位是反相的，回读值不等于写入值。 */
  if (Bus_Read(s_ear.imu_addr, MPU_REG_SMPLRT_DIV, &check, 1U) == 0U) { return 0U; }
  if (check != 0x09U) { return 0U; }

  /* 确认芯片真的醒着：PWR_MGMT_1 的 bit6 就是 SLEEP。
   * 一批克隆片在上电/软复位后会自己回到 sleep —— 表现正是"配置写得进去、
   * 回读也对，但六轴数据恒为 0 或整帧 0xFF"，用常规流程查不出来。 */
  if (Bus_Read(s_ear.imu_addr, MPU_REG_PWR_MGMT_1, &check, 1U) == 0U) { return 0U; }
  if ((check & 0x40U) != 0U)
  {
    if (Bus_Write(s_ear.imu_addr, MPU_REG_PWR_MGMT_1, pwr) == 0U) { return 0U; }
    Ear_DelayMs(10U);
    if (Bus_Read(s_ear.imu_addr, MPU_REG_PWR_MGMT_1, &check, 1U) == 0U) { return 0U; }
    if ((check & 0x40U) != 0U) { return 0U; }   /* 叫不醒，按配置失败处理 */
  }

  return 1U;
}

/* 数据区探测：配置写得进去 ≠ 数据读得出来。
 * 读 0x3F(ACCEL_ZOUT_H) 和 0x41(TEMP_OUT_H) 各一次，只要事务拿到 ACK 就算通。
 * 这两个是"数据区"的寄存器，和 0x19/0x75 这些配置寄存器走的是不同路径，
 * 正是这一次探测把"配置能读、数据读不了"这种片子当场识别出来。 */
static uint8_t IMU_DataProbe(void)
{
  uint8_t v = 0U;

  if (Bus_Read(s_ear.imu_addr, (uint8_t)(MPU_REG_ACCEL_XOUT_H + 4U), &v, 1U) == 0U) { return 0U; }
  if (Bus_Read(s_ear.imu_addr, MPU_REG_TEMP_OUT_H, &v, 1U) == 0U) { return 0U; }

  return 1U;
}

/* 逐个变体试配置，每个变体都必须通过"数据区能读"才算成功。
 * 三个都失败时，按事务卡在哪一步给出更精确的故障码。 */
static uint8_t IMU_Config(void)
{
  uint8_t v;
  uint8_t configured = 0U;

  for (v = 0U; v < 3U; v++)
  {
    if (IMU_ConfigOnce(v) == 0U)
    {
      Ear_DelayMs(20U);
      continue;
    }

    configured = 1U;                 /* 至少配置写进去了 */

    if (IMU_DataProbe() != 0U)
    {
      return 1U;
    }
  }

  if (configured != 0U)
  {
    /* 配置写得进、读得回，但数据区一律读不出来。
     * 按 ACK 缺失的位置区分性质：
     *   9  = 寄存器字节无应答 -> 这颗片子没有 0x3B 这个寄存器（型号不对）
     *   10 = 读地址无应答     -> 切换读写方向时从机掉线
     *   4  = 地址字节无应答 / 硬件总线给不出细节 */
    if (s_nack_stage == 2U)      { s_ear.fault_count = 9U; }
    else if (s_nack_stage == 3U) { s_ear.fault_count = 10U; }
    else                         { s_ear.fault_count = 4U; }
  }
  else
  {
    s_ear.fault_count = (IMU_WhoamiValid(s_ear.whoami) != 0U) ? 2U : 5U;   /* 型号未知时保留 5 */
  }

  return 0U;
}

/* 零偏标定：上电时保持静止，统计陀螺静态输出与初始俯仰角
 * 返回 1 = 拿到了可用数据；0 = 一帧都读不出来 */
static uint8_t IMU_Calibrate(void)
{
  uint8_t buf[14];
  int32_t sum[3] = {0, 0, 0};
  float   ax_sum = 0.0f;
  float   az_sum = 0.0f;
  uint16_t i;
  uint16_t ok = 0U;
  uint16_t nonzero = 0U;
  uint16_t nack = 0U;       /* 事务失败：地址/寄存器字节没拿到 ACK */
  uint16_t ff   = 0U;       /* 事务成功但整帧 0xFF：应答了却不驱动数据线 */

  for (i = 0U; i < IMU_CALIB_SAMPLES; i++)
  {
    if (IMU_ReadFrame(buf) == 0U)
    {
      nack++;
    }
    else if (IMU_FrameUsable(buf) == 0U)
    {
      ff++;
    }
    else
    {
      sum[0] += (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
      sum[1] += (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
      sum[2] += (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

      ax_sum += (float)(int16_t)(((uint16_t)buf[0] << 8) | buf[1]) / ACC_LSB_PER_G;
      az_sum += (float)(int16_t)(((uint16_t)buf[4] << 8) | buf[5]) / ACC_LSB_PER_G;

      /* 检查所有已读取的运动轴，排除温度字节。水平放置时 accX
       * 可以恰好为 0，而 accZ 约 1g，不能仅凭 accX 判定僵尸传感器。
       * 配对/逐字节模式中未读取的轴已清零，同样适用。 */
      if ((buf[0] | buf[1] | buf[2] | buf[3] | buf[4] | buf[5] |
           buf[8] | buf[9] | buf[10] | buf[11] | buf[12] | buf[13]) != 0U)
      {
        nonzero++;
      }

      ok++;
    }
    Ear_DelayMs(2U);
  }
  g_ear_imu_diag[s_bus].calib_ok = ok;
  g_ear_imu_diag[s_bus].calib_nack = nack;
  g_ear_imu_diag[s_bus].calib_ff = ff;

  if (ok == 0U)
  {
    /* 两种"读不回来"性质完全不同，必须分开报：
     *   8 = 事务拿到 ACK，但数据字节全是 0xFF —— 芯片在应答却不肯驱动数据线，
     *       典型是没真正唤醒、供电处在边界（比如 3.3V 灌进模块 VCC 被 LDO 压到 2.4V）
     *   4 = 事务本身失败（ACK 都没拿到），或两种各占一半 */
    if (ff > nack)
    {
      s_ear.fault_count = 8U;
    }
    else if (s_nack_stage == 2U)
    {
      s_ear.fault_count = 9U;
    }
    else if (s_nack_stage == 3U)
    {
      s_ear.fault_count = 10U;
    }
    else
    {
      s_ear.fault_count = 4U;
    }
    return 0U;
  }

  if (nonzero == 0U)
  {
    /* 能读能写，但六轴数据恒为 0：芯片没真正唤醒，或寄存器映射不是这一族
     * （典型是"标着 MPU6500 实际是别的片子"） */
    s_ear.fault_count = (IMU_WhoamiValid(s_ear.whoami) != 0U) ? 6U : 5U;
    return 0U;
  }

  /* 样本偏少也照常使用：总线既然通了，零偏用已有样本平均就够，
   * 没必要因为标定不完美就整机罢工 */
  s_ear.gyro_bias[0] = (int16_t)(sum[0] / (int32_t)ok);
  s_ear.gyro_bias[1] = (int16_t)(sum[1] / (int32_t)ok);
  s_ear.gyro_bias[2] = (int16_t)(sum[2] / (int32_t)ok);

  /* 俯仰零点与初始姿态。
   * 先无条件把 pitch_deg 设成"此刻由重力算出的真实角度"，
   * 再决定零点取哪里 —— 两件事分开，避免滤波从错误的值开始收敛。 */
  {
    float a0 = atan2f(IMU_PITCH_ACC_SIGN * (ax_sum / (float)ok),
                      az_sum / (float)ok) * RAD_TO_DEG;

    s_ear.pitch_deg = a0;

    if (s_ear.ref_valid == 0U)
    {
#if (PITCH_ZERO_MODE == 0)
      s_ear.pitch_ref = (float)PITCH_LEVEL_TRIM_DEG;   /* 绝对水平 */
#else
      s_ear.pitch_ref = a0;                            /* 上电姿态 */
#endif
      s_ear.ref_valid = 1U;
    }
    /* 已经锁定过零点就不再动：运行中掉线重连会再走一遍标定，
     * 如果那时正好歪着头，零点会被悄悄改掉，动作整个偏一半。 */
  }
  s_ear.yaw_deg = 0.0f;

  return 1U;
}

/* 故障码的"深度"：越靠后说明流程走得越远，报错时优先报走得远的那一条，
 * 它离根因最近。 */
static uint8_t FaultDepth(uint8_t code)
{
  switch (code)
  {
    case 3U:  return 0U;   /* 地址无人应答：连第一步都没成 */
    case 5U:  return 1U;   /* 有人应答但型号不认识 */
    case 2U:  return 2U;   /* 配置写不进去 */
    case 8U:  return 3U;   /* 事务成功但数据整帧 0xFF */
    case 4U:  return 4U;   /* 数据读事务失败 */
    case 10U: return 5U;   /* 切换读写方向时从机掉线 */
    case 9U:  return 6U;   /* 寄存器字节无应答：这颗片子没有该寄存器 */
    case 6U:  return 7U;   /* 能读写但六轴恒为 0 */
    default:  return 0U;
  }
}

/* 在某一条总线上跑一遍完整启动：扫描 -> 配置 -> 探测读法 -> 标定 */
static uint8_t IMU_BringUpOnBus(uint8_t bus)
{
  volatile EarImuDiag_t *d = &g_ear_imu_diag[bus];
  memset((void *)d, 0, sizeof(*d));
  d->magic = 0x554D4945UL;   /* 小端 ASCII: EIMU */
  d->bus = bus;
  d->phase = 1U;
  Bus_SetMode(bus);

  if (IMU_Scan() == 0U)
  {
    d->addr = s_ear.imu_addr;
    d->whoami = s_ear.whoami;
    d->fault = s_ear.fault_count;
    return 0U;  /* 无应答，或身份未知：记录现场，不盲写未知器件配置 */
  }

  d->addr = s_ear.imu_addr;
  d->whoami = s_ear.whoami;
  d->phase = 2U;
  if (IMU_Config() == 0U)
  {
    d->fault = s_ear.fault_count;
    return 0U;
  }

  /* 配置写进去了，顺手用这对已知值寄存器判定"多字节事务"是否可靠 */
  d->phase = 3U;
  IMU_SelectReadMode();

  d->phase = 4U;
  if (IMU_Calibrate() == 0U)
  {
    d->fault = s_ear.fault_count;
    return 0U;
  }
  d->phase = 5U;

  s_ear.fail_cnt = 0U;
  s_ear.zero_cnt = 0U;
  s_ear.bus_err  = 0U;
  return 1U;
}

/* 一次完整的启动流程：先硬件 I2C，不行再软件 I2C。
 * 两条总线都失败时，报"走得最远"的那个故障码。 */
static uint8_t IMU_BringUp(void)
{
  uint8_t code_hw = 0U;
  uint8_t code_sw = 0U;

  if (IMU_BringUpOnBus(BUS_HW) != 0U)
  {
    return 1U;
  }
  code_hw = s_ear.fault_count;

  if (IMU_BringUpOnBus(BUS_SOFT) != 0U)
  {
    return 1U;
  }
  code_sw = s_ear.fault_count;

  s_ear.fault_count = (FaultDepth(code_sw) >= FaultDepth(code_hw)) ? code_sw : code_hw;
  return 0U;
}

/*==============================================================================
 *  读一帧六轴数据（一次突发读 14 字节，避免多次事务导致数据错位）
 *============================================================================*/
static uint8_t IMU_ReadSample(float *ax, float *ay, float *az,
                              float *gx, float *gy, float *gz)
{
  uint8_t buf[14];

  if ((IMU_ReadFrame(buf) == 0U) || (IMU_FrameUsable(buf) == 0U))
  {
    return 0U;
  }

  *ax = (float)(int16_t)(((uint16_t)buf[0] << 8) | buf[1]) / ACC_LSB_PER_G;
  *ay = (float)(int16_t)(((uint16_t)buf[2] << 8) | buf[3]) / ACC_LSB_PER_G;
  *az = (float)(int16_t)(((uint16_t)buf[4] << 8) | buf[5]) / ACC_LSB_PER_G;

  *gx = ((float)(int16_t)(((uint16_t)buf[8]  << 8) | buf[9])  - (float)s_ear.gyro_bias[0]) / GYRO_LSB_PER_DPS;
  *gy = ((float)(int16_t)(((uint16_t)buf[10] << 8) | buf[11]) - (float)s_ear.gyro_bias[1]) / GYRO_LSB_PER_DPS;
  *gz = ((float)(int16_t)(((uint16_t)buf[12] << 8) | buf[13]) - (float)s_ear.gyro_bias[2]) / GYRO_LSB_PER_DPS;

  return 1U;
}

/*==============================================================================
 *  数学小工具
 *============================================================================*/
static float ClampF(float v, float lo, float hi)
{
  if (v > hi) { return hi; }
  if (v < lo) { return lo; }
  return v;
}

static uint16_t ClampUs(float us)
{
  if (us < (float)SERVO_SAFE_MIN_US) { return SERVO_SAFE_MIN_US; }
  if (us > (float)SERVO_SAFE_MAX_US) { return SERVO_SAFE_MAX_US; }
  return (uint16_t)(us + 0.5f);
}

/* 角度 -> [-1,1]：中间留死区抑制静止抖动，两端 smoothstep 软化，动作不生硬 */
static float ShapeNorm(float angle, float deadband, float full)
{
  float a = fabsf(angle);
  float y;

  if (a <= deadband)
  {
    return 0.0f;
  }
  a = (a - deadband) / (full - deadband);
  if (a > 1.0f)
  {
    a = 1.0f;
  }
  y = a * a * (3.0f - 2.0f * a);
  return (angle < 0.0f) ? -y : y;
}

/*==============================================================================
 *  舵机输出
 *============================================================================*/
static void Servo_Apply(float us1, float us2)
{
  s_ear.out1 = (float)ClampUs(us1);
  s_ear.out2 = (float)ClampUs(us2);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)s_ear.out1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)s_ear.out2);
}

/* 匀速走到目标位置，供自检/故障提示使用 */
static void Servo_RampTo(uint16_t us1, uint16_t us2, uint8_t steps)
{
  uint16_t start1 = (uint16_t)(s_ear.out1 + 0.5f);
  uint16_t start2 = (uint16_t)(s_ear.out2 + 0.5f);
  uint8_t  i;

  if (steps < 2U) { steps = 2U; }

  for (i = 1U; i <= steps; i++)
  {
    float k = (float)i / (float)steps;
    Servo_Apply((float)start1 + ((float)us1 - (float)start1) * k,
                (float)start2 + ((float)us2 - (float)start2) * k);
    Ear_DelayMs(15U);
  }
}

/*==============================================================================
 *  姿态解算 + 混控输出
 *============================================================================*/
static void Ear_Update(float dt)
{
  float ax, ay, az, gx, gy, gz;
  g_ear_imu_diag[s_bus].phase = 6U;

  if (IMU_ReadSample(&ax, &ay, &az, &gx, &gy, &gz) != 0U)
  {
    float rate;
    float yrate;
    float acc_pitch;
    float m2;
    float dev;
    float w_acc;
    float k;
    float alpha;
    float corr;

    s_ear.fail_cnt = 0U;

    /* 僵尸传感器：总线通、读得到，但加速度计恒为 0。
     * 真实芯片静止时 az 约 1g（16384 LSB），绝不会整帧全 0；
     * 出现这种值基本是芯片掉电复位后进了 sleep，或根本不是这一族片子。 */
    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))
    {
      if (s_ear.zero_cnt < 0xFFFFU) { s_ear.zero_cnt++; }
      if (s_ear.zero_cnt >= 100U)
      {
        s_ear.imu_ok      = 0U;
        s_ear.fault_count = 6U;
        s_ear.zero_cnt    = 0U;
        s_ear.retry_tick  = HAL_GetTick();
        return;
      }
    }
    else
    {
      s_ear.zero_cnt = 0U;
    }

    /*====================== 俯仰（点头）====================================
     * 核心思路：加速度计负责"角度是多少"，陀螺负责"正在怎么动"。
     *
     *   atan2(ax, az) 是重力方向给出的绝对倾角，静止时没有漂移，但它怕两件事：
     *   抖动、以及头在前后加速时混进来的惯性力。
     *   陀螺积分短时间极平滑，但长时间必漂。
     *
     * 所以：先信任陀螺积分往前走一步，再让加速度计按权重把它"拉回真值"。
     * 权重不是常数，而是看两件事自动决定：
     *   1) 动得多快（rate 越大 -> alpha 越接近 1 -> 几乎只信陀螺）
     *   2) 合矢量离 1g 多远（跑跳/磕碰时直接不信加速度计）
     * 这样静止时零点被迅速校正干净，快速点头时动作又不会被拖住。
     *======================================================================*/
    rate = IMU_PITCH_RATE_SIGN * gy;
    s_ear.pitch_rate = rate;

    acc_pitch = atan2f(IMU_PITCH_ACC_SIGN * ax, az) * RAD_TO_DEG;

    /* 合矢量大小：静止时严格 1g，走路/跑跳/磕碰会明显偏离。
     *
     * 这里刻意不求平方根，改用 |a|^2 - 1 判断：
     * gcc 的 sqrtf 会把 newlib 的整套 stdio/errno 拖进来（实测多吃
     * 1.8KB Flash 和 700B RAM），对于 24KB/3KB 的片子代价太大。
     * 而 |a|^2-1 与 |a|-1 在 1g 附近是单调对应的，只要把门限也平方一下，
     * 权重曲线的形状完全一样，结论一模一样。 */
    m2  = (ax * ax) + (ay * ay) + (az * az);
    dev = fabsf(m2 - 1.0f);            /* 2*(|a|-1g) 的近似 */

    if (dev <= (2.0f * ACC_TRUST_BAND_G))
    {
      w_acc = 1.0f;                                  /* 完全可信 */
    }
    else if (dev >= (2.0f * ACC_REJECT_BAND_G))
    {
      w_acc = 0.0f;                                  /* 完全是惯性力，弃用 */
    }
    else
    {
      w_acc = 1.0f - ((dev - (2.0f * ACC_TRUST_BAND_G)) /
                      (2.0f * (ACC_REJECT_BAND_G - ACC_TRUST_BAND_G)));
    }

    /* 运动强度 0..1：决定这一帧有多信任陀螺 */
    k = fabsf(rate) / FUSE_RATE_KNEE_DPS;
    if (k > 1.0f) { k = 1.0f; }

    alpha = FUSE_ALPHA_STATIC + ((FUSE_ALPHA_MOVING - FUSE_ALPHA_STATIC) * k);
    corr  = (1.0f - alpha) * w_acc;

    s_ear.pitch_deg += rate * dt;                              /* 陀螺：动态 */
    s_ear.pitch_deg += (acc_pitch - s_ear.pitch_deg) * corr;   /* 加计：零点 */
    s_ear.pitch_deg  = ClampF(s_ear.pitch_deg,
                              -PITCH_LIMIT_DEG, PITCH_LIMIT_DEG);

    /*====================== 偏航（摇头）====================================
     * 没有磁力计，加速度计也推不出"头朝哪边"，所以转头角只能对 gyro_z 积分。
     * 纯积分一定漂，因此配合一个"漏"：
     *   正在转 -> 正常积分 + 很慢的泄漏（慢到不影响跟随，只是防止跑飞）
     *   停下来 -> 按 YAW_LEAK_TAU_S 衰减回中
     * 于是"扭头耳朵跟着甩过去，停住后自己慢慢回正"就是天然行为，
     * 而且静止时的残余零漂会被这个泄漏直接吃掉，不需要额外的漂移补偿。
     *======================================================================*/
    yrate = IMU_YAW_RATE_SIGN * gz;
    s_ear.yaw_rate = yrate;

#if (YAW_HOLD_MODE != 0)
    /* 保持角度：转到哪停在哪。没有绝对参考，长时间必漂，只适合演示 */
    if (fabsf(yrate) >= YAW_RATE_DEADBAND)
    {
      s_ear.yaw_deg += yrate * dt;
      s_ear.yaw_deg  = ClampF(s_ear.yaw_deg, -YAW_LIMIT_DEG, YAW_LIMIT_DEG);
    }
#else
    if (fabsf(yrate) < YAW_RATE_DEADBAND)
    {
      /* 静止：按较快的常数漏回中位 */
      s_ear.yaw_deg *= (1.0f - (dt / YAW_LEAK_TAU_S));
    }
    else
    {
      /* 转动：正常积分，只叠一个很慢的泄漏。
       * 慢泄漏不是用来回中的，它是保险丝：万一零偏没标干净，
       * 积分会一直往一边涨，最后把舵机顶在限位上不动弹；
       * 有了这道慢泄漏，偏差最多稳在"零偏 × 20s"的位置，拆掉负载就自己回来。 */
      s_ear.yaw_deg += yrate * dt;
      s_ear.yaw_deg *= (1.0f - (dt / YAW_LEAK_TAU_SLOW_S));
      s_ear.yaw_deg  = ClampF(s_ear.yaw_deg, -YAW_LIMIT_DEG, YAW_LIMIT_DEG);
    }
#endif
  }
  else
  {
    if (s_ear.bus_err < 0xFFU) { s_ear.bus_err++; }
    if (s_ear.fail_cnt < 0xFFU) { s_ear.fail_cnt++; }
    if (s_ear.fail_cnt >= IMU_FAIL_LIMIT)
    {
      s_ear.imu_ok = 0U;
      s_ear.fault_count = 4U;   /* 读失败；1s 后重连会重新定标成 3/5/6 */
      g_ear_imu_diag[s_bus].fault = 4U;
      s_ear.pitch_rate = 0.0f;
      s_ear.yaw_rate = 0.0f;

#if (EAR_DBG_SNAPSHOT != 0)
      /* "配置得进去、数据读不回来"是最容易让人瞎猜的一种故障：
       * 芯片明明应答，却要不到 0x3B 的字节。既然 DAPLink 一直贴着，
       * 就把整片寄存器摊开来给 PC 看（含每个寄存器的应答状态），不用再猜。
       * 进去之后两路舵机停在中位，不再周期性抽动。 */
      Ear_FaultIndicate();
      Ear_DbgSnapshotLoop();
#endif
    }
  }

  Ear_Output();
}

#if (TURN_SYNC_MODE == EAR_SYNC_AFTER_MOVE) || (NOD_SYNC_MODE == EAR_SYNC_AFTER_MOVE)
/*----------------------------------------------------------------------------
 *  "等动作做完再动"的门控
 *
 *  输入：本帧的归一化指令 target、该轴的角速度（deg/s）、补位步长 step
 *  输出：真正送给舵机的指令
 *
 *  状态机只有三个变量：
 *    moving  头正在动 -> 输出冻结在 out 上，一动不动
 *    still   连续静止帧数，攒够 SYNC_STILL_FRAMES 就认为动作结束
 *    hold    冻结了多少帧，超过 SYNC_HOLD_MAX_FRAMES 强制放行
 *
 *  这里刻意不做滤波：角速度本身已经是互补滤波里用过的那一份，
 *  再滤一次只会让"动作结束"判得更晚，等于又变回固定延迟。
 *--------------------------------------------------------------------------*/
static float SyncGate_Update(SyncGate_t *g, float target, float rate_dps, float step)
{
  float d;

  if (g->primed == 0U)
  {
    /* 第一帧直接对齐，省得上电后从 0 慢慢爬过去 */
    g->primed = 1U;
    g->out    = target;
    return g->out;
  }

  if (fabsf(rate_dps) > SYNC_MOVE_ON_DPS)
  {
    /* 明确在动：站住，重新开始数静止帧 */
    g->moving = 1U;
    g->still  = 0U;
    g->hold   = 0U;
  }
  else if (g->moving != 0U)
  {
    if (fabsf(rate_dps) < SYNC_STILL_DPS)
    {
      if (g->still < 0xFFU) { g->still++; }
      if (g->still >= SYNC_STILL_FRAMES)
      {
        g->moving = 0U;      /* 判定动作结束，这一帧就开始补位 */
        g->still  = 0U;
      }
    }
    else
    {
      g->still = 0U;         /* 介于两档之间：还在慢悠悠地动，重新数 */
    }

    if (g->hold < 0xFFU) { g->hold++; }
    if (g->hold >= SYNC_HOLD_MAX_FRAMES)
    {
      g->moving = 0U;        /* 转太久就别装死了，边走边跟上 */
      g->still  = 0U;
      g->hold   = 0U;
    }
  }

  if (g->moving != 0U)
  {
    return g->out;           /* 动作中：舵机原地不动 */
  }

  /* 动作结束（或本来就静止）：以 step 为速度上限补向目标。
   * step 直接用归一化量，换算关系是 step = CATCHUP_US / SPAN_US，
   * 所以"补满整段行程需要多少帧"跟行程大小无关，调起来很直观。 */
  d = target - g->out;
  g->out += ClampF(d, -step, step);
  return g->out;
}
#endif /* 有轴在用 AFTER_MOVE */

/* 把姿态换算成两路脉宽
 *
 * 两级映射，缺一不可：
 *   角度 -> 归一化量：死区吃掉静止抖动，smoothstep 让起步和收尾都不生硬
 *   归一化量 -> 脉宽：中间叠加角速度前馈，快速动作时"抢一拍"
 *
 * 前馈为什么必要：纯角度的响应是"跟着角度走"，快速点头时耳朵总是慢半拍；
 * 加一点角速度前馈，起手瞬间就先把耳朵带出去，停下来前馈自动归零，
 * 不会留下偏移。手感差别很大，这就是"活"和"木"的分界。
 */
static void Ear_Output(void)
{
  float nod, turn, cmd1, cmd2;
  float target1, target2;
  float dir1 = (EAR_CH1_INVERT != 0) ? -1.0f : 1.0f;
  float dir2 = (EAR_CH2_INVERT != 0) ? -1.0f : 1.0f;

  /* 1) 角度 -> 归一化指令（带死区与平滑）
   *    俯仰用"相对上电姿态"的偏差，转头直接用累计角 */
  nod  = ShapeNorm(s_ear.pitch_deg - s_ear.pitch_ref, NOD_DEADBAND_DEG, NOD_FULL_DEG);
  turn = ShapeNorm(s_ear.yaw_deg, TURN_DEADBAND_DEG, TURN_FULL_DEG);

  /* 2) 角速度前馈：快速抬头/转头时先给力，停住后自动回中，手感干脆 */
  nod  += ClampF(s_ear.pitch_rate * NOD_RATE_FF,  -NOD_RATE_FF_LIMIT,  NOD_RATE_FF_LIMIT);
  turn += ClampF(s_ear.yaw_rate  * TURN_RATE_FF, -TURN_RATE_FF_LIMIT, TURN_RATE_FF_LIMIT);

  /* 3) 反应方式：让耳朵比头"慢半拍"
   *
   * AFTER_MOVE 是默认：用陀螺看"头还在不在动"。
   *   动 -> 舵机原地冻结（指令在变，输出不动）
   *   连续静止 70ms -> 判定动作结束，之后用受限速度补到新位置
   * 这样不管转得快还是慢，"头甩过去、耳朵愣一下再追"的观感都是恒定的。
   * 注意冻结的只是舵机输出，姿态解算、回中、死区全都还是实时的。
   */
#if (TURN_SYNC_MODE == EAR_SYNC_NONE)
  /* 直通：什么都不做 */
#elif (TURN_SYNC_MODE == EAR_SYNC_AFTER_MOVE)
  turn = SyncGate_Update(&s_sync_turn, turn, s_ear.yaw_rate,
                         (float)TURN_CATCHUP_US / (float)EAR_CH2_SPAN_US);
#endif

#if (NOD_SYNC_MODE == EAR_SYNC_AFTER_MOVE)
  nod = SyncGate_Update(&s_sync_nod, nod, s_ear.pitch_rate,
                        (float)NOD_CATCHUP_US / (float)EAR_CH1_SPAN_US);
#endif

#if (TURN_SYNC_MODE == EAR_SYNC_FIXED_DELAY) || (NOD_SYNC_MODE == EAR_SYNC_FIXED_DELAY)
  /* 老的固定延迟：把归一化指令压进环形缓冲，按各自的长度从后面取旧值。
   * 延迟作用在"指令"上而不是"姿态"上：解算、回中、死区全都实时，
   * 只是舵机慢一拍跟上，所以不会出现"转过头了角度还没算出来"的错觉。 */
  {
    uint8_t dn = (uint8_t)((NOD_DELAY_MS  + (IMU_PERIOD_MS / 2U)) / IMU_PERIOD_MS);
    uint8_t dt = (uint8_t)((TURN_DELAY_MS + (IMU_PERIOD_MS / 2U)) / IMU_PERIOD_MS);

    if (dn >= EAR_DLY_SAMPLES) { dn = EAR_DLY_SAMPLES - 1U; }
    if (dt >= EAR_DLY_SAMPLES) { dt = EAR_DLY_SAMPLES - 1U; }

    s_dly_nod[s_dly_wr]  = nod;
    s_dly_turn[s_dly_wr] = turn;

    /* dn = 0 时索引就是刚写进去的位置，等价于直通 */
    nod  = s_dly_nod[(uint8_t)((s_dly_wr + EAR_DLY_SAMPLES - dn) % EAR_DLY_SAMPLES)];
    turn = s_dly_turn[(uint8_t)((s_dly_wr + EAR_DLY_SAMPLES - dt) % EAR_DLY_SAMPLES)];

    s_dly_wr = (uint8_t)((s_dly_wr + 1U) % EAR_DLY_SAMPLES);
  }
#endif

  /* 4) 点头 Q 弹：二阶欠阻尼弹簧
   *    y'' = w²·(target - y) - 2·ζ·w·y'
   *    用半隐式欧拉（先更新速度再更新位置），比显式欧拉稳定，
   *    步长固定取采样周期，这样即使主循环偶尔被 I2C 拖慢，
   *    弹簧的频率也不会变 —— 频率一变，"几下停住"就不可控了。 */
  {
    const float sdt = (float)IMU_PERIOD_MS * 0.001f;
    const float w   = 6.2831853f * NOD_SPRING_HZ;
    float acc = (w * w) * (nod - s_spr_pos)
              - (2.0f * NOD_SPRING_ZETA * w * s_spr_vel);

    s_spr_vel += acc * sdt;
    s_spr_pos += s_spr_vel * sdt;
    nod = s_spr_pos;
  }

  nod  = ClampF(nod,  -1.0f, 1.0f);
  turn = ClampF(turn, -1.0f, 1.0f);

  /* 5) 混控 */
#if (EAR_MECH_MODE == EAR_MECH_DUAL_EAR)
  /* 每路一只耳朵：抬头两耳同时竖起；转头时左耳后倒、右耳前倾（反相） */
  cmd1 = nod - turn * EAR_TURN_MIX;   /* 左耳 */
  cmd2 = nod + turn * EAR_TURN_MIX;   /* 右耳 */
#else
  /* 云台式（当前结构）：两路各管一个自由度，互不干扰
   *   CH1 = 点头前后，CH2 = 摇头左右 */
  cmd1 = nod;
  cmd2 = turn;
#endif
  cmd1 = ClampF(cmd1, -1.0f, 1.0f);
  cmd2 = ClampF(cmd2, -1.0f, 1.0f);

  /* 6) 归一化 -> 脉宽 */
  target1 = (float)EAR_CH1_CENTER_US + (float)EAR_CH1_TRIM_US
          + dir1 * cmd1 * (float)EAR_CH1_SPAN_US;
  target2 = (float)EAR_CH2_CENTER_US + (float)EAR_CH2_TRIM_US
          + dir2 * cmd2 * (float)EAR_CH2_SPAN_US;

  /* 7) 斜率限制：限制舵机转速，避免甩动与齿轮冲击 */
  s_ear.out1 += ClampF(target1 - s_ear.out1, -(float)SERVO_SLEW_US, (float)SERVO_SLEW_US);
  s_ear.out2 += ClampF(target2 - s_ear.out2, -(float)SERVO_SLEW_US, (float)SERVO_SLEW_US);

  Servo_Apply(s_ear.out1, s_ear.out2);
}

/*==============================================================================
 *  频率编码遥测（诊断用）
 *
 *  为什么要有这个东西：舵机只能"抽 4 下"，而 4 这个数字背后可能是好几种
 *  完全不同的物理原因。既然手上只有万用表，那就把万用表的频率档当数据线用：
 *  把 0~255 的数直接变成频率，读数就是数值本身，不用示波器、不用再改一轮代码。
 *
 *      f = 800 + 25 × 数值   (Hz)
 *        0 -> 0.800kHz    3 -> 0.875kHz   15 -> 1.175kHz
 *       64 -> 2.400kHz  255 -> 7.175kHz
 *
 *  每个值响 3 秒、静音 1.5 秒，循环往复。顺序固定为 A~F 六项，见头文件说明。
 *============================================================================*/
#define TELEM_PIN_MASK    (1UL << 0)     /* 用 PA0（舵机 1 的信号脚）输出 */

static void Telem_PinInit(void)
{
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
  GPIOA->OTYPER &= ~TELEM_PIN_MASK;
  GPIOA->MODER   = (GPIOA->MODER & ~(3UL << 0)) | (1UL << 0);   /* 通用推挽输出 */
  GPIOA->BSRR    = (TELEM_PIN_MASK << 16);                      /* 先拉低 */
}

/* 精确微秒延时：直接借 TIM1 的计数器当时基。
 * TIM1 的 PSC=23 -> 计数 1µs，ARR=19999 -> 20ms 一个周期，
 * 精度比"数循环周期"高一个数量级 —— 遥测的相邻数值只差 25Hz，
 * 用空循环（误差 2~3%）在低频段会糊成一片。 */
static void Telem_WaitUs(uint16_t us)
{
  uint16_t period;
  uint16_t t0;
  uint16_t now;
  uint16_t elapsed;

  if ((TIM1->CR1 & TIM_CR1_CEN) == 0U)
  {
    Ear_DelayUs(us);              /* 定时器没跑（不该发生），退回空循环 */
    return;
  }

  period = (uint16_t)(TIM1->ARR + 1U);
  t0 = (uint16_t)TIM1->CNT;

  for (;;)
  {
    now = (uint16_t)TIM1->CNT;
    elapsed = (now >= t0) ? (uint16_t)(now - t0) : (uint16_t)(now + period - t0);
    if (elapsed >= us) { break; }
  }
}

static void Telem_Tone(uint8_t value)
{
  uint32_t f    = 800UL + (uint32_t)value * 25UL;
  uint32_t half = 500000UL / f;          /* 半周期（µs）：f=800 时 625µs */
  uint32_t n    = f * 3UL;               /* 约 3 秒 */
  uint32_t i;

  if (half < 6UL) { half = 6UL; }

  for (i = 0U; i < n; i++)
  {
    GPIOA->BSRR = TELEM_PIN_MASK;
    Telem_WaitUs((uint16_t)half);
    GPIOA->BSRR = (TELEM_PIN_MASK << 16);
    Telem_WaitUs((uint16_t)half);

    if ((i & 0x3FFU) == 0U) { Wdg_Kick(); }
  }
  GPIOA->BSRR = (TELEM_PIN_MASK << 16);
}

static void Telem_Gap(void)
{
  GPIOA->BSRR = (TELEM_PIN_MASK << 16);
  Ear_DelayMs(1500U);
}

/* 裸单字节读：绕开所有降级逻辑，直接报告卡在哪一步
 *   0 = 成功  1 = 地址字节无应答  2 = 寄存器字节无应答  3 = 读地址无应答 */
static uint8_t Raw_Probe(uint8_t addr, uint8_t reg, uint8_t *val)
{
  uint8_t ack;

  SW_Start();
  ack = SW_WriteByte((uint8_t)(addr << 1));
  if (ack != 0U) { SW_Stop(); return 1U; }

  ack = SW_WriteByte(reg);
  if (ack != 0U) { SW_Stop(); return 2U; }

  SW_Start();
  ack = SW_WriteByte((uint8_t)((addr << 1) | 0x01U));
  if (ack != 0U) { SW_Stop(); return 3U; }

  *val = SW_ReadByte(0U);
  SW_Stop();
  return 0U;
}

/* 读失败统一编码成 255（7.175kHz），和"真的读到 255"区分不了也没关系，
 * 因为配套的状态位会一起发出来 */
static uint8_t Raw_ProbeValue(uint8_t addr, uint8_t reg)
{
  uint8_t val = 0U;

  if (Raw_Probe(addr, reg, &val) != 0U) { return 0xFFU; }
  return val;
}

void Ear_TelemetryReport(void)
{
  uint8_t addr = s_ear.imu_addr;
  uint8_t val;
  uint8_t st;

  /* 只有软件总线才能报告"卡在第几个字节"，所以强制切回来 */
  Bus_SetMode(BUS_SOFT);
  Telem_PinInit();

  if ((addr == 0U) || (addr == 0xFFU)) { addr = IMU_ADDR_PRIMARY; }

  for (;;)
  {
    /* A：WHO_AM_I 真实值 */
    Telem_Tone(Raw_ProbeValue(addr, MPU_REG_WHO_AM_I));
    Telem_Gap();

    /* B：读数据寄存器 0x3B 卡在哪一步。
     * 这一步只有 4 种状态，所以刻意拉开间距（每态差 800Hz），
     * 就算频率有偏差也不会读错：
     *   0 成功   -> 0.800kHz
     *   1 地址无应答 -> 1.600kHz
     *   2 寄存器无应答 -> 2.400kHz
     *   3 读地址无应答 -> 3.200kHz */
    val = 0U;
    st  = Raw_Probe(addr, MPU_REG_ACCEL_XOUT_H, &val);
    Telem_Tone((uint8_t)(st * 32U));
    Telem_Gap();

    /* C：0x3B 读到的值 */
    Telem_Tone((st == 0U) ? val : 0xFFU);
    Telem_Gap();

    /* D：0x3F = ACCEL_ZOUT_H，静止时约 0x40(64) */
    Telem_Tone(Raw_ProbeValue(addr, (uint8_t)(MPU_REG_ACCEL_XOUT_H + 4U)));
    Telem_Gap();

    /* E：0x41 = TEMP_OUT_H，常温约 0x0F(15) */
    Telem_Tone(Raw_ProbeValue(addr, MPU_REG_TEMP_OUT_H));
    Telem_Gap();

    /* F：0x1A = CONFIG，配置写过后应为 3（变体 2 下为 0） */
    Telem_Tone(Raw_ProbeValue(addr, MPU_REG_CONFIG));
    Telem_Gap();
  }
}

/*==============================================================================
 *  SWD 在线快照（诊断用）
 *
 *  有了 DAPLink 就不用再靠万用表一个数一个数抄了：固件把"现场"持续写进
 *  RAM 的固定地址，PC 端随时整块读走，CPU 照常在跑，不打断任何东西。
 *
 *  为什么用固定地址而不是普通全局变量：
 *  每次重新编译全局变量的地址都可能变（多个变量、改个宏就挪位），
 *  写死的地址才能保证 PC 端的读脚本永远不用改。0x20000500 这一段
 *  在 .bss 之后（实测 .bss 结束于 0x200003D8），离栈顶 0x20000C00
 *  还有 1.5KB 余量，不会和栈打架。
 *============================================================================*/
#define EAR_DBG_ADDR      0x20000500UL
#define EAR_DBG_MAGIC     0x44524145UL      /* 小端读出为 "EARD" */

typedef struct
{
  uint32_t magic;         /* 0x00 */
  uint32_t seq;           /* 0x04 刷新计数 */
  uint32_t tick;          /* 0x08 SysTick 毫秒 */
  uint32_t fault;         /* 0x0C 故障码 */
  uint8_t  ack68;         /* 0x10 */
  uint8_t  who68;         /* 0x11 */
  uint8_t  ack69;         /* 0x12 */
  uint8_t  who69;         /* 0x13 */
  uint8_t  st68[4];       /* 0x14 0x75/0x3B/0x41/0x1A 的读取状态 */
  uint8_t  st69[4];       /* 0x18 */
  uint8_t  nerr;          /* 0x1C 读失败的寄存器个数 */
  uint8_t  pad[3];
  uint8_t  reg[128];      /* 0x20 寄存器 0x00~0x7F 镜像 */
  uint8_t  status[128];   /* 0xA0 每个寄存器的读取状态 */
} EarDbg_t;

#define EAR_DBG  ((volatile EarDbg_t *)EAR_DBG_ADDR)

/* 四个关键寄存器各读一次，状态写进 st[]，成功读到的值带回来 */
static void Dbg_KeyProbe(uint8_t addr, volatile uint8_t *st, uint8_t *vals)
{
  static const uint8_t key[4] =
  {
    MPU_REG_WHO_AM_I, MPU_REG_ACCEL_XOUT_H, MPU_REG_TEMP_OUT_H, MPU_REG_CONFIG
  };
  uint8_t i;
  uint8_t v;

  for (i = 0U; i < 4U; i++)
  {
    v = 0U;
    st[i]   = Raw_Probe(addr, key[i], &v);
    vals[i] = v;
  }
}

/* 把整片现场刷进 RAM（调用前保证已经在软件 I2C 模式） */
static void Ear_DbgFill(void)
{
  volatile EarDbg_t *d = EAR_DBG;
  uint8_t keyv[4];
  uint8_t i;
  uint8_t v;
  uint8_t st;
  uint8_t addr;

  /* 两个地址都探一遍，谁应答就用谁做后面的全量 dump */
  v = 0U;
  st = Raw_Probe(IMU_ADDR_PRIMARY, MPU_REG_WHO_AM_I, &v);
  d->ack68 = (st == 0U) ? 1U : 0U;
  d->who68 = (st == 0U) ? v : 0xFFU;
  Dbg_KeyProbe(IMU_ADDR_PRIMARY, d->st68, keyv);

  v = 0U;
  st = Raw_Probe(IMU_ADDR_SECONDARY, MPU_REG_WHO_AM_I, &v);
  d->ack69 = (st == 0U) ? 1U : 0U;
  d->who69 = (st == 0U) ? v : 0xFFU;
  Dbg_KeyProbe(IMU_ADDR_SECONDARY, d->st69, keyv);

  if (d->ack68 != 0U)      { addr = IMU_ADDR_PRIMARY; }
  else if (d->ack69 != 0U) { addr = IMU_ADDR_SECONDARY; }
  else                     { addr = IMU_ADDR_PRIMARY; }

  d->nerr = 0U;
  for (i = 0U; i < 128U; i++)
  {
    v  = 0U;
    st = Raw_Probe(addr, i, &v);
    d->reg[i]    = v;
    d->status[i] = st;
    if (st != 0U) { d->nerr++; }
    if ((i & 0x0FU) == 0U) { Wdg_Kick(); }
  }
}

void Ear_DbgSnapshotLoop(void)
{
  EarDbg_t *d = (EarDbg_t *)EAR_DBG_ADDR;

  for (;;)
  {
    /* 两路舵机停在中位：不再抽动。静止 = 固件已经进入快照模式，
     * 此时数据只在 RAM 里，用 SWD 读即可。 */
    Servo_Apply((float)(EAR_CH1_CENTER_US + EAR_CH1_TRIM_US),
                (float)(EAR_CH2_CENTER_US + EAR_CH2_TRIM_US));

    /* 强制切回软件总线：只有它才能报告"卡在第几个字节" */
    Bus_SetMode(BUS_SOFT);

    /* 0x20000500 这一段不在 .bss 里，上电不会清 0：第一次进来时先把序号归零，
     * 之后 PC 端读到的 seq 才是"刷新了多少次"，而不是一段随机垃圾 */
    if (d->magic != EAR_DBG_MAGIC)
    {
      d->seq = 0U;
    }

    d->magic = EAR_DBG_MAGIC;
    d->tick  = HAL_GetTick();
    d->fault = (uint32_t)s_ear.fault_count;

    Ear_DbgFill();

    d->seq++;               /* 写在最后：PC 端看到 seq 变化就说明数据是新的 */
    Ear_DelayMs(400U);
  }
}

/*==============================================================================
 *  对外接口
 *============================================================================*/

/*-- 读法提示：慢慢地点 n 下，告诉你陀螺仪数据帧用的是第几级读法 ------------
 *   1 下 = 14 字节突发读（最理想，每帧 1 次事务）
 *   2 下 = 2 字节配对读（每帧 4 次事务）
 *   3 下 = 逐字节读（每帧 8 次事务，最慢但最兼容）
 * 节奏是"慢点"，和故障码那种"急促抽动"一眼就能分开。
 *--------------------------------------------------------------------------*/
static void Ear_ModeIndicate(uint8_t n) __attribute__((unused));
static void Ear_ModeIndicate(uint8_t n)
{
  uint16_t mid1 = ClampUs((float)EAR_CH1_CENTER_US + (float)EAR_CH1_TRIM_US);
  uint16_t mid2 = ClampUs((float)EAR_CH2_CENTER_US + (float)EAR_CH2_TRIM_US);
  uint16_t amp1 = (uint16_t)(((EAR_CH1_SPAN_US > 400U) ? 400U : EAR_CH1_SPAN_US) / 4U);
  uint16_t amp2 = (uint16_t)(((EAR_CH2_SPAN_US > 400U) ? 400U : EAR_CH2_SPAN_US) / 4U);
  uint8_t  i;
  uint8_t  hold_backup = s_ear.hold;

  if (n == 0U) { return; }

  s_ear.hold = 1U;

  for (i = 0U; i < n; i++)
  {
    Servo_RampTo((uint16_t)(mid1 - amp1), (uint16_t)(mid2 + amp2), 8U);
    Ear_DelayMs(180U);
    Servo_RampTo((uint16_t)(mid1 + amp1), (uint16_t)(mid2 - amp2), 8U);
    Ear_DelayMs(180U);
  }
  Servo_RampTo(mid1, mid2, 8U);

  s_ear.hold = hold_backup;
}

void Ear_Init(void)
{
  memset(&s_ear, 0, sizeof(s_ear));
  s_ear.imu_addr = IMU_ADDR_PRIMARY;

  /* 先把两路拉到中位，避免上电瞬间舵机乱甩 */
  s_ear.out1 = (float)EAR_CH1_CENTER_US + (float)EAR_CH1_TRIM_US;
  s_ear.out2 = (float)EAR_CH2_CENTER_US + (float)EAR_CH2_TRIM_US;
  Servo_Apply(s_ear.out1, s_ear.out2);
  Ear_DelayMs(200U);

  /*--------------------------------------------------------------------------
   * 顺序很重要：机械动作必须发生在任何 I2C 操作之前。
   * 否则一旦陀螺仪那边卡住或反复重试，"上电没动作"就无法区分是
   * PWM/舵机侧的问题还是传感器侧的问题 —— 这正是最容易误判的地方。
   *------------------------------------------------------------------------*/

#if (EAR_PWM_ONLY_TEST != 0)
  /* 纯 PWM 台架模式：完全不碰传感器，两路舵机无限慢扫 */
  for (;;)
  {
    Ear_SelfTest();
    Ear_DelayMs(400U);
  }
#endif

#if (EAR_BOOT_BEACON_S > 0U)
  /* 第一步：开机信标 —— 极慢地左右摆，全程不碰传感器。
   * 看到它 = MCU 在跑 + 定时器在输出 + 舵机有电 + 机械没卡死，全通。
   * 每次半程 60 步 × 15ms ≈ 0.9s，一个来回约 1.8s，很好数。 */
  Ear_BeaconSwing((uint16_t)(((uint32_t)EAR_BOOT_BEACON_S * 1000U) / 900U));
#elif (EAR_BOOT_SELFTEST != 0)
  /* 关掉信标时只做一次快速自检 */
  Ear_SelfTest();
#endif

  /*--------------------------------------------------------------------------
   * 节拍自检：HAL_Delay / Ear_Task 全部依赖 SysTick 递增。
   * 如果 tick 没起来，固件会"上电有动作、之后永远静止"，症状和死机一样。
   * 这里用忙等对比两次 tick，死了就先尝试重建，仍然不行就报故障码 7。
   *------------------------------------------------------------------------*/
  {
    uint32_t t0 = HAL_GetTick();

    Ear_DelayMs(30U);
    if (HAL_GetTick() == t0)
    {
      (void)HAL_InitTick(TICK_INT_PRIORITY);   /* 尝试重建时间基准 */

      t0 = HAL_GetTick();
      Ear_DelayMs(30U);
      if (HAL_GetTick() == t0)
      {
        /* 确实没救：一直闪故障码 7，让"tick 死了"这件事看得见 */
        s_ear.fault_count = 7U;
        for (;;)
        {
          Ear_FaultIndicate();
          Ear_DelayMs(1200U);
        }
      }
    }
  }

  /* 第二步：建立总线并启动传感器。
   * IMU_BringUp 内部会先试硬件 I2C，不行再退回软件 I2C，两条都失败才返回 0。 */
  if (IMU_BringUp() != 0U)
  {
    s_ear.imu_ok = 1U;

    /* 报一下实际用的是第几级读法（1/2/3 下慢点），方便判断这颗片子
     * 到底支持多长的事务。默认关闭，否则上电动作里混进几下抽动，
     * 会让人以为是故障提示。 */
#if (EAR_MODE_INDICATE != 0)
    Ear_ModeIndicate((uint8_t)(s_rd_mode + 1U));
#endif
  }
  else
  {
#if (EAR_DBG_SNAPSHOT != 0)
    /* 先用抽动把故障码报一次（现象可见），然后进入"只写 RAM"的静默循环，
     * 等 PC 端用 DAPLink 把 0x20000500 那 288 字节读走。 */
    Ear_FaultIndicate();
    Ear_DbgSnapshotLoop();
#elif (EAR_TELEM_MODE != 0)
    /* 传感器起不来：把寄存器的真实内容用频率报出来，不再靠"抽几下"猜。
     * 进去就不返回了，PA0 会被从定时器手里抢走（两路舵机都停止输出）。 */
    Ear_TelemetryReport();
#else
    /* 没找到传感器：急促抽动，表示"总线没通"，避免误以为死机 */
    Ear_FaultIndicate();
#endif
  }

  s_ear.tick       = HAL_GetTick();
  s_ear.retry_tick = s_ear.tick;
  s_ear.fault_tick = s_ear.tick;
}

void Ear_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - s_ear.tick;
  float dt;

  Wdg_Kick();

  if (elapsed < (uint32_t)IMU_PERIOD_MS)
  {
    return;
  }
  s_ear.tick = now;

  dt = (float)elapsed * 0.001f;
  if (dt > 0.05f) { dt = 0.05f; }

  /* 传感器掉线：定期重连，期间耳朵回中并周期提示 */
  if (s_ear.imu_ok == 0U)
  {
    if ((now - s_ear.retry_tick) >= IMU_RETRY_MS)
    {
      s_ear.retry_tick = now;

      /* 重连会依次试硬件/软件两条总线，各自内部已做总线恢复 */
      if (IMU_BringUp() != 0U)
      {
        s_ear.imu_ok    = 1U;
        /* IMU_Calibrate 已设置姿态零点，保留重连时的新标定结果。 */
      }
    }

    /* 掉线期间让耳朵平滑回中，不要僵在最后一次动作上 */
    s_ear.pitch_deg += (s_ear.pitch_ref - s_ear.pitch_deg) * 0.10f;
    s_ear.yaw_deg *= 0.90f;
    s_ear.pitch_rate = 0.0f;
    s_ear.yaw_rate = 0.0f;

    if (s_ear.hold == 0U)
    {
      Ear_Output();
    }

#if (EAR_FAULT_INDICATE != 0)
    if ((now - s_ear.fault_tick) >= EAR_FAULT_PERIOD_MS)
    {
      s_ear.fault_tick = now;
      Ear_FaultIndicate();
    }
#endif
    return;
  }

  if (s_ear.hold == 0U)
  {
    Ear_Update(dt);
  }
}

uint8_t Ear_IsReady(void)
{
  return s_ear.imu_ok;
}

void Ear_GetState(EarState_t *st)
{
  if (st == NULL) { return; }

  st->pitch_deg      = s_ear.pitch_deg - s_ear.pitch_ref;
  st->yaw_deg        = s_ear.yaw_deg;
  st->pitch_rate_dps = s_ear.pitch_rate;
  st->yaw_rate_dps   = s_ear.yaw_rate;
  st->out1_us        = (uint16_t)(s_ear.out1 + 0.5f);
  st->out2_us        = (uint16_t)(s_ear.out2 + 0.5f);
  st->imu_ok         = s_ear.imu_ok;
  st->imu_addr       = s_ear.imu_addr;
  st->imu_whoami     = s_ear.whoami;
  st->bus_err        = s_ear.bus_err;
}

/* 手动打角：装配/找行程时用。传 (0,0) 退出并恢复闭环控制 */
void Ear_SetPulseRaw(uint16_t us1, uint16_t us2)
{
  if ((us1 == 0U) && (us2 == 0U))
  {
    s_ear.hold = 0U;
    s_ear.tick = HAL_GetTick();
    return;
  }
  s_ear.hold = 1U;
  s_ear.out1 = (float)ClampUs((float)us1);
  s_ear.out2 = (float)ClampUs((float)us2);
  Servo_Apply(s_ear.out1, s_ear.out2);
}

/* 平滑长扫：确认机械行程（找到传感器后上电自动执行一次） */
void Ear_SelfTest(void)
{
  uint16_t min1 = ClampUs((float)EAR_CH1_CENTER_US + (float)EAR_CH1_TRIM_US - (float)EAR_CH1_SPAN_US);
  uint16_t max1 = ClampUs((float)EAR_CH1_CENTER_US + (float)EAR_CH1_TRIM_US + (float)EAR_CH1_SPAN_US);
  uint16_t min2 = ClampUs((float)EAR_CH2_CENTER_US + (float)EAR_CH2_TRIM_US - (float)EAR_CH2_SPAN_US);
  uint16_t max2 = ClampUs((float)EAR_CH2_CENTER_US + (float)EAR_CH2_TRIM_US + (float)EAR_CH2_SPAN_US);
  uint16_t mid1 = ClampUs((float)EAR_CH1_CENTER_US + (float)EAR_CH1_TRIM_US);
  uint16_t mid2 = ClampUs((float)EAR_CH2_CENTER_US + (float)EAR_CH2_TRIM_US);

  Servo_RampTo(min1, min2, 10U);
  Servo_RampTo(max1, max2, 20U);
  Servo_RampTo(mid1, mid2, 20U);
}

/* 开机信标：慢慢地来回摆 half_swings 个半程，最后回中。
 * 每一步只有 15ms、每个半程 60 步（约 0.9s），慢到肉眼一定能跟上，
 * 也慢到供电偏弱的板子能跟得上而不会一激动就掉电。 */
static void Ear_BeaconSwing(uint16_t half_swings)
{
  uint16_t min1 = ClampUs((float)EAR_CH1_CENTER_US + (float)EAR_CH1_TRIM_US - (float)EAR_CH1_SPAN_US);
  uint16_t max1 = ClampUs((float)EAR_CH1_CENTER_US + (float)EAR_CH1_TRIM_US + (float)EAR_CH1_SPAN_US);
  uint16_t min2 = ClampUs((float)EAR_CH2_CENTER_US + (float)EAR_CH2_TRIM_US - (float)EAR_CH2_SPAN_US);
  uint16_t max2 = ClampUs((float)EAR_CH2_CENTER_US + (float)EAR_CH2_TRIM_US + (float)EAR_CH2_SPAN_US);
  uint16_t mid1 = ClampUs((float)EAR_CH1_CENTER_US + (float)EAR_CH1_TRIM_US);
  uint16_t mid2 = ClampUs((float)EAR_CH2_CENTER_US + (float)EAR_CH2_TRIM_US);
  uint16_t n;
  uint8_t  hold_backup = s_ear.hold;

  if (half_swings < 2U) { half_swings = 2U; }

  s_ear.hold = 1U;   /* 信标期间屏蔽闭环输出，避免算法抢控制权 */

  for (n = 0U; n < half_swings; n++)
  {
    if ((n & 1U) == 0U)
    {
      Servo_RampTo(max1, max2, 60U);
    }
    else
    {
      Servo_RampTo(min1, min2, 60U);
    }
  }
  Servo_RampTo(mid1, mid2, 40U);

  s_ear.hold = hold_backup;
}

/* 供外部（比如按键/串口命令）触发一次信标 */
void Ear_Beacon(void)
{
  Ear_BeaconSwing(4U);
}

/* 固定循环延时，约 500ms（按 24MHz、每循环约 4 周期估）。
 * 故意不依赖 SystemCoreClock —— 万一 .data 段没被正确搬到 RAM，
 * 这个心跳也必须照样能跑出来。 */
/* 半周期延时：500ms。
 * 这里原来是一个 300 万次的空循环，每轮约 12~14 个周期，
 * 实际是 1.5 秒而不是 0.5 秒 —— 12 次心跳就变成 36 秒以上，
 * 后面的 PWM 扫描被整体推迟了。改成按主频计算的精确延时。 */
static void HeartbeatDelay(void)
{
  Ear_DelayMs(500U);
}

/* 最早期引脚心跳：直接写寄存器，不依赖 HAL_Init / RCC / GPIO 的任何初始化。
 * 在 main 的最开头调用（甚至可以在 HAL_Init 之前）。
 *
 * 为什么需要它：定时器输出 1500us 时的平均电压（0.247V）和"引脚悬空"
 * 读出来的感应电压几乎一样，万用表分不开。而 1Hz 方波是"稳定 3.3V ↔ 稳定 0V"，
 * 只有真正的推挽输出才做得到。 */
void Ear_PinHeartbeat(uint8_t cycles)
{
  uint32_t n;

  if (cycles == 0U) { return; }

  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  /* PA0/PA1 两路都拉成普通推挽输出，同相 1Hz。
   * 两路都驱动，不管你探针夹在哪一路，看到的都是同一条曲线。 */
  GPIOA->OTYPER &= ~((1UL << 0) | (1UL << 1));
  GPIOA->MODER   = (GPIOA->MODER & ~((3UL << (0U * 2U)) | (3UL << (1U * 2U))))
                 |  (1UL << (0U * 2U)) | (1UL << (1U * 2U));

  for (n = 0U; n < (uint32_t)cycles; n++)
  {
    GPIOA->BSRR = (1UL << 0) | (1UL << 1);              /* 两路同时高，保持 0.5s */
    HeartbeatDelay();
    GPIOA->BSRR = (1UL << 16) | (1UL << 17);            /* 两路同时低，保持 0.5s */
    HeartbeatDelay();
  }

  GPIOA->BSRR = (1UL << 16) | (1UL << 17);              /* 结束时留在低电平 */
}

/*==============================================================================
 *  纯 GPIO 软件 PWM 演示（不依赖 TIM1 / HAL / SysTick）
 *
 *  为什么需要它：现在已知"GPIO 能翻转"，但"定时器 PWM 不扫"。
 *  这里用完全相同的 GPIO 推挽输出，手工拼出 50Hz 舵机帧，
 *  把脉宽从 1300us 慢慢扫到 1700us 再扫回来。
 *
 *  判据（这一条能把问题一刀切开）：
 *    舵机跟着扫  -> PA0/PA1 到舵机这一段完全正常，问题 100% 在 TIM1 侧
 *    舵机完全不动-> 舵机供电/信号线/舵机本体的问题，与 MCU 代码无关
 *============================================================================*/

/* 微秒级忙等。24MHz 下内层循环实测约 12 个周期，所以 1us 约 2 次循环。
 * 精度受中断影响会有一两个 us 的抖动，对舵机（10us 才 1.8°）完全够用。 */
static void Ear_DelayUs(uint16_t us)
{
  volatile uint32_t i;
  uint32_t loops = (uint32_t)us * 2U;

  for (i = 0U; i < loops; i++)
  {
    __NOP();
  }
}

static void Sw_ServoFrame(uint16_t width_us)
{
  GPIOA->BSRR = (1UL << 0) | (1UL << 1);              /* 两路同时拉高 */
  Ear_DelayUs(width_us);
  GPIOA->BSRR = (1UL << 16) | (1UL << 17);            /* 两路同时拉低 */
  Ear_DelayUs((uint16_t)(20000U - width_us));          /* 补足 20ms 帧长 */

  Wdg_Kick();                                          /* 每 20ms 喂一次狗 */
}

void Ear_SwPwmDemo(void)
{
  uint8_t  rep;
  uint8_t  round;

  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  GPIOA->OTYPER &= ~((1UL << 0) | (1UL << 1));
  GPIOA->MODER   = (GPIOA->MODER & ~((3UL << (0U * 2U)) | (3UL << (1U * 2U))))
                 |  (1UL << (0U * 2U)) | (1UL << (1U * 2U));

  /* 先在中位停 0.3s，让舵机认到有效帧 */
  for (rep = 0U; rep < 15U; rep++) { Sw_ServoFrame(1500U); }

  /* 极限往复：1000us 保持 2s → 2000us 保持 2s，来回 3 次（共 12 秒）。
   * 用极限而不是渐变，是因为渐变在整个行程上分摊，慢且不明显；
   * 打满两端是"立刻转到底、停住、再转回来"，肉眼绝不会看漏，
   * 对 180 度舵机和 360 度连续旋转舵机都能看出反应。 */
  for (round = 0U; round < (uint8_t)EAR_SW_PWM_ROUNDS; round++)
  {
    for (rep = 0U; rep < 100U; rep++) { Sw_ServoFrame(1000U); }   /* 2s */
    for (rep = 0U; rep < 100U; rep++) { Sw_ServoFrame(2000U); }   /* 2s */
  }

  /* 回中 */
  for (rep = 0U; rep < 15U; rep++) { Sw_ServoFrame(1500U); }

  GPIOA->BSRR = (1UL << 16) | (1UL << 17);
}

/* HardFault：直接写寄存器，把舵机反复小幅快摆。
 * 不依赖 HAL、不依赖 SysTick、不返回 ——
 * 目的是让"CPU 死了"这件事在舵机上有明确的、无法和待机混淆的表现。
 * 幅度只取 ±150us，不会打坏齿轮。
 * 注意：计数变量用 static 而不是局部变量，这样即使异常本身是栈溢出
 * 引起的，处理函数也不会再碰栈、不会二次故障。 */
void Ear_PanicPark(void)
{
  static volatile uint32_t d;

  for (;;)
  {
    TIM1->CCR1 = 1650U;
    TIM1->CCR2 = 1650U;
    for (d = 0U; d < 3000000U; d++) { if ((d & 0x3FFU) == 0U) { Wdg_Kick(); } __NOP(); }

    TIM1->CCR1 = 1350U;
    TIM1->CCR2 = 1350U;
    for (d = 0U; d < 3000000U; d++) { if ((d & 0x3FFU) == 0U) { Wdg_Kick(); } __NOP(); }
  }
}

/* 急促抽动：表示"传感器没通"，和自检的"平滑长扫"一眼就能区分。
 * 抽动的下数就是故障码，见 app_ears.h 里的说明。
 * 每下之间留 140ms 停顿，是为了让人能数清 —— 连着抖是数不出来的。 */
void Ear_FaultIndicate(void)
{
  uint16_t mid1 = ClampUs((float)EAR_CH1_CENTER_US + (float)EAR_CH1_TRIM_US);
  uint16_t mid2 = ClampUs((float)EAR_CH2_CENTER_US + (float)EAR_CH2_TRIM_US);
  uint16_t amp1 = (uint16_t)((EAR_CH1_SPAN_US > 150U) ? 150U : (EAR_CH1_SPAN_US / 2U));
  uint16_t amp2 = (uint16_t)((EAR_CH2_SPAN_US > 150U) ? 150U : (EAR_CH2_SPAN_US / 2U));
  uint8_t  i;
  uint8_t  times = s_ear.fault_count;
  uint8_t  hold_backup = s_ear.hold;

  if (times < 2U) { times = 3U; }

  s_ear.hold = 1U;

  for (i = 0U; i < times; i++)
  {
    Servo_RampTo((uint16_t)(mid1 + amp1), (uint16_t)(mid2 - amp2), 2U);
    Servo_RampTo((uint16_t)(mid1 - amp1), (uint16_t)(mid2 + amp2), 2U);
    Ear_DelayMs(140U);          /* 下与下之间的停顿，方便数 */
  }
  Servo_RampTo(mid1, mid2, 2U);
  Ear_DelayMs(400U);            /* 收尾停顿，下一轮开始时不会连在一起 */

  s_ear.hold = hold_backup;
}
