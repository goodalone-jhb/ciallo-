#!/usr/bin/env python3
"""Host tests: actual firmware discovery functions with mocked bus; requires gcc and pyocd Python."""
from pathlib import Path
import re
import subprocess
import tempfile
import imu_status

ROOT = Path(__file__).resolve().parents[1]
src = (ROOT / 'Core/SRC/app_ears.c').read_text(encoding='utf-8')

def function(name):
    m = re.search(r'^static [^;\n]*\b' + name + r'\([^;]*?\)\s*\{', src, re.M)
    assert m, name
    end = m.end()
    depth = 1
    while depth:
        depth += (src[end] == '{') - (src[end] == '}')
        end += 1
    return src[m.start():end]

ctx = re.search(r'typedef struct\s*\{[^}]*\} EarCtx_t;', src).group(0)
diag = re.search(r'typedef struct\s*\{[^}]*\} EarImuDiag_t;', src).group(0)
header = r'''
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#define IMU_SCAN_ADDR_MIN 0x08U
#define IMU_SCAN_ADDR_MAX 0x77U
#define IMU_ADDR_PRIMARY 0x68U
#define IMU_ADDR_SECONDARY 0x69U
#define MPU_REG_WHO_AM_I 0x75U
'''
mocks = r'''
static EarCtx_t s_ear;
static volatile EarImuDiag_t g_ear_imu_diag[2];
static uint8_t s_bus;
static uint8_t devices[128], ids[128], read_ok[128], probes[128];
static int scl=1,sda=1,recover_ok=0,config_calls=0,last_stuck=0;
#define SCL_LEVEL() scl
#define SDA_LEVEL() sda
static void Wdg_Kick(void) {}
static void Bus_Recover(void) {if(recover_ok) scl=sda=1;}
static uint8_t Bus_Probe(uint8_t a) {assert(a>=8 && a<=0x77);probes[a]++;if(last_stuck && a==0x77)sda=0;return devices[a];}
static uint8_t Bus_Read(uint8_t a,uint8_t reg,uint8_t *out,uint16_t n) {assert(reg==0x75 && n==1);*out=ids[a];return read_ok[a];}
static void Bus_SetMode(uint8_t bus) {s_bus=bus;}
static uint8_t IMU_Config(void) {config_calls++;return 1;}
static void IMU_SelectReadMode(void) {}
static uint8_t IMU_Calibrate(void) {return 1;}
static void reset(void) {memset(&s_ear,0,sizeof(s_ear));memset((void*)g_ear_imu_diag,0,sizeof(g_ear_imu_diag));memset(devices,0,128);memset(ids,0,128);memset(read_ok,1,128);memset(probes,0,128);s_bus=0;scl=sda=1;config_calls=last_stuck=recover_ok=0;}
static void add(uint8_t a,uint8_t id) {devices[a]=1;ids[a]=id;}
static void complete(void) {assert(g_ear_imu_diag[s_bus].scan_count==112);assert(g_ear_imu_diag[s_bus].scan_state==2);for(int a=0;a<128;a++)assert(probes[a]==(a>=8 && a<=0x77));}
'''
tests = r'''
int main(void) {
 reset();assert(IMU_Scan()==0 && s_ear.fault_count==3);complete();
 reset();add(0x28,0x70);assert(IMU_BringUpOnBus(0)==1 && s_ear.imu_addr==0x28 && config_calls==1);complete();
 reset();add(0x6a,0x98);assert(IMU_Scan()==1 && s_ear.imu_addr==0x6a);complete();
 reset();add(0x08,0x70);add(0x77,0x71);assert(IMU_Scan()==1);assert(g_ear_imu_diag[0].found_count==2);assert((g_ear_imu_diag[0].ack_map[0x77>>3]&0x80)!=0);complete();
 reset();add(0x20,0x70);add(0x69,0x71);add(0x68,0x70);assert(IMU_Scan()==1 && s_ear.imu_addr==0x68);complete();
 reset();add(0x20,0x70);add(0x69,0x71);assert(IMU_Scan()==1 && s_ear.imu_addr==0x69);complete();
 reset();add(0x10,0x42);add(0x6a,0x70);assert(IMU_Scan()==1 && s_ear.imu_addr==0x6a);complete();
 reset();add(0x6b,0);assert(IMU_BringUpOnBus(0)==0 && s_ear.fault_count==5 && config_calls==0);assert(g_ear_imu_diag[0].found_count==1);complete();
 reset();add(0x6b,0xff);assert(IMU_BringUpOnBus(0)==0 && s_ear.fault_count==5 && config_calls==0);complete();
 reset();add(0x6b,0);read_ok[0x6b]=0;assert(IMU_BringUpOnBus(1)==0 && s_ear.fault_count==5 && config_calls==0);assert(g_ear_imu_diag[1].id75_ok[0x6b>>3]==0);complete();
 reset();sda=0;assert(IMU_Scan()==0 && g_ear_imu_diag[0].scan_state==3 && g_ear_imu_diag[0].found_count==0);
 reset();scl=0;recover_ok=1;assert(IMU_Scan()==0);complete();
 reset();last_stuck=1;assert(IMU_Scan()==0 && g_ear_imu_diag[0].scan_state==3);
 assert(sizeof(EarImuDiag_t)==216);
 puts("PASS: 13 C discovery cases, all 112 addresses, unknown-ID write prevention");return 0;
}
'''
code = header + ctx + diag + mocks + '\n'.join(function(n) for n in
    ['IMU_WhoamiValid', 'IMU_Scan', 'IMU_BringUpOnBus']) + tests
with tempfile.TemporaryDirectory(prefix='imu_scan_test_') as temp:
    c = Path(temp) / 'test.c'
    exe = Path(temp) / 'test.exe'
    c.write_text(code, encoding='utf-8')
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(c), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

assert imu_status.LAYOUT.size == 216
ack = bytearray(16)
valid = bytearray(16)
ids = bytearray(128)
for a in (0x08, 0x28, 0x77):
    ack[a >> 3] |= 1 << (a & 7)
valid[0x28 >> 3] |= 1 << (0x28 & 7)
ids[0x28] = 0x70
values = (b'EIMU', 7, 123, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 5, 0x28,
          0x70, 0x75, 1, 0, bytes(14), bytes(ack), bytes(ids), bytes(valid), 112, 3, 2, 0)
data = imu_status.LAYOUT.pack(*values) + bytes(216)
records = imu_status.decode(data)
assert [d['address'] for d in records[0]['devices']] == ['0x08', '0x28', '0x77']
assert records[0]['devices'][1]['id75'] == '0x70'
assert records[0]['devices'][0]['id75'] is None
assert not records[1]['initialized']
try:
    imu_status.decode(data[:104])
except ValueError:
    pass
else:
    raise AssertionError('Old record layout accepted')
print('PASS: full-scan decoder, unknown identity representation, old-layout rejection')

