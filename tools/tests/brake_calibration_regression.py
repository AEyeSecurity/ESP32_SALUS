#!/usr/bin/env python3
"""Exercise production brake calibration functions with inert servo I/O."""
from pathlib import Path
import subprocess
import tempfile
from uart_tx_no_battery_regression import definition
ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'src/quad_functions.cpp').read_text()
stubs = r'''
#include <cassert>
#include <cstdint>
struct BrakeServoAngles { int servoA; int servoB; };
struct Config { int releaseAngleServoADeg=25, brakeAngleServoADeg=110;
 int releaseAngleServoBDeg=130, brakeAngleServoBDeg=80; } g_brakeConfig;
bool g_brakeInitialized=true, g_brakeCalibrationEnabled=false;
BrakeServoAngles g_brakeCurrentAngles{25,130}, g_brakeCalibrationAngles{0,0};
int g_brakeMux=0, lastPercent=-1;
void portENTER_CRITICAL(int*) {}
void portEXIT_CRITICAL(int*) {}
void applyBrakeAngles(int a,int b) { g_brakeCurrentAngles={a,b}; }
void quadBrakeApplyPercent(uint8_t p) {
 lastPercent=p;
 applyBrakeAngles(25+85*p/100,130-50*p/100);
}
'''
signatures = ['bool quadBrakeSetCalibrationAngle(', 'void quadBrakeEndCalibration(',
              'bool quadBrakeApplyControl(', 'bool quadBrakeSetServoEndpoint(']
checks = r'''
int main() {
 assert(!quadBrakeApplyControl(0));
 assert(quadBrakeSetCalibrationAngle(false,40));
 assert(quadBrakeApplyControl(0));
 assert(g_brakeCurrentAngles.servoA==40 && g_brakeCurrentAngles.servoB==130);
 assert(quadBrakeSetCalibrationAngle(true,90));
 assert(quadBrakeApplyControl(0));
 assert(g_brakeCurrentAngles.servoA==40 && g_brakeCurrentAngles.servoB==90);
 assert(!quadBrakeSetCalibrationAngle(true,181));
 assert(!quadBrakeSetCalibrationAngle(false,-1));
 // Every ordinary braking demand wins over calibration, including E-stop 100%.
 for (int p=1;p<=100;++p) { assert(quadBrakeApplyControl(p)); assert(lastPercent==p); }
 assert(g_brakeCurrentAngles.servoA==110 && g_brakeCurrentAngles.servoB==80);
 assert(quadBrakeApplyControl(0));
 assert(g_brakeCurrentAngles.servoA==40 && g_brakeCurrentAngles.servoB==90);
 assert(quadBrakeSetServoEndpoint(false,false,30));
 assert(g_brakeConfig.releaseAngleServoADeg==30 && g_brakeConfig.releaseAngleServoBDeg==130);
 assert(quadBrakeSetServoEndpoint(true,true,70));
 assert(g_brakeConfig.brakeAngleServoADeg==110 && g_brakeConfig.brakeAngleServoBDeg==70);
 assert(!quadBrakeSetServoEndpoint(true,true,181));
 assert(g_brakeConfig.brakeAngleServoBDeg==70);
 quadBrakeEndCalibration(); assert(!quadBrakeApplyControl(0));
 assert(g_brakeCurrentAngles.servoA==25 && g_brakeCurrentAngles.servoB==130);
 g_brakeInitialized=false;
 assert(!quadBrakeSetCalibrationAngle(false,30));
 assert(!quadBrakeSetServoEndpoint(false,false,30));
}
'''
with tempfile.TemporaryDirectory() as directory:
 p=Path(directory); cpp=p/'test.cpp'; exe=p/'test'
 cpp.write_text(stubs+'\n'.join(definition(source,s) for s in signatures)+checks)
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra',str(cpp),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
print('Brake calibration regression PASS (inert I/O; no hardware)')
