#!/usr/bin/env python3
"""Run the production TX loop with inert host I/O, without flashing hardware.

Tests the actual packet writer for 200 scheduler cycles, including invalid Hall
and steering sentinels, ESTOP flags, CRC and the absence of battery-side writes.
Peripheral I/O and sensor inputs are mocked; this is not an RC/failsafe HIL test.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def definition(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == '{':
            depth += 1
        elif source[index] == '}':
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise ValueError(f'Unclosed definition: {signature}')


def main():
    source = (ROOT / 'src/pi_comms.cpp').read_text()
    stubs = r'''
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <vector>
#include <array>
using TickType_t = uint32_t;
constexpr int portTICK_PERIOD_MS = 1;
constexpr TickType_t pdMS_TO_TICKS(int value) { return value; }
struct PiCommsConfig { TickType_t txTaskPeriod; int uartNum; };
struct HallSpeedSnapshot {};
enum class SystemDiagTaskId { kPiUartTx };
constexpr uint8_t kHeaderTx = 0x55;
constexpr size_t kTxFrameSize = 8;
bool g_initialized = true;
uint32_t ticks = 0;
std::vector<std::array<uint8_t, 8>> frames;
TickType_t xTaskGetTickCount() { return ticks; }
int64_t esp_timer_get_time() { return ticks * 1000; }
uint32_t micros() { return ticks * 1000; }
void broadcastIf(bool, const char*) {}
void vTaskDelete(void*) {}
bool hallSpeedGetSnapshot(HallSpeedSnapshot&) { return frames.size() % 2 == 0; }
uint8_t encodeStatusFlags() { return frames.size() % 2 == 0 ? 0x21 : 0x07; }
uint16_t encodeSpeedTelemetryFromHall(const HallSpeedSnapshot&, bool ok) {
    return ok ? 123 : 0xFFFF;
}
int16_t encodeSteerTelemetryCentered() {
    return frames.size() % 2 == 0 ? -123 : INT16_MIN;
}
uint8_t encodeAppliedBrakePercent() { return frames.size() % 2 == 0 ? 20 : 100; }
void recordHallTelemetryCapture(const HallSpeedSnapshot&, bool, uint32_t, uint32_t, uint16_t) {}
void logHallTelemetryTrace(const HallSpeedSnapshot&, bool, uint32_t, uint32_t, uint16_t) {}
void logTxFrame(const PiCommsConfig&, uint8_t, uint16_t, int16_t, uint8_t, TickType_t) {}
void systemDiagReportLoop(SystemDiagTaskId, uint32_t, uint32_t, bool, bool) {}
int uart_write_bytes(int, const char* data, size_t size) {
    assert(size == 8);
    std::array<uint8_t, 8> frame{};
    for (size_t i = 0; i < 8; ++i) frame[i] = static_cast<uint8_t>(data[i]);
    frames.push_back(frame);
    return size;
}
struct Done {};
void vTaskDelayUntil(TickType_t* wake, TickType_t period) {
    assert(period == 10);
    *wake += period;
    ticks = *wake;
    if (ticks == 2000) throw Done{};
}
'''
    check = r'''
int main() {
    PiCommsConfig cfg{10, 0};
    try { taskPiCommsTx(&cfg); } catch (const Done&) {}
    assert(frames.size() == 200);
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& f = frames[i];
        assert(f[0] == 0x55);
        if (i % 2 == 0) {
            assert(f[1] == 0x21 && f[2] == 123 && f[3] == 0);
            assert(f[4] == 0x85 && f[5] == 0xFF && f[6] == 20);
        } else {
            assert(f[1] == 0x07 && f[2] == 0xFF && f[3] == 0xFF);
            assert(f[4] == 0 && f[5] == 0x80 && f[6] == 100);
        }
        // Independent polynomial division, preserving the existing non-reflected 0x31 wire CRC.
        uint64_t remainder = 0;
        for (size_t byte = 0; byte < 7; ++byte)
            remainder = (remainder << 8) | f[byte];
        remainder <<= 8;
        for (int bit = 63; bit >= 8; --bit)
            if (remainder & (uint64_t{1} << bit))
                remainder ^= uint64_t{0x131} << (bit - 8);
        assert(f[7] == remainder);
    }
}
'''
    with tempfile.TemporaryDirectory(prefix='esp32-uart-regression-') as directory:
        path = Path(directory)
        (path / 'check.cpp').write_text(stubs + '\n' + definition(source, 'uint8_t crc8_maxim(')
                                      + '\n' + definition(source, 'void taskPiCommsTx(')
                                      + '\n' + check)
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        str(path / 'check.cpp'), '-o', str(path / 'check')], check=True)
        subprocess.run([str(path / 'check')], check=True)
    print('PASS: 200 TX cycles; only 0x55 frames, valid CRC, preserved sentinels and status bytes')


if __name__ == '__main__':
    main()
