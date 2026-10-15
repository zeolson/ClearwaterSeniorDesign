#pragma once

enum class SensorType {
  PH,
  ORP,
  EC,
};

// Change only this value when swapping the single sensor on the test bench.
constexpr SensorType SENSOR_TYPE = SensorType::PH;

constexpr unsigned long USB_BAUD = 115200;
constexpr unsigned long SENSOR_BAUD = 4800;
constexpr unsigned long SENSOR_RESPONSE_TIMEOUT_MS = 1000;
constexpr unsigned long SENSOR_POLL_INTERVAL_MS = 1000;
constexpr uint8_t SENSOR_MODBUS_ADDRESS = 0x01;
