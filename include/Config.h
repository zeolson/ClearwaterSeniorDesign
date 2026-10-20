#pragma once

constexpr unsigned long USB_BAUD = 115200;
constexpr unsigned long SENSOR_BAUD = 4800;
constexpr unsigned long SENSOR_RESPONSE_TIMEOUT_MS = 1000;
constexpr unsigned long SENSOR_POLL_INTERVAL_MS = 1000;
constexpr unsigned long INTER_SENSOR_DELAY_MS = 250;

// Program the three sensors individually before placing them on the same bus.
constexpr uint8_t PH_MODBUS_ADDRESS = 0x01;
constexpr uint8_t ORP_MODBUS_ADDRESS = 0x02;
constexpr uint8_t EC_MODBUS_ADDRESS = 0x03;
