#pragma once

// Classic ESP32 T-Beam test connection. These pins avoid the onboard GPS,
// LoRa radio, I2C/PMU, and USB programming serial port.
constexpr int RS485_UART_RX_PIN = 13;  // Connect to converter TXD.
constexpr int RS485_UART_TX_PIN = 14;  // Connect to converter RXD.
