#pragma once

// Classic ESP32 T-Beam test connection. These pins avoid the onboard GPS,
// LoRa radio, I2C/PMU, and USB programming serial port.
// The HiLetgo automatic-flow-control module labels its UART pins from the
// controller's perspective, so connect RX-to-RXD and TX-to-TXD (no crossover).
constexpr int RS485_UART_RX_PIN = 13;  // Connect to converter RXD.
constexpr int RS485_UART_TX_PIN = 14;  // Connect to converter TXD.
