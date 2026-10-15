#include <Arduino.h>
#include <HardwareSerial.h>

#include "Config.h"
#include "PinConfig.h"

namespace {

HardwareSerial sensorSerial(2);

constexpr uint8_t READ_PRIMARY_REQUEST[] = {
    0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
constexpr uint8_t READ_EC_SECONDARY_REQUEST[] = {
    0x01, 0x03, 0x00, 0x02, 0x00, 0x02, 0x65, 0xCB};
constexpr size_t RESPONSE_SIZE = 9;

uint16_t modbusCrc(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t position = 0; position < length; ++position) {
    crc ^= data[position];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void printBytes(const char *label, const uint8_t *data, size_t length) {
  Serial.print(label);
  for (size_t index = 0; index < length; ++index) {
    if (data[index] < 0x10) {
      Serial.print('0');
    }
    Serial.print(data[index], HEX);
    if (index + 1 < length) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

void clearSensorInput() {
  while (sensorSerial.available() > 0) {
    sensorSerial.read();
  }
}

bool transact(const uint8_t *request, uint8_t *response) {
  clearSensorInput();
  printBytes("TX: ", request, 8);
  sensorSerial.write(request, 8);
  sensorSerial.flush();

  size_t received = 0;
  const unsigned long startedAt = millis();
  while (received < RESPONSE_SIZE &&
         millis() - startedAt < SENSOR_RESPONSE_TIMEOUT_MS) {
    if (sensorSerial.available() > 0) {
      response[received++] = static_cast<uint8_t>(sensorSerial.read());
    }
  }

  if (received > 0) {
    printBytes("RX: ", response, received);
  }
  if (received != RESPONSE_SIZE) {
    Serial.print("ERROR: response timeout/length (received ");
    Serial.print(received);
    Serial.println(" of 9 bytes)");
    return false;
  }
  if (response[0] != SENSOR_MODBUS_ADDRESS || response[1] != 0x03 ||
      response[2] != 0x04) {
    Serial.println("ERROR: unexpected Modbus address, function, or byte count");
    return false;
  }

  const uint16_t calculatedCrc = modbusCrc(response, RESPONSE_SIZE - 2);
  const uint16_t receivedCrc =
      static_cast<uint16_t>(response[7]) |
      (static_cast<uint16_t>(response[8]) << 8);
  if (calculatedCrc != receivedCrc) {
    Serial.print("ERROR: CRC mismatch; received 0x");
    Serial.print(receivedCrc, HEX);
    Serial.print(", calculated 0x");
    Serial.println(calculatedCrc, HEX);
    return false;
  }

  Serial.println("CRC: OK");
  return true;
}

uint16_t unsignedRegister(const uint8_t high, const uint8_t low) {
  return (static_cast<uint16_t>(high) << 8) | low;
}

int16_t signedRegister(const uint8_t high, const uint8_t low) {
  return static_cast<int16_t>(unsignedRegister(high, low));
}

void readPrimaryMeasurement() {
  uint8_t response[RESPONSE_SIZE] = {};
  if (!transact(READ_PRIMARY_REQUEST, response)) {
    return;
  }

  const float temperature = signedRegister(response[5], response[6]) / 10.0F;
  if (SENSOR_TYPE == SensorType::PH) {
    const float ph = unsignedRegister(response[3], response[4]) / 100.0F;
    Serial.print("pH: ");
    Serial.print(ph, 2);
  } else if (SENSOR_TYPE == SensorType::ORP) {
    const int16_t orp = signedRegister(response[3], response[4]);
    Serial.print("ORP: ");
    Serial.print(orp);
    Serial.print(" mV");
  } else {
    const uint16_t ec = unsignedRegister(response[3], response[4]);
    Serial.print("EC: ");
    Serial.print(ec);
    Serial.print(" uS/cm");
  }
  Serial.print("  Temperature: ");
  Serial.print(temperature, 1);
  Serial.println(" C");
}

void readEcSecondaryMeasurements() {
  uint8_t response[RESPONSE_SIZE] = {};
  if (!transact(READ_EC_SECONDARY_REQUEST, response)) {
    return;
  }

  const uint16_t salinity = unsignedRegister(response[3], response[4]);
  const uint16_t tds = unsignedRegister(response[5], response[6]);
  Serial.print("Salinity: ");
  Serial.print(salinity);
  Serial.print(" ppm  TDS: ");
  Serial.print(tds);
  Serial.println(" ppm");
}

const char *sensorName() {
  switch (SENSOR_TYPE) {
    case SensorType::PH:
      return "SEN0708 pH";
    case SensorType::ORP:
      return "SEN0709 ORP";
    case SensorType::EC:
      return "SEN0707 EC";
  }
  return "unknown";
}

}  // namespace

void setup() {
  Serial.begin(USB_BAUD);
  delay(1000);
  sensorSerial.begin(SENSOR_BAUD, SERIAL_8N1, RS485_UART_RX_PIN,
                     RS485_UART_TX_PIN);

  Serial.println();
  Serial.println("Clearwater single-sensor RS-485 test");
  Serial.print("Sensor: ");
  Serial.println(sensorName());
  Serial.print("UART: 4800 8N1, RX GPIO ");
  Serial.print(RS485_UART_RX_PIN);
  Serial.print(", TX GPIO ");
  Serial.println(RS485_UART_TX_PIN);
}

void loop() {
  Serial.println();
  readPrimaryMeasurement();
  if (SENSOR_TYPE == SensorType::EC) {
    delay(250);
    readEcSecondaryMeasurements();
  }
  delay(SENSOR_POLL_INTERVAL_MS);
}
