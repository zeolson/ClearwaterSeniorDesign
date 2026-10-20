#include <Arduino.h>
#include <HardwareSerial.h>

#include "Config.h"
#include "PinConfig.h"

namespace {

HardwareSerial sensorSerial(2);

constexpr size_t REQUEST_SIZE = 8;
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

void buildReadRequest(uint8_t address, uint16_t firstRegister,
                      uint16_t registerCount, uint8_t *request) {
  request[0] = address;
  request[1] = 0x03;
  request[2] = static_cast<uint8_t>(firstRegister >> 8);
  request[3] = static_cast<uint8_t>(firstRegister);
  request[4] = static_cast<uint8_t>(registerCount >> 8);
  request[5] = static_cast<uint8_t>(registerCount);
  const uint16_t crc = modbusCrc(request, REQUEST_SIZE - 2);
  request[6] = static_cast<uint8_t>(crc);
  request[7] = static_cast<uint8_t>(crc >> 8);
}

bool transact(const uint8_t *request, uint8_t expectedAddress,
              uint8_t *response) {
  clearSensorInput();
  printBytes("TX: ", request, REQUEST_SIZE);
  sensorSerial.write(request, REQUEST_SIZE);
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
  if (response[0] != expectedAddress || response[1] != 0x03 ||
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

struct Measurement {
  bool phValid = false;
  bool orpValid = false;
  bool ecPrimaryValid = false;
  bool ecSecondaryValid = false;
  bool depthValid = false;
  float ph = 0.0F;
  float phTemperature = 0.0F;
  int16_t orp = 0;
  float orpTemperature = 0.0F;
  uint16_t ec = 0;
  float ecTemperature = 0.0F;
  uint16_t salinity = 0;
  uint16_t tds = 0;
  float depthMeters = 0.0F;
};

bool readRegisters(uint8_t address, uint16_t firstRegister,
                   uint8_t *response) {
  uint8_t request[REQUEST_SIZE] = {};
  buildReadRequest(address, firstRegister, 2, request);
  return transact(request, address, response);
}

void readPh(Measurement &measurement) {
  Serial.print("Polling pH sensor at address ");
  Serial.println(PH_MODBUS_ADDRESS);
  uint8_t response[RESPONSE_SIZE] = {};
  if (!readRegisters(PH_MODBUS_ADDRESS, 0x0000, response)) {
    return;
  }

  measurement.ph = unsignedRegister(response[3], response[4]) / 100.0F;
  measurement.phTemperature =
      signedRegister(response[5], response[6]) / 10.0F;
  measurement.phValid = true;
  Serial.print("pH: ");
  Serial.print(measurement.ph, 2);
  Serial.print("  Temperature: ");
  Serial.print(measurement.phTemperature, 1);
  Serial.println(" C");
}

void readOrp(Measurement &measurement) {
  Serial.print("Polling ORP sensor at address ");
  Serial.println(ORP_MODBUS_ADDRESS);
  uint8_t response[RESPONSE_SIZE] = {};
  if (!readRegisters(ORP_MODBUS_ADDRESS, 0x0000, response)) {
    return;
  }

  measurement.orp = signedRegister(response[3], response[4]);
  measurement.orpTemperature =
      signedRegister(response[5], response[6]) / 10.0F;
  measurement.orpValid = true;
  Serial.print("ORP: ");
  Serial.print(measurement.orp);
  Serial.print(" mV  Temperature: ");
  Serial.print(measurement.orpTemperature, 1);
  Serial.println(" C");
}

void readEc(Measurement &measurement) {
  Serial.print("Polling EC sensor at address ");
  Serial.println(EC_MODBUS_ADDRESS);
  uint8_t response[RESPONSE_SIZE] = {};
  if (readRegisters(EC_MODBUS_ADDRESS, 0x0000, response)) {
    measurement.ec = unsignedRegister(response[3], response[4]);
    measurement.ecTemperature =
        signedRegister(response[5], response[6]) / 10.0F;
    measurement.ecPrimaryValid = true;
    Serial.print("EC: ");
    Serial.print(measurement.ec);
    Serial.print(" uS/cm  Temperature: ");
    Serial.print(measurement.ecTemperature, 1);
    Serial.println(" C");
  }

  delay(INTER_SENSOR_DELAY_MS);
  if (!readRegisters(EC_MODBUS_ADDRESS, 0x0002, response)) {
    return;
  }
  measurement.salinity = unsignedRegister(response[3], response[4]);
  measurement.tds = unsignedRegister(response[5], response[6]);
  measurement.ecSecondaryValid = true;
  Serial.print("Salinity: ");
  Serial.print(measurement.salinity);
  Serial.print(" ppm  TDS: ");
  Serial.print(measurement.tds);
  Serial.println(" ppm");
}

void printCsvRecord(const Measurement &measurement) {
  // The host monitor filter recognizes DATA records and adds wall-clock time.
  // Fields: uptime,pH,pH temp,ORP,ORP temp,EC,EC temp,salinity,TDS,
  // depth,pH status,ORP status,EC status
  Serial.print("DATA,");
  Serial.print(millis());
  Serial.print(',');
  if (measurement.phValid) {
    Serial.print(measurement.ph, 2);
  }
  Serial.print(',');
  if (measurement.phValid) {
    Serial.print(measurement.phTemperature, 1);
  }
  Serial.print(',');
  if (measurement.orpValid) {
    Serial.print(measurement.orp);
  }
  Serial.print(',');
  if (measurement.orpValid) {
    Serial.print(measurement.orpTemperature, 1);
  }
  Serial.print(',');
  if (measurement.ecPrimaryValid) {
    Serial.print(measurement.ec);
  }
  Serial.print(',');
  if (measurement.ecPrimaryValid) {
    Serial.print(measurement.ecTemperature, 1);
  }
  Serial.print(',');
  if (measurement.ecSecondaryValid) {
    Serial.print(measurement.salinity);
  }
  Serial.print(',');
  if (measurement.ecSecondaryValid) {
    Serial.print(measurement.tds);
  }
  Serial.print(',');
  if (measurement.depthValid) {
    Serial.print(measurement.depthMeters, 3);
  }
  Serial.print(',');
  Serial.print(measurement.phValid ? "ok" : "error");
  Serial.print(',');
  Serial.print(measurement.orpValid ? "ok" : "error");
  Serial.print(',');
  if (!measurement.ecPrimaryValid) {
    Serial.println("primary_error");
  } else if (!measurement.ecSecondaryValid) {
    Serial.println("secondary_error");
  } else {
    Serial.println("ok");
  }
}

}  // namespace

void setup() {
  Serial.begin(USB_BAUD);
  delay(1000);
  sensorSerial.begin(SENSOR_BAUD, SERIAL_8N1, RS485_UART_RX_PIN,
                     RS485_UART_TX_PIN);

  Serial.println();
  Serial.println("Clearwater three-sensor RS-485 multidrop test");
  Serial.print("Addresses: pH=");
  Serial.print(PH_MODBUS_ADDRESS);
  Serial.print(", ORP=");
  Serial.print(ORP_MODBUS_ADDRESS);
  Serial.print(", EC=");
  Serial.println(EC_MODBUS_ADDRESS);
  Serial.print("UART: 4800 8N1, RX GPIO ");
  Serial.print(RS485_UART_RX_PIN);
  Serial.print(", TX GPIO ");
  Serial.println(RS485_UART_TX_PIN);
}

void loop() {
  Serial.println();
  Measurement measurement;
  readPh(measurement);
  delay(INTER_SENSOR_DELAY_MS);
  readOrp(measurement);
  delay(INTER_SENSOR_DELAY_MS);
  readEc(measurement);
  printCsvRecord(measurement);
  delay(SENSOR_POLL_INTERVAL_MS);
}
