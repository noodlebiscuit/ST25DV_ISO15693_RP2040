#include <Arduino.h>
#include <ArduinoBLE.h>
#include <mbed.h>
#include <rtos.h>
#include <Wire.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <iomanip>

#include "ST25DV64KC//ST25DV64KC_Arduino_Library.h"
#include "CRC32/SerialBuffer.h"
#include "CRC32/CRC32.h"

#pragma once
#include <stdint.h>

//------------------------------------------------------------------------------------------------

using namespace mbed;
using namespace std::chrono;

//------------------------------------------------------------------------------------------------

#pragma region BLUETOOTH LOW ENERGY SUPPORT
// BLE service descriptors
#define UUID_SERVICE_NORDIC_SPP "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"         // UUID for NORDIC SPP UART service
#define UUID_SERVICE_BATTERY "0000180F-0000-1000-8000-00805F9B34Fb"            // UUID for the battery service
#define UUID_SERVICE_DEVICE_INFORMATION "0000180A-0000-1000-8000-00805F9B34Fb" // UUID for the device information service

// BLE service characteristics
#define UUID_CHARACTERISTIC_SPP_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // NORDIC SPP UART receive data
#define UUID_CHARACTERISTIC_SPP_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // NORDIC SSP UART transmit data
// ------------------------------------------------------------------------------------------------
#define UUID_CHARACTERISTIC_BATTERY "00002A19-0000-1000-8000-00805f9b34fb" // battery level characteristic
// ------------------------------------------------------------------------------------------------
#define UUID_CHARACTERISTIC_MODEL "00002A24-0000-1000-8000-00805f9b34fb"        // model number characteristic
#define UUID_CHARACTERISTIC_SERIAL "00002A25-0000-1000-8000-00805f9b34fb"       // serial number characteristic
#define UUID_CHARACTERISTIC_FIRMWARE "00002A26-0000-1000-8000-00805f9b34fb"     // firmware revision characteristic
#define UUID_CHARACTERISTIC_HARDWARE "00002A27-0000-1000-8000-00805f9b34fb"     // hardware revision characteristic
#define UUID_CHARACTERISTIC_MANUFACTURER "00002A29-0000-1000-8000-00805f9b34fb" // manufacturers name for device information service

// device characteristics
#define LOCAL_NAME_OF_PERIPHERAL "CMWR24 NFC Emulator"
#define PERIPERHAL_DEVICE_NAME "CMWR24 NFC Emulator"
#define MANUFACTURER_NAME_STRING "SKF (UK) Ltd"
#define MODEL_NAME_STRING "SKF Insight Rail"
#define HARDWARE_NAME_STRING "Emulator by SKF (UK)"
#define FIRMWARE_NAME_STRING "20230725-BINARY"
#define SERIAL_NO_NAME_STRING "CMWR24-NFC-EMU-001D"

// set the manufacturer code to 'SKF (U.K.) Limited'
const uint8_t SKF_MANUFACTURER_CODE[2] = {0x0e, 0x04};

// Setup the incoming data characteristic (RX).
#define RX_BUFFER_SIZE 32
#define RX_BUFFER_FIXED_LENGTH false

// Setup the outgoinging data characteristic (TX).
#define TX_BUFFER_SIZE 32
#define TX_BUFFER_FIXED_LENGTH false

#define BLE_ATTRIBUTE_MAX_VALUE_LENGTH 512

// Buffer to read samples into, each sample is 16-bits
uint8_t configBuffer[RX_BUFFER_SIZE];

// add each of the core services
BLEService nearFieldService(UUID_SERVICE_NORDIC_SPP);
BLEService deviceInfoService(UUID_SERVICE_DEVICE_INFORMATION);
BLEService batteryService(UUID_SERVICE_BATTERY);

// RX / TX Characteristics for BYTE ARRAYS
BLECharacteristic rxChar(UUID_CHARACTERISTIC_SPP_RX, BLEWriteWithoutResponse | BLEWrite, RX_BUFFER_SIZE, RX_BUFFER_FIXED_LENGTH);
BLECharacteristic txChar(UUID_CHARACTERISTIC_SPP_TX, BLERead | BLENotify, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);

// battery characteristics
BLECharacteristic batteryCharacteristic(UUID_CHARACTERISTIC_BATTERY, BLERead | BLENotify, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);

// device characteristics
BLECharacteristic manufacturerCharacteristic(UUID_CHARACTERISTIC_MANUFACTURER, BLERead, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);
BLECharacteristic firmwareRevisionCharacteristic(UUID_CHARACTERISTIC_FIRMWARE, BLERead, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);
BLECharacteristic modelNumberCharacteristic(UUID_CHARACTERISTIC_MODEL, BLERead, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);
BLECharacteristic hardwareCharacteristic(UUID_CHARACTERISTIC_HARDWARE, BLERead, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);
BLECharacteristic serialNumberCharacteristic(UUID_CHARACTERISTIC_SERIAL, BLERead, TX_BUFFER_SIZE, TX_BUFFER_FIXED_LENGTH);
#pragma endregion

//------------------------------------------------------------------------------------------------

#define TICK_RATE_MS 1000ms // update rate for the mbed timer

//------------------------------------------------------------------------------------------------

#define SPI_SCK (13)              // SPI pin SCLK
#define SPI_MISO (12)             // SPI pin MISO
#define SPI_MOSI (11)             // SPI pin MOSI
#define SPI_SS (10)               // SPI pin SS
#define GPIO_PIN_2 2              // ATTENTION pin from the NoteCarrier
#define GPIO_PIN_3 3              // Acknowledge button pin (resets the LED and display)
#define GPIO_PIN_4 4              // GPIO pin
#define GPIO_PIN_5 5              // GPIO pin
#define GPIO_PIN_6 6              // GPIO pin
#define GPIO_PIN_7 7              // GPIO pin
#define GPIO_PIN_8 8              // GPIO pin
#define GPIO_PIN_9 9              // GPIO pin
#define SERIAL_BAUD_RATE 115200   // serial port baud rate
#define SERIAL_BUFFER_SIZE 400    // how many bytes are in thes serial buffer
#define SERIAL_USB Serial         // USB serial port (for non-debug use)
#define ISO15693_USER_MEMORY 256  // NFC tag memory
#define RECEIVE_BUFFER_LENGTH 128 // how many bytes in the receive buffer
//------------------------------------------------------------------------------------------------

/// @brief UNCOMMENT THIS LINE TO ALLOW SUPPORT FOR NORDIC SPP UART
#define NORDIC_SPP_FUNCTIONALITY

/// @brief  when set TRUE, hex strings are expressed in UPPER CASE
#define HEX_UPPER_CASE true

/// @brief  when set TRUE, when SCAN output is to be ASCII (binary) instead of HEX based text
#define SET_OUTPUT_AS_BINARY true

//------------------------------------------------------------------------------------------------

#define SDA_PIN 18
#define SCL_PIN 19
#define WireNFC MyWire

//------------------------------------------------------------------------------------------------

#pragma region PRIVATE MEMBERS

/// @brief detail which pins are allocated to I2C
TwoWire MyWire(digitalPinToPinName(SDA_PIN), digitalPinToPinName(SCL_PIN));

/// @brief  MBED* control the BLE connected pin
DigitalOut LED_SetConnectedToBLE(digitalPinToPinName(GPIO_PIN_4));

/// @brief  Set direct write access to GPIO PIN 3 here
InterruptIn OnTagDetectedInterrupt(digitalPinToPinName(GPIO_PIN_9));

/// @brief  when set true, we need to block all other I/O activites
volatile bool _readerBusy = false;

/// @brief MBED RTOS timer
Ticker timer;

/// @brief managed serial receive buffer (non-rotating!)
SerialBuffer<RECEIVE_BUFFER_LENGTH> _SerialBuffer;

#pragma endregion

//------------------------------------------------------------------------------------------------

#pragma region METHOD PROTOTYPES

void main_thread();
void bluetooth_thread();
void publish_tag();
void TagDetectedInterrupt();
void AtTime();

size_t WriteToSPP(uint8_t);
static void onBLEWritten(BLEDevice, BLECharacteristic);
void onBLEConnected(BLEDevice);
void onBLEDisconnected(BLEDevice);
void onRxCharValueUpdate(BLEDevice, BLECharacteristic);

void SetupBLE();
void StartBLE();

void AddBatteryServiceBLE();
void AddDataServiceBLE();
void AddDeviceServiceBLE();
void PublishBattery();
void PublishBinaryPayloadToBluetooth(uint8_t *, uint8_t *);
void PublishBinaryUIDToBluetooth(uint8_t *);
void PublishHexUIDToBluetooth(uint8_t *);
void PublishHardwareDetails();
void PublishHexPayloadToBluetooth(uint8_t *, uint8_t *);
void PublishResponseToBluetooth(char *, size_t);

#pragma endregion