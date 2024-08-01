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
#include "CMWR23.h"

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

// DEBUG CONTROLLERS - REMOVE COMMENT BLOCKS TO ENABLE OUTPUT OVER SERIAL

#define READER_DEBUG
// #define READER_DEBUG_APPEND_FUNCTIONALITY
#define SERIAL_RECEIVE_DEBUG
#define READER_BROADCAST_DEBUG
#define READER_DEBUG_PRINT Serial

//------------------------------------------------------------------------------------------------

#define HEADER_BYTES 19          // how many bytes make up the complete SCOMP PROTOCOL header
#define QUERY_HEADER_BYTES 10    // how many bytes in a QUERY payload form the SCOMP PROTOCOL header
#define RESPONSE_HEADER_BYTES 10 // how many bytes in a RESPONSE payload form the SCOMP PROTOCOL header
#define RFID_RESPONSE_BYTES 9    // how many bytes in the SCOMP RFID response data header
#define FOOTER_BYTES 4           // how many bytes make up the CRC32 block
#define LENGTH_BYTES 2           // how many bytes make up the CRC32 block
#define QUERY_OFFSET_BYTES 4     // how many bytes should we skip before we hit the QUERY (Q) char
#define CRC32_CHARACTERS 8       // how many ASCII HEX characters are in a CRC32
#define RECEIVE_BUFFER_LENGTH 48 // maximum number of command bytes we can accept
#define BLOCK_SIZE_BLE 16        // block size in bytes
#define BLOCK_WAIT_BLE 50000     // wait 50ms between each BLE transmit packet
#define SENSOR_STARTUP_TIME 120  // time in seconds for the CMWR32 sensor to start
#define SENSOR_SHUTDOWN_TIME 30  // time in seconds for the CMWR32 sensor to shutdown

//------------------------------------------------------------------------------------------------

#define SDA_PIN 18
#define SCL_PIN 19
#define WireNFC MyWire

//------------------------------------------------------------------------------------------------

#pragma region PRIVATE MEMBERS
/// @brief  > RECORD HEADER
/// @brief    These ten bytes describe both the data type as well as the total number of bytes
uint8_t PAYLOAD_LEGTH[LENGTH_BYTES] = {0x00, 0x00};

/// @brief  > END OF RECORD four byte CRC32
uint8_t EOR[FOOTER_BYTES] = {0x00, 0x00, 0x00, 0x00};

/// @brief commissioned command written to sensor
byte COMMAND_CMSD[] = {0x63, 0x6d, 0x64, 0x3a, 0x63, 0x6d, 0x73, 0x64};

/// @brief ship command written to sensor
byte COMMAND_SHIP[] = {0x63, 0x6d, 0x64, 0x3a, 0x73, 0x68, 0x69, 0x70};

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

/// @brief  calculate the CRC value of any byte or character stream
CRC32 crc;

/// @brief managed serial receive buffer (non-rotating!)
SerialBuffer<RECEIVE_BUFFER_LENGTH> _SerialBuffer;

/// @brief  > BASIC CARRIAGE RETURN \ LINE FEED
uint8_t CR_LF[2] = {0x0d, 0x0a};

/// @brief create the default SCANNDY PROTOCOL header for returning NFC payload data
char scomp_rfid_response_header[] = "0000R0000#rfiddata:";

/// @brief create the default SCANNDY PROTOCOL header for returning an OK response
char scomp_ok_response_header[] = "0000R0000#";

/// @brief scomp query and response message identifier represented as a four character long string
char scomp_query_ID[] = "0000";

/// @brief scomp default response to a successfully received and processed query
char scomp_response_ok[] = "ok";

/// @brief scomp default response to a invalid processed query (E.g. wrong CRC32 value)
char scomp_response_error[] = "error";

/// @brief scomp default response to a sensor being enabled
char scomp_response_sensor_enabled[] = "sensor enabled";

/// @brief scomp default response to a sensor being disabled
char scomp_response_sensor_disabled[] = "sensor disabled";

/// @brief sensor is commissioned
const char *CMSD = "cmsd";

/// @brief sensor is disabled and can be transported
const char *SHIP = "ship";

/// @brief has the reader received an SCANNDY SCOMP query?
volatile bool _queryReceived = false;

/// @brief has the reader received an SCANNDY SCOMP query?
volatile bool _invalidQueryReceived = false;

/// @brief SCANNDY SCOMP message identifier as a 16 bit unsigned integer
uint16_t _messageIdentifier = 0x0000;

/// @brief cmwr23 sensor object
CMWR23 sensor;
#pragma endregion

//------------------------------------------------------------------------------------------------

const size_t CMWR_PARAMETER_COUNT = 12;

const char IMEI[] = _IMEI;
const char MODL[] = _MODL;
const char MFDT[] = _MFDT;
const char HWVN[] = _HWVN;
const char BTVN[] = _BTVN;
const char APVN[] = _APVN;
const char PMVN[] = _PMVN;
const char ANGL[] = _ANGL;
const char CMST[] = _CMST;
const char TLIV[] = _TLIV;
const char STST[] = _STST;
const char STTS[] = _STTS;

///
/// @brief array of cmwr specific command strings
///
const std::string cmwr_nfc_parameter[CMWR_PARAMETER_COUNT] = {IMEI,
                                                              MODL,
                                                              MFDT,
                                                              HWVN,
                                                              BTVN,
                                                              APVN,
                                                              PMVN,
                                                              ANGL,
                                                              CMST,
                                                              TLIV,
                                                              STST,
                                                              STTS};

//------------------------------------------------------------------------------------------------

const size_t CMWR_COMMAND_COUNT = 2;

#define _READ "read"
#define _RESET "reset"

const std::string command_prefix = "command";

const char READ[] = _READ;
const char RESET[] = _RESET;

const std::string cmwr_command[CMWR_COMMAND_COUNT] = {READ,
                                                      RESET};


/// @brief supported commands
enum class CMWR_Command : uint8_t
{
    none = 0x00,
    read = 0x01,
    reset = 0x02
};

//------------------------------------------------------------------------------------------------

#pragma region METHOD PROTOTYPES
bool CheckNeedle(uint8_t *, uint8_t *, size_t, size_t);
bool CompareTagIdentifier(uint8_t *);
char *Substring(char *, int, int);
const char *HexStr(const uint8_t *, int, bool);
size_t WriteToSPP(uint8_t);
static void onBLEWritten(BLEDevice, BLECharacteristic);
void AddBatteryServiceBLE();
void AddDataServiceBLE();
void AddDeviceServiceBLE();
void AtTime();
void bluetooth_thread();
void InsertSubstring(char *, const char *, int);
void main_thread();
void onBLEConnected(BLEDevice);
void onBLEDisconnected(BLEDevice);
void onRxCharValueUpdate(BLEDevice, BLECharacteristic);
void ProcessReceivedQueries();
void publish_tag();
void PublishHardwareDetails();
void PublishResponseToBluetooth(char *, size_t);
void ResetReader();
void SetTagIdentifier(uint8_t *);
void SetupBLE();
void SimulateSensor();
void StartBLE();
void TagDetectedInterrupt();
#pragma endregion