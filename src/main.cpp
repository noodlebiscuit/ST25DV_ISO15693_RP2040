/**************************************************************************************************
 * Author: Alex Pinkerton
 *
 * License: (c) 2021, MIT LICENSE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions: The above copyright notice and this
 * permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 * Description: main.cpp
 * =====================
 *
 * The software allows you to emulate the functionality of a FEIG ECCO+ BLE NearField reader
 * using an ARDUINO NANO 33 BLE (with a Nordic NRF-52840 Microcontroller) as well as a PN532
 * NFC reader board from either SUNFOUNDER or ADA FRUIT.
 *
 * The protocol being used is the proprietary SCANNDY SComP by PANMOBIL (FEIG)
 *
 * No source code from either FEIG or PANMOBIL is contained in this firmware, and it is provided
 * purely to allow engineers who are developing for the ECCO+, to be able to debug Bluetooth data
 * at a VERY low level. It is ABSOLUTELY NOT intended for use in ANY commercial application!
 *
 * The author CANNOT guarantee that everything here is correct, and FEIG has no involvement with
 * the project at ANY level.
 *
 * July 2023
 *
 ***************************************************************************************************/

#include "main.h"

//------------------------------------------------------------------------------------------------

volatile bool _blockTimerEvents = false;

volatile bool _bluetoothConnected = false;

volatile bool _blockBannerText = true;

/// @brief main thread timer event latch
volatile bool timerEvent = false;

/// @brief have we detected anything on the NFC eenergy line?
volatile bool tagDetectedEvent = false;

/// @brief  block access to the reader hardware?
volatile bool _blockReader = false;

/// @brief
volatile bool reader_detected = false;

/// @brief
volatile bool sensor_starting = false;

/// @brief main IO application thread
rtos::Thread t1;

/// @brief secondary bluetooth thread
rtos::Thread t2;

/// @brief NFC user memory
uint8_t tagMemory[ISO15693_USER_MEMORY];

/// @brief NFC tag object
SFE_ST25DV64KC_NDEF tag;

/// @brief  references the UID from the TAG to block multiple reads
uint8_t _headerdata[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/// @brief what command type was issued by the connected client?
CMWR_Parameter _scomp_command = none;

/// @brief represent time in seconds from sensor receiving cmd:cmsd to it starting0071Q0014#tliv:3.47 2312041113ed020960
volatile uint8_t sensor_startup_count = 0x00;

//------------------------------------------------------------------------------------------------

#pragma region BLUETOOTH LOW ENERGY SUPPORT
///
/// @brief Configure the BLE hardware
///
void SetupBLE()
{
   // initiate BLE comms
   StartBLE();

   // Create BLE service and characteristics.
   BLE.setDeviceName(PERIPERHAL_DEVICE_NAME);
   BLE.setLocalName(LOCAL_NAME_OF_PERIPHERAL);
   BLE.setManufacturerData(SKF_MANUFACTURER_CODE, 2);

   // configure the main NFC communications service
   AddDataServiceBLE();

   // configure the battery level service
   AddBatteryServiceBLE();

   // configure the device information service
   AddDeviceServiceBLE();

   // Bluetooth LE connection handlers.
   BLE.setEventHandler(BLEConnected, onBLEConnected);
   BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);

   // Event driven reads.
   // rxChar.setEventHandler(BLEWritten, onRxCharValueUpdate);
   rxChar.setEventHandler(BLEWritten, onBLEWritten);

   // Let's tell all local devices about us.
   BLE.advertise();

   // set the default characteristics
   PublishHardwareDetails();

   // clear the BLE serial receive buffer
   _SerialBuffer.clear();
}

///
/// @brief Start the BLE service!
///
void StartBLE()
{
   if (!BLE.begin())
   {
#ifdef READER_DEBUG
      READER_DEBUGPRINT.println("starting BLE failed!");
#endif
      while (1)
         ;
   }
}

///
/// @brief Publish the initial details of this device at POWER ON
///
void PublishHardwareDetails()
{
   manufacturerCharacteristic.writeValue(MANUFACTURER_NAME_STRING, false);
   modelNumberCharacteristic.writeValue(MODEL_NAME_STRING, false);
   hardwareCharacteristic.writeValue(HARDWARE_NAME_STRING, false);
   firmwareRevisionCharacteristic.writeValue(FIRMWARE_NAME_STRING, false);
   serialNumberCharacteristic.writeValue(SERIAL_NO_NAME_STRING, false);
}

///
/// @brief Add the device information service and associate the required characteristics
///
void AddDeviceServiceBLE()
{
   BLE.setAdvertisedService(deviceInfoService);
   deviceInfoService.addCharacteristic(manufacturerCharacteristic);
   deviceInfoService.addCharacteristic(modelNumberCharacteristic);
   deviceInfoService.addCharacteristic(firmwareRevisionCharacteristic);
   deviceInfoService.addCharacteristic(serialNumberCharacteristic);
   deviceInfoService.addCharacteristic(hardwareCharacteristic);
   BLE.addService(deviceInfoService);
}

///
/// @brief Add the battery state service and associate the required characteristics
///
void AddBatteryServiceBLE()
{
   BLE.setAdvertisedService(batteryService);
   batteryService.addCharacteristic(batteryCharacteristic);
   BLE.addService(batteryService);
}

///
/// @brief Add the core data service and associate the required characteristics
///
void AddDataServiceBLE()
{
   BLE.setAdvertisedService(nearFieldService);
   nearFieldService.addCharacteristic(rxChar);
   nearFieldService.addCharacteristic(txChar);
   BLE.addService(nearFieldService);
}

///
/// @brief A BLE device has connected to our sensor - illuminate the connection LED
/// @param cental BLE device
///
void onBLEConnected(BLEDevice central)
{
   LED_SetConnectedToBLE = HIGH;
   ResetReader();
   READER_DEBUGPRINT.println(F("Connected"));
}

///
/// @brief A BLE device has just disconnected from our sensor - power off the connection LED
/// @param central BLE device
///
void onBLEDisconnected(BLEDevice central)
{
   LED_SetConnectedToBLE = LOW;
   READER_DEBUGPRINT.println(F("Disconnected"));
}

///
/// @brief Reads and returns an averaged value for the 3.7V Lithium ion battery in MV
/// @param PIN what analogue pin are we connecting to?
/// @param average how many samples to read and average
/// @return voltage as a big-endian integer
///
uint16_t ReadBattery(pin_size_t PIN, int average)
{
   //  float value = 0.0;
   //  for (int i = 0; i < average; ++i)
   //  {
   //     value += (float)analogRead(PIN);
   //     delayMicroseconds(BATTERY_READ_DELAY);
   //  }
   //  value = (value * BATTERY_DESCALE) / average;
   //  return (uint16_t)(value);
}

///
/// @brief Streams the NDEF contents out over Bluetooth as a series HEX NOTATION characters
/// @brief Select using SET_OUTPUT_AS_BINARY
/// @brief Example: UID 04:4d:ec:b4 will be returned as "044decb4"
/// @param pagedata raw NDEF message payload
/// @param headerdata NDEF meassage header with UUID
///
void PublishHexPayloadToBluetooth(uint8_t *pagedata, uint8_t *headerdata)
{
   //  const char *hexNotation;
   //  uint8_t *queryBody = new uint8_t[BLOCK_SIZE_BLE];

   //  // we make a global copy of the UID to prevent multiple reads of empty TAGS
   //  SetTagIdentifier(headerdata);

   //  // make sure we don't have any NFC scanning overlaps here
   //  _readerBusy = true;

   //  // what is the total message size in bytes? We get this from the TAG data itself
   //  int message_length = pagedata[1] + 3;

   //  // convert the TAG header into HEX NOTATION strings (as opposed to raw binary)
   //  hexNotation = HexStr(headerdata, BLOCK_SIZE_BLE, HEX_UPPER_CASE);

   //  // now we need to double the length of the message string (ONE byte value needs TWO chars)
   //  message_length *= 2;

   //  // how many bytes is this payload going to contain in total?
   //  uint16_t totalBytes = RFID_RESPONSE_BYTES + (BLOCK_SIZE_BLE * 2) + (message_length);

   //  // set the SCOMP PROTOCOL total TAG payload length
   //  PAYLOAD_LEGTH[0] = (uint8_t)((totalBytes & 0xff00) >> 8);
   //  PAYLOAD_LEGTH[1] = (uint8_t)(totalBytes & 0x00ff);

   //  // insert the payload length into the SCOMP PROTOCOL RFID DATA HEADER
   //  const char *payloadLength = HexStr(PAYLOAD_LEGTH, LENGTH_BYTES, HEX_UPPER_CASE);
   //  for (int i = 0; i < (int)sizeof(payloadLength); i++)
   //  {
   //     scomp_rfid_response_header[i + 5] = payloadLength[i];
   //  }

   //  // generate the CRC for the SCANNDY PROTOCOL HEADER
   //  crc.update(scomp_rfid_response_header, HEADER_BYTES);

   //  // PUBLISH SCANNDY PROTOCOL HEADER TO BLUETOOTH
   //  txChar.writeValue(scomp_rfid_response_header, false);
   //  delayMicroseconds(BLOCK_WAIT_BLE_HEX);

   //  // generate the CRC for the NFC (ISO 14443) header block
   //  crc.update(hexNotation, (BLOCK_SIZE_BLE * 2));

   //  // reset the page index
   //  int index = 0;

   //  // PUBLISH THE NTAG (ISO14443) 16 BYTES UUID HEADER
   //  for (int k = 0; k < 2; k++)
   //  {
   //     memset(queryBody, 0, BLOCK_SIZE_BLE);
   //     for (int i = 0; i < BLOCK_SIZE_BLE; i++)
   //     {
   //        queryBody[i] = (uint8_t)hexNotation[i + (index * BLOCK_SIZE_BLE)];
   //     }
   //     txChar.writeValue(queryBody, BLOCK_SIZE_BLE);
   //     delayMicroseconds(BLOCK_WAIT_BLE_HEX);
   //     index++;
   //  }

   //  // convert the TAG PAGE DATA into HEX NOTATION strings (as opposed to raw binary)
   //  hexNotation = HexStr(pagedata, message_length, HEX_UPPER_CASE);

   //  // generate the CRC for the NFC (ISO 14443) user data payload (NDEF)
   //  crc.update(hexNotation, (message_length));

   //  // reset the page index
   //  index = 0;

   //  // PUBLISH THE NTAG (ISO14443) USER DATA (AKA NDEF)
   //  while (message_length >= 0)
   //  {
   //     // flush the transmission buffer and allow for some delay
   //     memset(queryBody, 0, BLOCK_SIZE_BLE);
   //     delayMicroseconds(BLOCK_WAIT_BLE_HEX);

   //     //
   //     // we initially transmit data in 16 byte blocks, then transmit the
   //     // remaining bytes together in a single payload
   //     //
   //     if (message_length >= BLOCK_SIZE_BLE)
   //     {
   //        for (int i = 0; i < BLOCK_SIZE_BLE; i++)
   //        {
   //           queryBody[i] = (uint8_t)hexNotation[i + (index * BLOCK_SIZE_BLE)];
   //        }
   //        txChar.writeValue(queryBody, BLOCK_SIZE_BLE);
   //        index++;
   //     }
   //     else
   //     {
   //        for (int i = 0; i < message_length; i++)
   //        {
   //           queryBody[i] = (uint8_t)hexNotation[i + (index * BLOCK_SIZE_BLE)];
   //        }
   //        txChar.writeValue(queryBody, message_length);
   //     }
   //     message_length -= BLOCK_SIZE_BLE;
   //  }

   //  // release this memory
   //  delete[] queryBody;

   //  // add the serial port delay to improve comms efficiency
   //  delayMicroseconds(BLOCK_WAIT_BLE_HEX);

   //  // publish the final CRC as an array of bytes
   //  crc.finalizeAsArray(EOR);
   //  const char *crcValue = HexStr(EOR, FOOTER_BYTES, HEX_UPPER_CASE);
   //  txChar.writeValue(crcValue, false);
   //  crc.reset();

   //  // close for DEBUG
   //  delayMicroseconds(BLOCK_WAIT_BLE_HEX);
   //  txChar.writeValue(CR_LF, 2);

   //  // release the blocker
   //  _readerBusy = false;
}

///
/// @brief Streams the NDEF contents out over Bluetooth as a series of 16 byte packets
/// @brief Select using SET_OUTPUT_AS_BINARY
/// @param pagedata raw NDEF message payload
/// @param headerdata NDEF meassage header with UUID
///
void PublishBinaryPayloadToBluetooth(uint8_t *pagedata, uint8_t *headerdata)
{
   //  // we make a global copy of the UID to prevent multiple reads of empty TAGS
   //  SetTagIdentifier(headerdata);

   //  // make sure we don't have any NFC scanning overlaps here
   //  _readerBusy = true;

   //  // what is the total message size in bytes?
   //  int message_length = pagedata[1] + 3;

   //  // how many bytes is this payload going to contain?
   //  uint16_t totalBytes = RFID_RESPONSE_BYTES + BLOCK_SIZE_BLE + message_length;

   //  // set the SCOMP PROTOCOL total TAG payload length
   //  PAYLOAD_LEGTH[0] = (uint8_t)((totalBytes & 0xff00) >> 8);
   //  PAYLOAD_LEGTH[1] = (uint8_t)(totalBytes & 0x00ff);

   //  // insert the payload length into the SCOMP PROTOCOL RFID DATA HEADER
   //  const char *payloadLength = HexStr(PAYLOAD_LEGTH, LENGTH_BYTES, HEX_UPPER_CASE);
   //  for (int i = 0; i < (int)sizeof(payloadLength); i++)
   //  {
   //     scomp_rfid_response_header[i + 5] = payloadLength[i];
   //  }

   //  // generate the CRC for the payload header block
   //  crc.update(scomp_rfid_response_header, HEADER_BYTES);

   //  // PUBLISH SCANNDY PROTOCOL HEADER TO BLUETOOTH
   //  txChar.writeValue(scomp_rfid_response_header, false);

   //  // generate the CRC for the NFC (ISO 14443) header block
   //  crc.update(headerdata, BLOCK_SIZE_BLE);

   //  // PUBLISH ISO14443 TAG DATA TO BLUETOOTH
   //  delayMicroseconds(BLOCK_WAIT_BLE);
   //  txChar.writeValue(headerdata, BLOCK_SIZE_BLE);

   //  // reset the page index
   //  int index = 0;

   //  // write out each block of the received payload
   //  while (message_length >= 0)
   //  {
   //     delayMicroseconds(BLOCK_WAIT_BLE);
   //     if (message_length >= BLOCK_SIZE_BLE)
   //     {
   //        txChar.writeValue(pagedata + (index * BLOCK_SIZE_BLE), BLOCK_SIZE_BLE);
   //        index++;
   //     }
   //     else
   //     {
   //        txChar.writeValue(pagedata + (index * BLOCK_SIZE_BLE), message_length);
   //     }
   //     message_length -= BLOCK_SIZE_BLE;
   //  }

   //  // append the CRC based on the transmitted payload
   //  message_length = pagedata[1] + 3;
   //  crc.update(pagedata, message_length);

   //  // add the serial port delay to improve comms efficiency
   //  delayMicroseconds(BLOCK_WAIT_BLE);

   //  // publish the final CRC as an array of bytes
   //  crc.finalizeAsArray(EOR);
   //  const char *crcValue = HexStr(EOR, FOOTER_BYTES, HEX_UPPER_CASE);
   //  txChar.writeValue(crcValue, false);
   //  crc.reset();

   //  // close for DEBUG
   //  delayMicroseconds(BLOCK_WAIT_BLE);
   //  txChar.writeValue(CR_LF, 2);

   // release the blocker
   _readerBusy = false;
}

///
/// @brief Streams the NDEF UID header out over Bluetooth as a single sixteen byte packet
/// @param headerdata NDEF meassage header with UUID
///
void PublishBinaryUIDToBluetooth(uint8_t *headerdata)
{
   //  //
   //  // if the UID matches beacuse we've just read this device, then force an arbitrary
   //  // delay here. This is to prevent multiple reads of the same TAG from swamping the
   //  // BLE comms and hammering the battery
   //  //
   //  if (CompareTagIdentifier(headerdata))
   //  {
   //     delayMicroseconds(MULTIPLE_READ_WAIT);
   //  }

   //  // reference a newly received UID from an empty card
   //  SetTagIdentifier(headerdata);

   //  // make sure we don't have any NFC scanning overlaps here
   //  _readerBusy = true;

   //  // how many bytes is this payload going to contain?
   //  uint16_t totalBytes = RFID_RESPONSE_BYTES + BLOCK_SIZE_BLE;

   //  // set the SCOMP PROTOCOL total TAG payload length
   //  PAYLOAD_LEGTH[0] = (uint8_t)((totalBytes & 0xff00) >> 8);
   //  PAYLOAD_LEGTH[1] = (uint8_t)(totalBytes & 0x00ff);

   //  // insert the payload length into the SCOMP PROTOCOL RFID DATA HEADER
   //  const char *payloadLength = HexStr(PAYLOAD_LEGTH, LENGTH_BYTES, HEX_UPPER_CASE);
   //  for (int i = 0; i < (int)sizeof(payloadLength); i++)
   //  {
   //     scomp_rfid_response_header[i + 5] = payloadLength[i];
   //  }

   //  // generate the CRC for the payload header block
   //  crc.update(scomp_rfid_response_header, HEADER_BYTES);

   //  // PUBLISH SCANNDY PROTOCOL HEADER TO BLUETOOTH
   //  txChar.writeValue(scomp_rfid_response_header, false);

   //  // generate the CRC for the NFC (ISO 14443) header block
   //  crc.update(headerdata, BLOCK_SIZE_BLE);

   //  // PUBLISH ISO14443 TAG DATA TO BLUETOOTH
   //  delayMicroseconds(BLOCK_WAIT_BLE);
   //  txChar.writeValue(headerdata, BLOCK_SIZE_BLE);

   //  // add the serial port delay to improve comms efficiency
   //  delayMicroseconds(BLOCK_WAIT_BLE);

   //  // publish the final CRC as an array of bytes
   //  crc.finalizeAsArray(EOR);
   //  const char *crcValue = HexStr(EOR, FOOTER_BYTES, HEX_UPPER_CASE);
   //  txChar.writeValue(crcValue, false);
   //  crc.reset();

   //  // close for DEBUG
   //  delayMicroseconds(BLOCK_WAIT_BLE);
   //  txChar.writeValue(CR_LF, 2);

   // release the blocker
   _readerBusy = false;
}

///
/// @brief Streams the NDEF UID header out over Bluetooth as a single sixteen byte packet
/// @param headerdata NDEF meassage header with UUID
///
void PublishHexUIDToBluetooth(uint8_t *headerdata)
{
   //  //
   //  // if the UID matches beacuse we've just read this device, then force an arbitrary
   //  // delay here. This is to prevent multiple reads of the same TAG from swamping the
   //  // BLE comms and hammering the battery
   //  //
   //  if (CompareTagIdentifier(headerdata))
   //  {
   //     delayMicroseconds(MULTIPLE_READ_WAIT);
   //  }

   //  // reference a newly received UID from an empty card
   //  SetTagIdentifier(headerdata);

   //  const char *hexNotation;
   //  uint8_t *queryBody = new uint8_t[BLOCK_SIZE_BLE];

   //  // convert the TAG header into HEX NOTATION strings (as opposed to raw binary)
   //  hexNotation = HexStr(headerdata, BLOCK_SIZE_BLE, HEX_UPPER_CASE);

   //  // make sure we don't have any NFC scanning overlaps here
   //  _readerBusy = true;

   //  // how many bytes is this payload going to contain?
   //  uint16_t totalBytes = RFID_RESPONSE_BYTES + (BLOCK_SIZE_BLE * 2);

   //  // set the SCOMP PROTOCOL total TAG payload length
   //  PAYLOAD_LEGTH[0] = (uint8_t)((totalBytes & 0xff00) >> 8);
   //  PAYLOAD_LEGTH[1] = (uint8_t)(totalBytes & 0x00ff);

   //  // insert the payload length into the SCOMP PROTOCOL RFID DATA HEADER
   //  const char *payloadLength = HexStr(PAYLOAD_LEGTH, LENGTH_BYTES, HEX_UPPER_CASE);
   //  for (int i = 0; i < (int)sizeof(payloadLength); i++)
   //  {
   //     scomp_rfid_response_header[i + 5] = payloadLength[i];
   //  }

   //  // generate the CRC for the payload header block
   //  crc.update(scomp_rfid_response_header, HEADER_BYTES);

   //  // PUBLISH SCANNDY PROTOCOL HEADER TO BLUETOOTH
   //  txChar.writeValue(scomp_rfid_response_header, false);

   //  // generate the CRC for the NFC (ISO 14443) header block
   //  crc.update(hexNotation, (BLOCK_SIZE_BLE * 2));

   //  // reset the page index
   //  int index = 0;

   //  // PUBLISH THE NTAG (ISO14443) 16 BYTES UUID HEADER
   //  for (int k = 0; k < 2; k++)
   //  {
   //     memset(queryBody, 0, BLOCK_SIZE_BLE);
   //     for (int i = 0; i < BLOCK_SIZE_BLE; i++)
   //     {
   //        queryBody[i] = (uint8_t)hexNotation[i + (index * BLOCK_SIZE_BLE)];
   //     }
   //     txChar.writeValue(queryBody, BLOCK_SIZE_BLE);
   //     delayMicroseconds(BLOCK_WAIT_BLE_HEX);
   //     index++;
   //  }

   //  // add the serial port delay to improve comms efficiency
   //  delayMicroseconds(BLOCK_WAIT_BLE_HEX);

   //  // publish the final CRC as an array of bytes
   //  crc.finalizeAsArray(EOR);
   //  const char *crcValue = HexStr(EOR, FOOTER_BYTES, HEX_UPPER_CASE);
   //  txChar.writeValue(crcValue, false);
   //  crc.reset();

   //  // close for DEBUG
   //  delayMicroseconds(BLOCK_WAIT_BLE_HEX);
   //  txChar.writeValue(CR_LF, 2);

   // release the blocker
   _readerBusy = false;
}

///
/// @brief Reset the reader after RTOS timeout and erase the NDEF message cache
///
void ResetReader()
{
   _readerBusy = false;
   _blockReader = false;
   _SerialBuffer.clear();
   _messageIdentifier = 0x0000;
   _queryReceived = false;
   _invalidQueryReceived = false;
}
#pragma endregion

//------------------------------------------------------------------------------------------------

///
/// @brief configure application
///
void setup()
{
   READER_DEBUGPRINT.begin(SERIAL_BAUD_RATE);

   // attach interrupt handler to the received ST25DV GPO signal
   OnTagDetectedInterrupt.fall(&TagDetectedInterrupt);

   MyWire.begin();
   if (tag.begin(MyWire))
   {

      // The GPO registers can only be changed during an open security session
      uint8_t password[8] = {0x00};
      tag.openI2CSession(password);
      tag.setGPO1Bit(BIT_GPO1_FIELD_CHANGE_EN, false);
      tag.setGPO1Bit(BIT_GPO1_RF_USER_EN, false);
      tag.setGPO1Bit(BIT_GPO1_RF_ACTIVITY_EN, false);
      tag.setGPO1Bit(BIT_GPO1_RF_INTERRUPT_EN, true);
      tag.setGPO1Bit(BIT_GPO1_RF_PUT_MSG_EN, false);
      tag.setGPO1Bit(BIT_GPO1_RF_GET_MSG_EN, false);
      tag.setGPO1Bit(BIT_GPO1_RF_WRITE_EN, true);
      tag.setGPO1Bit(BIT_GPO1_GPO_EN, true);

      // Clear the first TAG of user memory
      memset(tagMemory, 0, ISO15693_USER_MEMORY);

      READER_DEBUGPRINT.println("Writing 0x0 to the first 256 bytes of user memory.");
      tag.writeEEPROM(0x0, tagMemory, ISO15693_USER_MEMORY);

      // Write the Type 5 CC File - starting at address zero
      READER_DEBUGPRINT.println(F("Writing CC_File"));
      tag.writeCCFile8Byte();

      publish_tag();
   }

   // now we setup all services within the BLE layer
   SetupBLE();

   // start the main and bluetooth threads running
   t1.start(main_thread);
   t2.start(bluetooth_thread);

   // set the timeout value
   timer.attach(&AtTime, TICK_RATE_MS);
}

///
/// @brief Write CMWR record to TAG
///
void publish_tag()
{
   // Write two NDEF UTF-8 Text records
   uint16_t memLoc = tag.getCCFileLen();

   // populate all other properties
   tag.writeNDEFText(sensor.GetSensorProperty(CMWR_Parameter::imei).c_str(), &memLoc, true, false);
   tag.writeNDEFText(sensor.GetSensorProperty(CMWR_Parameter::modl).c_str(), &memLoc, false, false);
   tag.writeNDEFText(sensor.GetSensorProperty(CMWR_Parameter::mfdt).c_str(), &memLoc, false, false);
   tag.writeNDEFText(sensor.GetSensorProperty(CMWR_Parameter::hwvn).c_str(), &memLoc, false, false);
   tag.writeNDEFText(sensor.GetSensorProperty(CMWR_Parameter::btvn).c_str(), &memLoc, false, false);
   tag.writeNDEFText(sensor.GetSensorProperty(CMWR_Parameter::apvn).c_str(), &memLoc, false, false);
   tag.writeNDEFText(sensor.GetSensorProperty(CMWR_Parameter::pmvn).c_str(), &memLoc, false, false);
   tag.writeNDEFText(sensor.GetSensorProperty(CMWR_Parameter::angl).c_str(), &memLoc, false, false);
   tag.writeNDEFText(sensor.GetSensorProperty(CMWR_Parameter::cmst).c_str(), &memLoc, false, false);
   tag.writeNDEFText(sensor.GetSensorProperty(CMWR_Parameter::tliv).c_str(), &memLoc, false, false);
   tag.writeNDEFText(sensor.GetSensorProperty(CMWR_Parameter::stst).c_str(), &memLoc, false, false);
   tag.writeNDEFText(sensor.GetSensorProperty(CMWR_Parameter::stts).c_str(), &memLoc, false, true);
}

///
/// @brief executes the PRIMARY thread
///
void main_thread()
{
   while (true)
   {
      if (timerEvent & !_bluetoothConnected)
      {
         timerEvent = false;

         if (reader_detected & !sensor_starting)
         {
            READER_DEBUGPRINT.println('*');
            READER_DEBUGPRINT.println("READER DETECTED");

            // reset the reader detected flag
            reader_detected = false;

            // Read 16 bytes from EEPROM location 0x0
            uint8_t tagRead[ISO15693_USER_MEMORY] = {0};

            // Read the EEPROM: start at address 0x00, read contents into tagRead; read 16 bytes
            tag.readEEPROM(0x00, tagRead, ISO15693_USER_MEMORY);

            // is the sensor starting? we detect this by checking for the char array 'cmd:cmsd'
            sensor_starting = CheckNeedle(tagRead, COMMAND_CMSD, ISO15693_USER_MEMORY, 8);
         }

         //
         // if the sensor is starting we initiate a countdown of 'n' seconds
         // during which time we DO NOT continue to read the EEPROM contents
         //
         else if (sensor_starting)
         {
            READER_DEBUGPRINT.print("*");
            if (sensor_startup_count++ > 60)
            {
               sensor_startup_count = 0x00;
               sensor_starting = false;
               reader_detected = false;
               tagDetectedEvent = false;
            }
         }
         else
         {
            READER_DEBUGPRINT.print(".");
         }

         if (tagDetectedEvent)
         {
            reader_detected = true;
            tagDetectedEvent = false;
         }
      }
   }
}

///
/// @brief executes the SECONDARY (bluetooth low energy) thread
///
void bluetooth_thread()
{
   while (true)
   {
      // top priority here is the BLE controller
      BLEDevice central = BLE.central();
      if (central)
      {
         // we need
         _bluetoothConnected = true;

         // inner loop when connected to BLUETOOTH
         while (central.connected())
         {
            if (timerEvent)
            {
               READER_DEBUGPRINT.print("+");
               ProcessReceivedQueries();
               timerEvent = false;
            }
         }
      }
      else
      {
         _bluetoothConnected = false;
      }
   }
}

/// @brief not used
void loop()
{
}

///
/// @brief MBED timer tick event *** APPLICATION CORE ***
///
void AtTime()
{
   timerEvent = true;
}

/// @brief TAG read event has been raised on GPIO-9
void TagDetectedInterrupt()
{
   tagDetectedEvent = true;
}

//-------------------------------------------------------------------------------------------------

#pragma region STRING MANAGEMENT AND SUPPORT

///
/// @brief
/// @param buffer source byte array to seach against
/// @param cmd reference byte array to search for within the buffer
/// @param buffer_length length of the buffer array
/// @param cmd_length length of the reference array
///
bool CheckNeedle(uint8_t *buffer, uint8_t *cmd, size_t buffer_length, size_t cmd_length)
{
   std::string needle(cmd, cmd + cmd_length);
   std::string haystack(buffer, buffer + buffer_length); // or "+ sizeof Buffer"
   std::size_t n = haystack.find(needle);
   return (n != std::string::npos);
}

///
/// @brief inserts on string into another
/// @param a source string
/// @param b substring
/// @param position insert position
///
void InsertSubstring(char *a, const char *b, int position)
{
   char *f, *e;
   int length;

   length = strlen(a);

   f = Substring(a, 1, position - 1);
   e = Substring(a, position, length - position + 1);

   strcpy(a, "");
   strcat(a, f);
   free(f);
   strcat(a, b);
   strcat(a, e);
   free(e);
}

///
/// @brief
/// @param string raw string
/// @param position starting character index
/// @param length number of characters to extract
/// @return
///
char *Substring(char *string, int position, int length)
{
   char *pointer;
   int c;

   pointer = (char *)malloc(length + 1);

   if (pointer == NULL)
      exit(EXIT_FAILURE);

   for (c = 0; c < length; c++)
      *(pointer + c) = *((string + position - 1) + c);

   *(pointer + c) = '\0';

   return pointer;
}

///
/// @brief converts an array of bytes into an ASCII string
/// @brief E.g. {0x22, 0x4a, 0x0f, 0xe2} would return as '224a0fe2'
/// @param data array of bytes
/// @param len number of bytes to process
/// @param uppercase return hex string with value A->F in upper case
/// @return standard C string (array of chars)
///
const char *HexStr(const uint8_t *data, int len, bool uppercase)
{
   std::stringstream ss;
   ss << std::hex;

   for (int i(0); i < len; ++i)
   {
      ss << std::setw(2) << std::setfill('0') << (int)data[i];
   }

   std::string x = ss.str();
   if (uppercase)
   {
      std::transform(x.begin(), x.end(), x.begin(), ::toupper);
   }
   return x.c_str();
}

///
/// @brief writes the UID of the last TAG read to a global cache
/// @param headerdata first eight bytes of an ISO 14443 (NDEF) card
///
// void SetTagIdentifier(uint8_t *headerdata)
// {
//    for (int i = 0; i < 8; i++)
//    {
//       NTAG_UUID[i] = headerdata[i];
//    }
// }

///
/// @brief inserts on string into another
/// @param headerdata first eight bytes of an ISO 14443 (NDEF) card
/// @return true if the eight ISO 14443 header bytes match
///
// bool CompareTagIdentifier(uint8_t *headerdata)
// {
//    uint8_t count = 0;
//    for (uint8_t i = 0x00; i < 0x08; i++)
//    {
//       if (NTAG_UUID[i] == headerdata[i])
//       {
//          count++;
//       }
//    }
//    return count >= 0x08;
// }

///
/// @brief clear contents of the TAG identifier
///
// void ClearTagIdentifier()
// {
//    for (uint8_t i = 0x00; i < 0x08; i++)
//    {
//       NTAG_UUID[i] == 0x00;
//    }
// }
#pragma endregion

//-------------------------------------------------------------------------------------------------

// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************
// ************************************************************************************************

///
/// @brief EVENT on data written to SPP
/// @brief Message is of format as shown below. Initially we search for chars 'Q'  and  '#'
/// @brief
/// @brief     ->    nnnn Q pppp # n....n cccccccc
/// @brief
/// @brief When we've detected this two character sequence,  we then need to move back FOUR
/// @brief characters so that we're pointing to the characters 'PPPP' - which represent the
/// @brief complete length of the payload as a four-character long HEX string.
/// @brief
/// @brief     ->    .... Q PPPP # nnnnnn cccccccc
/// @brief
/// @brief That done we read PPPP bytes from the input stream and take that as the message.
/// @brief Then we take all received characters (nnnnQpppp#n....n) and calculate the CRC32.
/// @brief Lastly we compate the two CRC32 values to determine if the query is valid
/// @param central connected Bluetooth device
/// @param characteristic data associated with the serial (SPP) BLE service
///
void onBLEWritten(BLEDevice central, BLECharacteristic characteristic)
{
   // at this point we don't want the reader to keep checking for Tags
   _blockReader = true;

   //
   // before we do anything, we need to confirm that we're not looking at a potential buffer
   // overrun here! If we are, then we need to flush the serial receive buffer
   //
   if ((characteristic.valueLength() + _SerialBuffer.getLength()) > _SerialBuffer.getSize())
   {
      _SerialBuffer.clear();
   }

   // that done, we should now be safe to load the received BLE data into the receive buffer
   for (int i = 0; i < characteristic.valueLength(); i++)
   {
      _SerialBuffer.add(characteristic.value()[i]);
   }

   //
   // now we need to find the start of an SCOMP QUERY. This is done by searhing for the
   // character sequence nnnnQpppp#, where nn and pp are two character HEX strings
   //
   int startOfSequence = 0;
   if (_SerialBuffer.getLength() > QUERY_HEADER_BYTES)
   {
      for (int i = 0; i < (_SerialBuffer.getLength() - 5); i++)
      {
         if ((char)_SerialBuffer.get(i) == 'Q' & (char)_SerialBuffer.get(i + 5) == '#')
         {
            startOfSequence = i;
            break;
         }
      }
   }

   // if posn is four or above, then we have detected the start of an SCOMP QUERY string
   if (startOfSequence >= QUERY_OFFSET_BYTES)
   {
      // jump back to the start of the message (to include the request ID)
      startOfSequence -= QUERY_OFFSET_BYTES;

      // we need a temporary buffer here
      char *buffer = new char[RECEIVE_BUFFER_LENGTH];

      // clear the buffer contents
      memset(buffer, 0, RECEIVE_BUFFER_LENGTH);

      // OK, lock and load, and let's see what's in the barrel!
      int index = 0;
      for (int i = startOfSequence; i < (int)(_SerialBuffer.getLength() - startOfSequence); i++)
      {
         buffer[index++] = (char)_SerialBuffer.get(i);
      }

      // get the total payload length
      char *payloadLengthString = new char[5];
      memset(payloadLengthString, 0, 5);

      //
      // well, it's the payload length. Now we need to covert this four character long
      // representation of of a sixteen bit number - into an actual sixteen bit number
      //
      for (int i = 0; i < 4; i++)
      {
         payloadLengthString[i] = buffer[i + 5];
      }
      payloadLengthString[4] = '\n';

      //
      // now we need to convert the eight character long length string into a LONG value
      // and then move that to a sixteen bit unsigned value
      //
      char *ptr;
      uint16_t payloadLength = (uint16_t)strtol(payloadLengthString, &ptr, 16);

      // But what is the total length of the message (i.e. how many chars to we need?)
      uint16_t totalPayloadLength = payloadLength + CRC32_CHARACTERS + QUERY_HEADER_BYTES;

      // we need to extract the core payloads - one with, and one without the CRC32
      char *queryPayload = new char[payloadLength + QUERY_HEADER_BYTES + 1];
      char *queryID = new char[QUERY_OFFSET_BYTES + 1];
      char *queryBody = new char[payloadLength + 1];
      char *queryCRC32 = new char[CRC32_CHARACTERS + 1];
      char *valueCRC32 = new char[3];

      // OK, have all the required character been received yet?
      if (totalPayloadLength == _SerialBuffer.getLength())
      {
         // clear the buffer contents again..
         memset(buffer, 0, RECEIVE_BUFFER_LENGTH);
         memset(queryPayload, 0, payloadLength + QUERY_HEADER_BYTES + 1);
         memset(queryCRC32, 0, CRC32_CHARACTERS + 1);
         memset(valueCRC32, 0, 3);
         memset(queryID, 0, QUERY_OFFSET_BYTES + 1);
         memset(queryBody, 0, QUERY_OFFSET_BYTES + 1);

         // extract the complete complete SCOMP QUERY payload (with CRC32)
         index = 0;
         for (int i = startOfSequence; i < (int)(totalPayloadLength - (startOfSequence)); i++)
         {
            buffer[index++] = _SerialBuffer.get(i);
         }

         // extract ONLY the HEADER and the QUERY (we use this to calculate the CRC32)
         for (int i = 0; i < payloadLength + QUERY_HEADER_BYTES; i++)
         {
            queryPayload[i] = buffer[i];
         }
         queryPayload[payloadLength + QUERY_HEADER_BYTES] = (char)0x00;

         // extract only the payload body
         for (int i = 0; i < payloadLength; i++)
         {
            queryBody[i] = buffer[i + QUERY_HEADER_BYTES];
         }
         queryBody[payloadLength] = (char)0x00;

         // extract only the query message identifier
         for (int i = 0; i < QUERY_OFFSET_BYTES; i++)
         {
            queryID[i] = buffer[i];
            scomp_query_ID[i] = buffer[i];
         }
         queryID[QUERY_OFFSET_BYTES] = (char)0x00;
         _messageIdentifier = (uint16_t)strtol(queryID, &ptr, 16);

         // extract the embedded CRC32 value from the received SCOMP QUERY
         for (int i = 0; i < CRC32_CHARACTERS; i++)
         {
            queryCRC32[i] = buffer[i + payloadLength + QUERY_HEADER_BYTES];
         }
         queryCRC32[CRC32_CHARACTERS] = (char)0x00;

         // now we extract the CRC32 from the [*queryBuffer]
         crc.update(queryPayload, payloadLength + QUERY_HEADER_BYTES);
         crc.finalizeAsArray(EOR);

         bool crcIsConfirmed = true;
         index = 0;
         for (size_t i = 0; i < FOOTER_BYTES; i++)
         {
            valueCRC32[0] = queryCRC32[index++];
            valueCRC32[1] = queryCRC32[index++];
            valueCRC32[2] = '\n';
            uint16_t checkValue = (uint16_t)strtol(valueCRC32, &ptr, 16);
            if ((uint16_t)EOR[i] != checkValue)
            {
               crcIsConfirmed = false;
               break;
            }
         }

#ifdef SERIAL_RECEIVE_DEBUG
         READER_DEBUGPRINT.println(' ');
         READER_DEBUGPRINT.print(">> ID: [");
         READER_DEBUGPRINT.print(_messageIdentifier);
         READER_DEBUGPRINT.print("], query body: [");
         READER_DEBUGPRINT.print(queryBody);
         READER_DEBUGPRINT.print("], CRC32: [");
         READER_DEBUGPRINT.print(queryCRC32);
         READER_DEBUGPRINT.print("]  REQUIRED CRC: [");
         for (int i = 0; i < 4; i++)
         {
            READER_DEBUGPRINT.print(EOR[i]);
            READER_DEBUGPRINT.print(",");
         }
         if (crcIsConfirmed)
         {
            READER_DEBUGPRINT.println(" - VALID]");
         }
         else
         {
            READER_DEBUGPRINT.println(" - INVALID]");
         }
#endif

         if (crcIsConfirmed)
         {
            _queryReceived = true;
            _SerialBuffer.clear();
            for (size_t i = 0; i < payloadLength; i++)
            {
               _SerialBuffer.add(queryBody[i]);
            }
         }
         else
         {
            _invalidQueryReceived = true;
            _messageIdentifier = 0x0000;
            _SerialBuffer.clear();
         }
      }
      else
      {
#ifdef SERIAL_RECEIVE_DEBUG
         READER_DEBUGPRINT.print(".");
#endif
      }

      delete[] queryID;
      delete[] queryBody;
      delete[] valueCRC32;
      delete[] queryCRC32;
      delete[] queryPayload;
      delete[] payloadLengthString;
      delete[] buffer;
      crc.reset();

      // enable the reader again
      _blockReader = false;
   }
}

///
/// @brief Process any received query from the controller
/// @brief RUN FROM BLUETOOTH THREAD()
///
void ProcessReceivedQueries()
{

   // if an invalid QUERY was received, then we need to let the client know
   if (_invalidQueryReceived & (_messageIdentifier == 0x000))
   {
      PublishResponseToBluetooth(scomp_response_error, sizeof(scomp_response_error) - 1);
      _queryReceived = false;
      _invalidQueryReceived = false;
      _messageIdentifier = 0x0000;
      _SerialBuffer.clear();
   }

   // otherwise, return feedback and process the query
   else if (_queryReceived & (_messageIdentifier > 0x000))
   {
      // load the query string into its own string for post-processing
      char *queryBody = new char[_SerialBuffer.getLength() + 1];
      memset(queryBody, 0, _SerialBuffer.getLength() + 1);
      for (size_t i = 0; i < _SerialBuffer.getLength(); i++)
      {
         queryBody[i] = _SerialBuffer.get(i);
      }

      std::string search(queryBody);
      for (size_t i = 0; i < CMWR_PARAMETER_COUNT; i++)
      {
         if (search.find(scompCommands[i]) == 0)
         {
            _scomp_command = CMWR_Parameter(i + 1);
            break;
         }
      }

      // extract the the command payload
      if (_scomp_command != CMWR_Parameter::none)
      {
         size_t colon = search.find(':');
         char *subs = Substring(queryBody, colon + 2, _SerialBuffer.getLength() - (colon + 1));

         PublishResponseToBluetooth(scomp_response_ok, sizeof(scomp_response_ok) - 1);

         sensor.SetProperty(_scomp_command,subs);
         publish_tag();

         // process the query command
         switch (_scomp_command)
         {
         case CMWR_Parameter::angl:
            READER_DEBUGPRINT.print("ANGL ");
            break;
         case CMWR_Parameter::apvn:
            READER_DEBUGPRINT.print("APVN ");
            break;
         case CMWR_Parameter::btvn:
            READER_DEBUGPRINT.print("BTVN ");
            break;
         case CMWR_Parameter::cmst:
            READER_DEBUGPRINT.print("CMST ");
            break;
         case CMWR_Parameter::hwvn:
            READER_DEBUGPRINT.print("HWVN ");
            break;
         case CMWR_Parameter::imei:
            READER_DEBUGPRINT.print("IMEI ");
            break;
         case CMWR_Parameter::mfdt:
            READER_DEBUGPRINT.print("MFDT ");
            break;
         case CMWR_Parameter::modl:
            READER_DEBUGPRINT.print("MODL ");
            break;
         case CMWR_Parameter::pmvn:
            READER_DEBUGPRINT.print("PMVN ");
            break;
         case CMWR_Parameter::stst:
            READER_DEBUGPRINT.print("STST ");
            break;
         case CMWR_Parameter::stts:
            READER_DEBUGPRINT.print("STTS ");
            break;
         case CMWR_Parameter::tliv:
            READER_DEBUGPRINT.print("TLIV ");
            break;
         }

         READER_DEBUGPRINT.println(subs);
         free(subs);
      }

      delete[] queryBody;
      _queryReceived = false;
      _invalidQueryReceived = false;
      _messageIdentifier = 0x0000;
      _SerialBuffer.clear();
   }
}

///
/// @brief Streams the NDEF contents out over Bluetooth as a series of 16 byte packets
/// @param message_length raw NDEF message payload
/// @param headerdata NDEF meassage header with UUID
///
void PublishResponseToBluetooth(char *pagedata, size_t message_length)
{
   // make sure we don't have any NFC scanning overlaps here
   _readerBusy = true;

   // insert the message identified
   for (int i = 0; i < QUERY_OFFSET_BYTES; i++)
   {
      scomp_ok_response_header[i] = scomp_query_ID[i];
   }

   // set the SCOMP PROTOCOL total TAG payload length
   PAYLOAD_LEGTH[0] = (uint8_t)((message_length & 0xff00) >> 8);
   PAYLOAD_LEGTH[1] = (uint8_t)(message_length & 0x00ff);

   // insert the payload length into the SCOMP PROTOCOL RFID DATA HEADER
   const char *payloadLength = HexStr(PAYLOAD_LEGTH, LENGTH_BYTES, HEX_UPPER_CASE);
   for (int i = 0; i < (int)sizeof(payloadLength); i++)
   {
      scomp_ok_response_header[i + 5] = payloadLength[i];
   }

   // PUBLISH SCANNDY PROTOCOL HEADER TO BLUETOOTH
   txChar.writeValue(scomp_ok_response_header, false);
   crc.update(scomp_ok_response_header, RESPONSE_HEADER_BYTES);
   delayMicroseconds(BLOCK_WAIT_BLE);

   // PUBLISH THE PAYLOAD MESSAGE TO BLUETOOTH
   txChar.writeValue(pagedata, false);
   crc.update(pagedata, message_length);
   delayMicroseconds(BLOCK_WAIT_BLE);

   // publish the final CRC as an array of bytes
   crc.finalizeAsArray(EOR);
   const char *crcValue = HexStr(EOR, FOOTER_BYTES, HEX_UPPER_CASE);
   txChar.writeValue(crcValue, false);
   crc.reset();

   // release the blocker
   _readerBusy = false;
}
