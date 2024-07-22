
#include "main.h"

//------------------------------------------------------------------------------------------------

volatile bool _blockTimerEvents = false;
volatile bool _bluetoothConnected = false;
volatile bool _blockBannerText = true;

/// @brief main IO application thread
rtos::Thread t1;

/// @brief secondary bluetooth thread
rtos::Thread t2;

/// @brief main thread timer event latch
volatile bool timerEvent = false;

/// @brief have we detected anything on the NFC eenergy line?
volatile bool tagDetectedEvent = false;

/// @brief NFC user memory
uint8_t tagMemory[ISO15693_USER_MEMORY];

/// @brief NFC tag object
SFE_ST25DV64KC_NDEF tag;

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
   // rxChar.setEventHandler(BLEWritten, onBLEWritten);

   // Let's tell all local devices about us.
   BLE.advertise();

   // set the default characteristics
   PublishHardwareDetails();
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
   SERIAL_USB.println(F("Connected"));
}

///
/// @brief A BLE device has just disconnected from our sensor - power off the connection LED
/// @param central BLE device
///
void onBLEDisconnected(BLEDevice central)
{
   LED_SetConnectedToBLE = LOW;
   SERIAL_USB.println(F("Disconnected"));
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
#pragma endregion

//------------------------------------------------------------------------------------------------

///
/// @brief configure application
///
void setup()
{
   SERIAL_USB.begin(SERIAL_BAUD_RATE);

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

      SERIAL_USB.println("Writing 0x0 to the first 256 bytes of user memory.");
      tag.writeEEPROM(0x0, tagMemory, ISO15693_USER_MEMORY);

      // Write the Type 5 CC File - starting at address zero
      SERIAL_USB.println(F("Writing CC_File"));
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

   tag.writeNDEFText("cmd:cmsd", &memLoc, true, true); // MB=1, ME=0

   // tag.writeNDEFText("imei:753022080001312", &memLoc, true, false);  // MB=1, ME=0
   // tag.writeNDEFText("modl:CMWR 23", &memLoc, false, false);         // MB=0, ME=0
   // tag.writeNDEFText("mfdt:010170", &memLoc, false, false);          // MB=0, ME=0
   // tag.writeNDEFText("hwvn:13", &memLoc, false, false);              // MB=0, ME=0
   // tag.writeNDEFText("btvn:1.13.0", &memLoc, false, false);          // MB=0, ME=0
   // tag.writeNDEFText("apvn:1.13.0", &memLoc, false, false);          // MB=0, ME=0
   // tag.writeNDEFText("pmvn:0.8.0", &memLoc, false, false);           // MB=0, ME=0
   // tag.writeNDEFText("angl:?", &memLoc, false, false);               // MB=0, ME=0
   // tag.writeNDEFText("cmst:cmsd", &memLoc, false, false);            // MB=0, ME=0
   // tag.writeNDEFText("tliv:3.47 2312041113", &memLoc, false, false); // MB=0, ME=0
   // tag.writeNDEFText("stst:OK 20", &memLoc, false, false);           // MB=0, ME=0
   // tag.writeNDEFText("stts:2401100506", &memLoc, false, true);       // MB=0, ME=1
}

volatile bool reader_detected = false;
volatile bool sensor_starting = false;
volatile uint8_t sensor_startup_count = 0x00;

///
/// @brief executes the PRIMARY thread
///
void main_thread()
{
   const char *hexNotation;

   while (true)
   {
      if (timerEvent)
      {

         SERIAL_USB.print(".");
         timerEvent = false;

         if (reader_detected & !sensor_starting)
         {
            SERIAL_USB.println('*');
            SERIAL_USB.println("READER DETECTED");

            // reset the reader detected flag
            reader_detected = false;

            // Read 16 bytes from EEPROM location 0x0
            uint8_t tagRead[ISO15693_USER_MEMORY] = {0};

            // Read the EEPROM: start at address 0x00, read contents into tagRead; read 16 bytes
            tag.readEEPROM(0x00, tagRead, ISO15693_USER_MEMORY);

            // is the sensor starting? we detect this by checking for the char array 'cmd:cmsd'
            sensor_starting = CheckNeedle(tagRead, COMMAND_CMSD, ISO15693_USER_MEMORY, 8);

            if (sensor_starting)
            {
               // convert the TAG header into HEX NOTATION strings (as opposed to raw binary)
               hexNotation = HexStr(tagRead, ISO15693_USER_MEMORY, HEX_UPPER_CASE);
               SERIAL_USB.println(hexNotation);
            }
         }

         //
         // if the sensor is starting we initiate a countdown of 'n' seconds
         // during which time we DO NOT continue to read the EEPROM contents
         //
         if (sensor_starting)
         {
            if (sensor_startup_count++ > 30)
            {
               sensor_starting = false;
               sensor_startup_count = 0x00;
            }
         }
      }

      if (tagDetectedEvent)
      {
         reader_detected = true;
         tagDetectedEvent = false;
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
            if (timerEvent & !_blockTimerEvents)
            {
               // ** Bluetooth_CheckModemForMessages();
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
   return (n == std::string::npos);
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