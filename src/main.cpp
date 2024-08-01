#pragma region LICENSE and INFORMATION
/*
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
 * This firmware allows an  Arduino RP2040 and Adafruit  ST25DV16 combination to emulate the  NFC
 * functionality of a CMWR 23 sensor. It supports updates to any of the sensor's NDEF properties,
 * via a validated Bluetooth link. The protocol used to transfer property data is SCANNDY's SCOMP
 * which is detailed below:
 *
 *    Each SComP data packet is composed of three main parts:
 *
 *    1. the fixed size (10 octets) packet header
 *    2. variable length (up to 65535 octets) payload data
 *    3. the fixed size (8 octets) packet trailer
 *
 *    ssssTnnnnRp....pCCCCCCCC
 *    \__/|\__/|\____/\______/
 *      | |  | |    |        |
 *      | |  | |    |        8 hex digits CRC-32, covering header and payload
 *      | |  | |    |
 *      | |  | |    n ASCII characters of text payload data, see below
 *      | |  | |
 *      | |  | set to # for end-of-header flag
 *      | |  |
 *      | |  length n of payload data (four hex digits)
 *      | |
 *      | type of packet flag: 'Q' = request, 'R' = response
 *      |
 *      four hex digits message ID (sequence numnber), used to match related
 *      request/response pairs
 *
 *    Message IDs
 *    -----------
 *    The request generator should generate a new message ID for each request. The
 *    recommended implementation is a counter that is incremented for each generated
 *    request, but this approach is not mandatory. The receiver shall not make any
 *    assumptions about the algorithm generating the request message IDs. The sole
 *    purpose of the request message ID is to re-use it in the header of the
 *    corresponding response packet.
 *
 *    CRC-32
 *    ------
 *    The CRC-32 implementation shall use an initial value of 0xFFFFFFFF and a
 *    polynom of 0xEDB88320. The result shall be finalized by XORing it with
 *    0xFFFFFFFF.
 *
 *    Examples
 *    --------
 *    Set the IMEI property to 753022080001365:
 *    0068Q0014#imei:753022080001365b11a4060
 *
 *    Set the IMEI property to 753022080004406:
 *    0071Q0014#imei:753022080004406e163603d
 *
 *    Set the TLIV property to 3.47 2312041113:
 *    0071Q0014#tliv:3.47 2312041113ed020960
 *
 *    Set the CMST property to cmsd (commissioned):
 *    0071Q0009#cmst:cmsd0d61ac4f
 *
 *    Set the CMST property to ship (disabled/storage ready):
 *    0071Q0009#cmst:shipf144179d
 *
 * July 2024
 **/
#pragma endregion

//-------------------------------------------------------------------------------------------------

#include "main.h"

//-------------------------------------------------------------------------------------------------

#pragma region STRING MANAGEMENT AND SUPPORT
/// @brief set true when connected via bluetooth
volatile bool _bluetoothConnected = false;

/// @brief main thread timer event latch
volatile bool timerEvent = false;

/// @brief have we detected anything on the NFC eenergy line?
volatile bool tagDetectedEvent = false;

/// @brief  block access to the reader hardware?
volatile bool _blockReader = false;

/// @brief set true with the ST25DV16 detects an RFID energy pulse
volatile bool reader_detected = false;

/// @brief set true when a sensor has been commissioned and is starting
volatile bool sensor_starting = false;

/// @brief set true when a sensor has been switched to standby mode
volatile bool sensor_shutting_down = false;

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

/// @brief what nfc parameter type was issued by the connected client?
CMWR_Parameter _cmwr_parameter = CMWR_Parameter::none;

/// @brief what external command type was issued by the connected client?
CMWR_Command _cwwr_command = CMWR_Command::none;

/// @brief represent time in seconds from sensor receiving cmd:cmsd to it starting0071Q0014#tliv:3.47 2312041113ed020960
volatile uint8_t sensor_startup_count = 0x00;
#pragma endregion

//-------------------------------------------------------------------------------------------------

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
      READER_DEBUG_PRINT.println("starting BLE failed!");
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
   READER_DEBUG_PRINT.println(" ");
   READER_DEBUG_PRINT.println(F("Connected"));
}

///
/// @brief A BLE device has just disconnected from our sensor - power off the connection LED
/// @param central BLE device
///
void onBLEDisconnected(BLEDevice central)
{
   LED_SetConnectedToBLE = LOW;
   READER_DEBUG_PRINT.println(" ");
   READER_DEBUG_PRINT.println(F("Disconnected"));
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

//-------------------------------------------------------------------------------------------------

#pragma region FIRMWARE SETUP AND THREAD MANAGEMENT
///
/// @brief configure application
///
void setup()
{
   READER_DEBUG_PRINT.begin(SERIAL_BAUD_RATE);

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

      READER_DEBUG_PRINT.println("Writing 0x0 to the first 256 bytes of user memory.");
      tag.writeEEPROM(0x0, tagMemory, ISO15693_USER_MEMORY);

      // Write the Type 5 CC File - starting at address zero
      READER_DEBUG_PRINT.println(F("Writing CC_File"));
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
/// @brief executes the PRIMARY thread
///
void main_thread()
{
   while (true)
   {
      if (timerEvent & !_bluetoothConnected)
      {
         timerEvent = false;
         if (sensor_starting)
         {
            READER_DEBUG_PRINT.print("$");
         }
         else
         {
            READER_DEBUG_PRINT.print(".");
         }
         SimulateSensor();
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
               timerEvent = false;
               if (sensor_starting)
               {
                  READER_DEBUG_PRINT.print("$");
               }
               else
               {
                  READER_DEBUG_PRINT.print("+");
               }
               ProcessReceivedQueries();
               SimulateSensor();
            }
         }
      }
      else
      {
         _bluetoothConnected = false;
      }
   }
}

///
/// @brief MBED timer tick event *** APPLICATION CORE ***
///
void AtTime()
{
   timerEvent = true;
}

///
/// @brief TAG read event has been raised on GPIO-9
///
void TagDetectedInterrupt()
{
   tagDetectedEvent = true;
}

/// @brief not used
void loop() {}
#pragma endregion

//-------------------------------------------------------------------------------------------------

#pragma region STRING MANAGEMENT AND SUPPORT
///
/// @brief does a particular array of bytes exist within a buffer?
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
/// @brief what is the start position of a particular string within a buffer?
/// @param buffer source byte array to seach against
/// @param cmd reference byte array to search for within the buffer
/// @param buffer_length length of the buffer array
/// @param cmd_length length of the reference array
///
size_t GetNeedlePosition(uint8_t *buffer, uint8_t *cmd, size_t buffer_length, size_t cmd_length)
{
   std::string needle(cmd, cmd + cmd_length);
   std::string haystack(buffer, buffer + buffer_length); // or "+ sizeof Buffer"
   return haystack.find(needle);
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

uint8_t *Substring(uint8_t *sourceArray, int position, int length)
{
   uint8_t *pointer;
   int c;

   pointer = (uint8_t *)malloc(length + 1);

   if (pointer == NULL)
      exit(EXIT_FAILURE);

   for (c = 0; c < length; c++)
      *(pointer + c) = *((sourceArray + position - 1) + c);

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
#pragma endregion

//-------------------------------------------------------------------------------------------------

#pragma region READ AND WRITE TO NFC EEPROM - SIMULATE SENSOR
///
/// @brief simulate the actions of a real sensor. We support both activation and deactivation
///
void SimulateSensor()
{
   if (!_readerBusy)
   {
      if (reader_detected & !sensor_starting & !sensor_shutting_down)
      {
         READER_DEBUG_PRINT.println(" ");
         READER_DEBUG_PRINT.println("READER DETECTED");

         // reset the reader detected flag
         reader_detected = false;

         // Read 16 bytes from EEPROM location 0x0
         uint8_t tagRead[ISO15693_USER_MEMORY] = {0};

         // Read the EEPROM: start at address 0x00, read contents into tagRead; read 16 bytes
         tag.readEEPROM(0x00, tagRead, ISO15693_USER_MEMORY);

         // is the sensor starting? we detect this by checking for the char array 'cmd:cmsd'
         sensor_starting = CheckNeedle(tagRead, COMMAND_CMSD, ISO15693_USER_MEMORY, 8);

         // is the sensor instead shutting down? we detect this by checking for the char array 'cmd:ship'
         sensor_shutting_down = CheckNeedle(tagRead, COMMAND_SHIP, ISO15693_USER_MEMORY, 8);
      }

      //
      // if the sensor is starting we initiate a countdown of 'n' seconds
      // during which time we DO NOT continue to read the EEPROM contents
      //
      else if (sensor_starting & !sensor_shutting_down)
      {
         if (sensor_startup_count++ > SENSOR_STARTUP_TIME)
         {
            sensor.SetProperty(CMWR_Parameter::cmst, CMSD);
            publish_tag();

            READER_DEBUG_PRINT.println(" ");
            READER_DEBUG_PRINT.println("SENSOR NOW READY");

            sensor_startup_count = 0x00;
            sensor_starting = false;
            sensor_shutting_down = false;
            reader_detected = false;
            tagDetectedEvent = false;

            if (_bluetoothConnected)
            {
               PublishResponseToBluetooth(scomp_response_sensor_enabled,
                                          sizeof(scomp_response_sensor_enabled) - 1);
            }
         }
      }

      //
      // the same for when the sensor is shutting down, except now we only
      // apply a nominal shutdown period of 30 seconds
      //
      else if (!sensor_starting & sensor_shutting_down)
      {
         if (sensor_startup_count++ > SENSOR_SHUTDOWN_TIME)
         {
            sensor.SetProperty(CMWR_Parameter::cmst, SHIP);
            publish_tag();

            READER_DEBUG_PRINT.println(" ");
            READER_DEBUG_PRINT.println("SENSOR NOW DISABLED");

            sensor_startup_count = 0x00;
            sensor_starting = false;
            sensor_shutting_down = false;
            reader_detected = false;
            tagDetectedEvent = false;

            if (_bluetoothConnected)
            {
               PublishResponseToBluetooth(scomp_response_sensor_disabled,
                                          sizeof(scomp_response_sensor_disabled) - 1);
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
#pragma endregion

//-------------------------------------------------------------------------------------------------

#pragma region BLUETOOTH MESSAGE PROCESSING
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
         READER_DEBUG_PRINT.println(' ');
         READER_DEBUG_PRINT.print(">> ID: [");
         READER_DEBUG_PRINT.print(_messageIdentifier);
         READER_DEBUG_PRINT.print("], query body: [");
         READER_DEBUG_PRINT.print(queryBody);
         READER_DEBUG_PRINT.print("], CRC32: [");
         READER_DEBUG_PRINT.print(queryCRC32);
         READER_DEBUG_PRINT.print("]  REQUIRED CRC: [");
         for (int i = 0; i < 4; i++)
         {
            READER_DEBUG_PRINT.print(EOR[i]);
            READER_DEBUG_PRINT.print(",");
         }
         if (crcIsConfirmed)
         {
            READER_DEBUG_PRINT.println(" - VALID]");
         }
         else
         {
            READER_DEBUG_PRINT.println(" - INVALID]");
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
         READER_DEBUG_PRINT.print(".");
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
   // reset the command and parameter type flags
   _cmwr_parameter = CMWR_Parameter::none;
   _cwwr_command = CMWR_Command::none;

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

      // first we search through the NFC parameters list. One of these?
      for (size_t i = 0; i < CMWR_PARAMETER_COUNT; i++)
      {
         if (search.find(cmwr_nfc_parameter[i], 0) != std::string::npos)
         {
            _cmwr_parameter = CMWR_Parameter(i + 1);

            break;
         }
      }

      // ok, maybe it was a command. Let's have a look through them
      if (search.find(command_prefix) != std::string::npos)
      {
         size_t colon = search.find(':');
         char *substr = Substring(queryBody, colon + 2, _SerialBuffer.getLength() - (colon + 1));
         std::string search_cmd(substr);
         for (size_t i = 0; i < CMWR_COMMAND_COUNT; i++)
         {
            if (search_cmd.find(cmwr_command[i]) != std::string::npos)
            {
               _cwwr_command = CMWR_Command(i + 1);
               break;
            }
         }
         free(substr);
      }

      // if it's an nfc parameter, then extract the required payload to be written
      if ((_cmwr_parameter != CMWR_Parameter::none) & (_cwwr_command == CMWR_Command::none))
      {
         size_t colon = search.find(':');
         char *nfc_param = Substring(queryBody, colon + 2, _SerialBuffer.getLength() - (colon + 1));

         // let the calling service know that we processed the nfc property without issues
         PublishResponseToBluetooth(scomp_response_ok, sizeof(scomp_response_ok) - 1);

         // update the sensor property and public the new object to the EEPROM
         sensor.SetProperty(_cmwr_parameter, nfc_param);
         publish_tag();

         // debug print the extracted property payload
         READER_DEBUG_PRINT.println(nfc_param);

         free(nfc_param);
      }
      else if ((_cmwr_parameter == CMWR_Parameter::none) & (_cwwr_command != CMWR_Command::none))
      {
         std::vector<std::string> records;

         switch (_cwwr_command)
         {
         case CMWR_Command::none:
            READER_DEBUG_PRINT.println("none");
            break;
         case CMWR_Command::read:
            records = ReadContentsOfEEPROM();
            for (size_t i = 0; i < records.size(); i++)
            {
               READER_DEBUG_PRINT.println(records[i].c_str());
               PublishResponseToBluetooth(records[i].c_str(), records[i].length());
            }
            break;
         case CMWR_Command::reset:
            READER_DEBUG_PRINT.println("reset");
            break;
         }
      }

      delete[] queryBody;
      _queryReceived = false;
      _invalidQueryReceived = false;
      _messageIdentifier = 0x0000;
      _SerialBuffer.clear();
   }
}

///
/// @brief reads all NDEF records from the EEPROM and returns them as a list of strings
/// @return formatted list of strings
///
std::vector<std::string> ReadContentsOfEEPROM()
{
   std::vector<std::string> ndefRecords;
   ndefRecords.clear();

   // Read 16 bytes from EEPROM location 0x0
   uint8_t tagRead[ISO15693_USER_MEMORY] = {0};

   // Read the EEPROM: start at address 0x00, read contents into tagRead; read 16 bytes
   tag.readEEPROM(0x00, tagRead, ISO15693_USER_MEMORY);

   // now we're goinf to read the entire EEPROM contents and parse them into an array of strings
   int endPosition = 0;
   size_t startPosition = 0;
   size_t length = ISO15693_USER_MEMORY;
   uint8_t *apples = Substring(tagRead, endPosition, length);
   bool valid = true;

   while (valid)
   {
      valid = CheckNeedle(apples, NDEF_START, length, NDEF_SEARCH_BYTES);
      if (!valid)
      {
         break;
      }

      startPosition = GetNeedlePosition(apples, NDEF_START, length, NDEF_SEARCH_BYTES);
      char *ndef_string = Substring((char *)apples, startPosition + NDEF_HEADER_BYTES, (apples[startPosition - 1] - NDEF_FOOTER_BYTES));
      ndefRecords.push_back(ndef_string);
      endPosition = startPosition + apples[startPosition - 1];
      length = length - (apples[startPosition - 1] + startPosition);
      apples = Substring(apples, endPosition, length);
   }

   return ndefRecords;
}

///
/// @brief Streams the NDEF contents out over Bluetooth as a series of 16 byte packets
/// @param message_length raw NDEF message payload
/// @param headerdata NDEF meassage header with UUID
///
void PublishResponseToBluetooth(const char *pagedata, size_t message_length)
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
#pragma endregion

//-------------------------------------------------------------------------------------------------