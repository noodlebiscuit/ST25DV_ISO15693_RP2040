
#include "main.h"

/// @brief main IO application thread
rtos::Thread t1;

/// @brief main IO application thread
rtos::Thread t2;

/// @brief main thread timer event latch
volatile bool timerEvent = false;

/// @brief have we detected anything on the NFC eenergy line?
volatile bool nfcEvent = false;

/// @brief NFC user memory
uint8_t tagMemory[ISO15693_USER_MEMORY];

/// @brief NFC tag object
SFE_ST25DV64KC_NDEF tag;

/// ============================================================================================

///
/// @brief configure application
///
void setup()
{
  SERIAL_USB.begin(SERIAL_BAUD_RATE);
  SERIAL_USB.println("Initializing...");

  MyWire.begin();
  if (tag.begin(MyWire))
  {
    SERIAL_USB.println("ST25 connected...");
  }

  // Clear the first TAG of user memory
  memset(tagMemory, 0, ISO15693_USER_MEMORY);

  SERIAL_USB.println("Writing 0x0 to the first 256 bytes of user memory.");
  tag.writeEEPROM(0x0, tagMemory, ISO15693_USER_MEMORY);

  // Write the Type 5 CC File - starting at address zero
  SERIAL_USB.println(F("Writing CC_File"));
  tag.writeCCFile8Byte();

  // Write two NDEF UTF-8 Text records
  uint16_t memLoc = tag.getCCFileLen();

  tag.writeNDEFText("imei:753055080000006", &memLoc, true, false);  // MB=1, ME=0
  tag.writeNDEFText("modl:CMWR 23", &memLoc, false, false);         // MB=0, ME=0
  tag.writeNDEFText("mfdt:010170", &memLoc, false, false);          // MB=0, ME=0
  tag.writeNDEFText("hwvn:13", &memLoc, false, false);              // MB=0, ME=0
  tag.writeNDEFText("btvn:1.13.0", &memLoc, false, false);          // MB=0, ME=0
  tag.writeNDEFText("apvn:1.13.0", &memLoc, false, false);          // MB=0, ME=0
  tag.writeNDEFText("pmvn:0.8.0", &memLoc, false, false);           // MB=0, ME=0
  tag.writeNDEFText("angl:?", &memLoc, false, false);               // MB=0, ME=0
  tag.writeNDEFText("cmst:cmsd", &memLoc, false, false);            // MB=0, ME=0
  tag.writeNDEFText("tliv:3.47 2312041113", &memLoc, false, false); // MB=0, ME=0
  tag.writeNDEFText("stst:OK 20", &memLoc, false, false);           // MB=0, ME=0
  tag.writeNDEFText("stts:2401100506", &memLoc, false, true);       // MB=0, ME=1

  // Read back the second NDEF UTF-8 Text record
  char theText[40];

  for (int i = 0; i < 12; i++)
  {
    if (tag.readNDEFText(theText, 40, i))
    {
      Serial.println(theText);
    }
  }

  // The GPO registers can only be changed during an open security session
  // uint8_t password[8] = {0x0};
  // tag.openI2CSession(password);
  // tag.setGPO1Bit(BIT_GPO1_FIELD_CHANGE_EN, true);
  // tag.setGPO1Bit(BIT_GPO1_RF_USER_EN, false);
  // tag.setGPO1Bit(BIT_GPO1_RF_ACTIVITY_EN, false);
  // tag.setGPO1Bit(BIT_GPO1_RF_INTERRUPT_EN, false);
  // tag.setGPO1Bit(BIT_GPO1_RF_PUT_MSG_EN, false);
  // tag.setGPO1Bit(BIT_GPO1_RF_GET_MSG_EN, false);
  // tag.setGPO1Bit(BIT_GPO1_RF_WRITE_EN, false);
  // tag.setGPO1Bit(BIT_GPO1_GPO_EN, true);

  // pinMode(GPIO_PIN_9, INPUT);
  // attachInterrupt(digitalPinToInterrupt(GPIO_PIN_9), acknowledgePressedInterrupt, CHANGE);

  t1.start(main_thread);

  t2.start(secondary_thread);

  // set the timeout value
  timer.attach(&AtTime, TICK_RATE_MS);
}

///
/// @brief all we do here is look to see if the card had detected a phone
///
void secondary_thread()
{
  while (true)
  {
    nfcEvent = tag.RFFieldDetected();
    delay(500);
  }
}

///
/// @brief executes the PRIMARY thread
///
void main_thread()
{
  while (true)
  {
    if (timerEvent)
    {
      SERIAL_USB.println("-------->>> SCHEDULED ACTION");
      timerEvent = false;
    }

    if (nfcEvent)
    {
      nfcEvent = false;
      SERIAL_USB.println("-------->>> NFC EVENT");
    }
  }
}

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
