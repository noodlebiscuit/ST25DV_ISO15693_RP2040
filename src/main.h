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

#include <ST25DV64KC//ST25DV64KC_Arduino_Library.h>

#pragma once
#include <stdint.h>

//------------------------------------------------------------------------------------------------

using namespace mbed;
using namespace std::chrono;

//------------------------------------------------------------------------------------------------

#define TICK_RATE_MS 1000ms // update rate for the mbed timer

//------------------------------------------------------------------------------------------------

#define SPI_SCK (13)             // SPI pin SCLK
#define SPI_MISO (12)            // SPI pin MISO
#define SPI_MOSI (11)            // SPI pin MOSI
#define SPI_SS (10)              // SPI pin SS
#define GPIO_PIN_2 2             // ATTENTION pin from the NoteCarrier
#define GPIO_PIN_3 3             // Acknowledge button pin (resets the LED and display)
#define GPIO_PIN_4 4             // GPIO pin
#define GPIO_PIN_5 5             // GPIO pin
#define GPIO_PIN_6 6             // GPIO pin
#define GPIO_PIN_7 7             // GPIO pin
#define GPIO_PIN_8 8             // GPIO pin
#define GPIO_PIN_9 9             // GPIO pin
#define SERIAL_BAUD_RATE 115200  // serial port baud rate
#define SERIAL_BUFFER_SIZE 400   // how many bytes are in thes serial buffer
#define SERIAL_USB Serial        // USB serial port (for non-debug use)
#define ISO15693_USER_MEMORY 256 // NFC tag memory

#define SDA_PIN 18
#define SCL_PIN 19
#define WireNFC MyWire
TwoWire MyWire(digitalPinToPinName(SDA_PIN), digitalPinToPinName(SCL_PIN));
// ST25DV st25dv(12, -1, &MyWire);

//------------------------------------------------------------------------------------------------

/// <summary>
/// MBED* Set direct write access to GPIO PIN 3 here
/// </summary>
// InterruptIn OnButtonPressInterupt(digitalPinToPinName(GPIO_PIN_9));

/// @brief MBED RTOS timer
Ticker timer;

void main_thread();
void secondary_thread();
void AtTime();