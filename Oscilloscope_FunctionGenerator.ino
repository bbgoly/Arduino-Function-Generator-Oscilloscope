/* 
 * EECS 2032 Project Oscilloscope & Function Generator
 * By Yousif Kndkji
 *
 * This project makes use of three public libraries each under a MIT or BSD license, such as the SdFat library by Bill Greiman 
 * (https://github.com/greiman/SdFat-beta), the Adafruit graphics libraries, and the XPT2046_Touchscreen library by
 * Paul Stoffregen (https://github.com/PaulStoffregen/XPT2046_Touchscreen). The rest of the code is my own work.
 *
 * The SdFat library used is the same library used in Arduino's standard SD library, but a newer version of it to support
 * exFAT MicroSDXC cards (which is what I am using).
 *
 * As for the Adafruit graphics libraries, they are used to run the ILI9341 driver on the TFT LCD I have, and the
 * XPT2046_Touchscreen library is used to interface with the XPT2046 touchscreen IC that comes with the TFT LCD I am using.

 * Uses SdFat library by Bill Greiman (https://github.com/greiman/SdFat-beta), a newer version of the same library used in 
 * Arduino's standard SD library to support exFAT MicroSDXC cards.
 *
 * Also uses Adafruit graphics libraries to run the ILI9341 driver on the TFT LCD I have.
 * I also use the XPT2046_Touchscreen library by Paul Stoffregen (https://github.com/PaulStoffregen/XPT2046_Touchscreen)
 * to communicate with the XPT2046 touchscreen IC that comes with the ILI9341 TFT LCD I have.
 */

#include <stdint.h>
#include <math.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include "circ_buffer.h"
#include "ILI9341_UI.h"

#define TFT_CS 28
#define TFT_DC 26
#define T_CS 40
#define SD_CS 53

#define RE_SW 12
#define RE_DT 11
#define RE_CLK 10

#define LOOKUP_SIZE 512
#define SAMPLING_RATE 100000.0

/* Macros for testing functionalities */
#define TEST_DAC 0


Adafruit_ILI9341 lcd(TFT_CS, TFT_DC);
XPT2046_Touchscreen ts(T_CS);

// Calibration values for touch screen derived from another program (under gen folder)
const float xCalM = 0.09, xCalC = -19.88, yCalM = 0.07, yCalC = -21.17;

// Frame frames[10];
// TextField textFields[10];
// TextButton textButtons[10]; 

void init_lcd() {
    lcd.begin();
    lcd.setRotation(1);

    ts.begin();
    ts.setRotation(1);

    lcd.setTextSize(3);
    TextField(0, 0, lcd.width(), lcd.height(), "Initializing...", ILI9341_BLACK).render(lcd);
}

void setup() {
    Serial.begin(9600);

    pinMode(A0, INPUT);
    analogReadResolution(12);

    analogWriteResolution(12);

    init_lcd();

    // ... init stuff, maybe put a fake delay if too fast lol
}

void loop() {
    
}
