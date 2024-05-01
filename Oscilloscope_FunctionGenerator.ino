/* 
 * EECS 2032 Project Oscilloscope & Function Generator
 * By Yousif Kndkji
 *
 * This project makes use of three public libraries each under a MIT or BSD license, such as the SdFat library by Bill Greiman 
 * (https://github.com/greiman/SdFat-beta), the Adafruit graphics libraries for the ILI9341, and the XPT2046_Touchscreen library by
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
#define T_CS 34
#define SD_CS 22

#define RE_SW 12
#define RE_DT 11
#define RE_CLK 10

// Adafruit_ILI9341 has lcd.width() and lcd.height() methods that return these values, but unfortunately
// the library only sets its internal values of width and height after lcd.begin() is called,
// and I need those values before lcd.begin()
#define TFT_WIDTH 320
#define TFT_HEIGHT 240

#define LOOKUP_SIZE 512
#define SAMPLING_RATE 10000.0

/* Macros for testing functionalities */
#define TEST_DAC 0

Adafruit_ILI9341 lcd(TFT_CS, TFT_DC);
XPT2046_Touchscreen ts(T_CS);


int buttonCounter = 0;
// Frame frames[10];
// TextField textFields[10];

// TextButton textButtons[1] = {
//     TextButton((TFT_WIDTH - 100) / 2, (TFT_HEIGHT - 100) / 2, 50, 100, "click here", ILI9341_RED)
// };

UIContainer container;

void buttonPressed() {
    Serial.println("hello lol");
}

void initLCD() {
    lcd.begin();
    lcd.setRotation(1);

    ts.begin();
    ts.setRotation(1);

    lcd.setTextSize(3);
    TextField(0, 0, lcd.width(), lcd.height(), "Initializing...", ILI9341_BLACK).render(lcd);

    lcd.setTextSize(1);
    // textButtons[0].render(lcd);
    TextButton textButtons[1] = {
        TextButton((TFT_WIDTH - 100) / 2, (TFT_HEIGHT - 100) / 2, 50, 100, "click here", ILI9341_RED, buttonPressed)
    };

    TextField textFields[1] = {
        TextField((TFT_WIDTH - 100) / 2, (TFT_HEIGHT - 100) / 2, 50, 50, "click here", ILI9341_BLUE)
    };

    Frame frames[1] = {
        Frame((TFT_WIDTH - 100) / 2, (TFT_HEIGHT - 100) / 2, 50, 50, ILI9341_GREEN)
    };    

    container = UIContainer(frames, textFields, textButtons);
}

void displayOscilloscopeUI() {
    lcd.setTextSize(1);

    // TextButton buttons[1] = {
    //     TextButton((lcd.width() - 100) / 2, (lcd.height() - 100) / 2, 100, 100, "click here", ILI9341_RED)
    // };
    // buttons[0].render(lcd);

    // textButtons = buttons;
    Serial.println(container.getButton(0).getText());
    container.getButton(0).render(lcd);
}

void setupTC() {
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk(TC5_IRQn);

    TC_Configure(TC1, 2, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK5);
    // TC_SetRC(TC1, 2, )
}

void setupDACC() {
    pmc_enable_periph_clk(ID_DACC);
}

void setup() {
    Serial.begin(9600);

    pinMode(A0, INPUT);
    analogReadResolution(12);
    analogWriteResolution(12);

    initLCD();

    // ... init stuff, maybe put a fake delay if too fast lol
    // probably start DAC in sleep mode and have oscilloscope portion be active first?
    
    // ConfigureTC();
    // ConfigurePDCDMA();
    // ConfigureSD();
    // ConfigureDACC();
    // ConfigureADCC();

    analogWrite(DAC0, 0);

    displayOscilloscopeUI();
}

void loop() {
    if (ts.touched()) {
        TS_Point tPoint = convertTSCoords(lcd, ts.getPoint());
        if (container.getButton(buttonCounter).isPressed(tPoint)) {
            container.getButton(buttonCounter).executeCallback();
        }
        // if (textButtons[buttonCounter].isPressed(tPoint)) {
        //     textButtons[buttonCounter].setText(lcd, "pressed!");
        //     delayMicroseconds(1000000);
        //     textButtons[buttonCounter].setText(lcd, ":3");
        // }
        buttonCounter++;
    }
    // Serial.print(tPoint.x);
    // Serial.print(" ");
    // Serial.println(tPoint.y);
    if (buttonCounter == 1) {
        buttonCounter = 0;
    }
}
