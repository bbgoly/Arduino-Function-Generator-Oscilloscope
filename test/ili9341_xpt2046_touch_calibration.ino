#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <stdint.h>

#define TFT_DC 26  // Data/Command digital pin of ILI9341 TFT display
#define TFT_CS 28  // Chip select digital pin of ILI9341 TFT display
#define T_CS 40    // Chip select digital pin of the resitive touch screen used with the ILI9341

// dont need to define reset since I have it connected to SPI reset
Adafruit_ILI9341 lcd(TFT_CS, TFT_DC);
XPT2046_Touchscreen ts(T_CS);

float xCalM = 0.0, yCalM = 0.0;  // gradients
float xCalC = 0.0, yCalC = 0.0;  // y axis crossing points

int16_t blockX = 0, blockY = 0;
int8_t blockWidth = 20, blockHeight = 20;

void setup() {
    Serial.begin(9600);
    Serial.println("ILI9341 testing and two-point calibration");

    lcd.begin();
    lcd.setRotation(1);

    ts.begin();
    ts.setRotation(1);

    calibrateTS();
}

void calibrateTS() {
    int16_t x1, y1, x2, y2;
    lcd.fillScreen(ILI9341_BLACK);

    while (ts.touched());  // Prevents false readings

    // Setup two points, whose pixel dimensions we know, to measure the touch screen resistive values (based on https://youtu.be/QZbgPHvLfGw)
    // Xp = mXt + c

    // First point
    lcd.drawFastHLine(10, 20, 20, ILI9341_RED);
    lcd.drawFastVLine(20, 10, 20, ILI9341_RED);

    while (!ts.touched());  // Wait for next touch, the next touch will be used for calibration
    delay(50);

    TS_Point p = ts.getPoint();
    x1 = p.x;
    y1 = p.y;

    lcd.drawFastHLine(10, 20, 20, ILI9341_BLACK);
    lcd.drawFastVLine(20, 10, 20, ILI9341_BLACK);

    delay(500);
    while (ts.touched());  // Wait for touch to end
    lcd.drawFastHLine(lcd.width() - 30, lcd.height() - 20, 20, ILI9341_RED);
    lcd.drawFastVLine(lcd.width() - 20, lcd.height() - 30, 20, ILI9341_RED);

    while (!ts.touched());
    delay(50);

    p = ts.getPoint();
    x2 = p.x;
    y2 = p.y;

    lcd.drawFastHLine(lcd.width() - 30, lcd.height() - 20, 20, ILI9341_BLACK);
    lcd.drawFastVLine(lcd.width() - 20, lcd.height() - 30, 20, ILI9341_BLACK);

    int16_t xDist = lcd.width() - 40;
    int16_t yDist = lcd.height() - 40;

    // translate in form pos = m(x val) + c

    // x
    xCalM = (float)xDist / (float)(x2 - x1);
    xCalC = 20.0 - ((float)x1 * xCalM);

    // y
    yCalM = (float)yDist / (float)(y2 - y1);
    yCalC = 20.0 - ((float)y1 * yCalM);

    Serial.print("x1 = ");
    Serial.print(x1);
    Serial.print(", y1 = ");
    Serial.println(y1);
    Serial.print("x2 = ");
    Serial.print(x2);
    Serial.print(", y2 = ");
    Serial.println(y2);
    Serial.print("xCalM = ");
    Serial.print(xCalM);
    Serial.print(", xCalC = ");
    Serial.println(xCalC);
    Serial.print("yCalM = ");
    Serial.print(yCalM);
    Serial.print(", yCalC = ");
    Serial.println(yCalC);
}

void convertScreenCoords(TS_Point* p) {
    int16_t xCoord = round(p->x * xCalM + xCalC);
    int16_t yCoord = round(p->y * yCalM + yCalC);

    if (xCoord < 0)
        xCoord = 0;
    else if (xCoord >= lcd.width())
        xCoord = lcd.width() - 1;

    if (yCoord < 0)
        yCoord = 0;
    else if (yCoord >= lcd.height())
        yCoord = lcd.height() - 1;
    p->x = xCoord;
    p->y = yCoord;
}

// test method to see if calibration values work (if the block constantly moves to where my finger is)
void moveBlock() {
    int16_t newBlockX, newBlockY;
    if (ts.touched()) {
        TS_Point p = ts.getPoint();
        convertScreenCoords(&p);

        newBlockX = p.x - (blockWidth / 2);
        newBlockY = p.y - (blockHeight / 2);

        if (newBlockX < 0)
            newBlockX = 0;
        else if (newBlockX >= lcd.width() - blockWidth)
            newBlockX = lcd.width() - 1 - blockWidth;

        if (newBlockY < 0)
            newBlockY = 0;
        else if (newBlockY >= lcd.height() - blockHeight)
            newBlockY = lcd.height() - 1 - blockHeight;
    }

    if (abs(newBlockX - blockX) > 2 || abs(newBlockY - blockY) > 2) {
        lcd.fillRect(blockX, blockY, blockWidth, blockHeight, ILI9341_BLACK);
        blockX = newBlockX;
        blockY = newBlockY;
        lcd.fillRect(blockX, blockY, blockWidth, blockHeight, ILI9341_RED);
    }
}

void loop() {
    if (ts.touched()) {
        moveBlock();
    }
}
