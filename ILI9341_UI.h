/* Simple UI library for the ILI9341
 * By Yousif Kndkji
 *
 * Uses the Adafruit graphics libraries for the ILI9341 and the XPT2046_Touchscreen
 * library by Paul Stoffregen (https://github.com/PaulStoffregen/XPT2046_Touchscreen)
 */


#ifndef ILI9341_UI_H
#define ILI9341_UI_H

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <stdint.h>

enum TextAlignment { LeftAlign,
                     RightAlign,
                     CenterAlign };
enum CircularBorderType { Top,
                          Bottom,
                          Left,
                          Right,
                          All };

typedef void (*button_callback_t)(int);

// Calibration values for touch screen derived from another program (under gen folder)
const float xCalM = 0.09, xCalC = -19.88, yCalM = 0.07, yCalC = -21.17;

TS_Point convertTSCoords(Adafruit_ILI9341& lcd, TS_Point p);

class Frame {
  public:
    Frame(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint16_t frameColor);
    Frame(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint16_t frameColor, unsigned int radius, CircularBorderType borderType);
    Frame(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint16_t frameColor, bool drawBorder, uint16_t borderColor);
    Frame(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint16_t frameColor, bool drawBorder, uint16_t borderColor, unsigned int radius, CircularBorderType borderType);

    void setRadius(unsigned int radius, CircularBorderType borderType = CircularBorderType::All);
    void setCircularBorderType(CircularBorderType borderType);
    void setFrameColor(uint16_t frameColor);

    void render(Adafruit_ILI9341& lcd);
    void unrender(Adafruit_ILI9341& lcd, uint16_t backgroundColor);

  protected:
    uint16_t _frameColor, _borderColor;
    unsigned int _x, _y, _width, _height, _radius;
    CircularBorderType _borderType;

    bool _isRendered = false, _drawBorder = false;
};

class TextField : public Frame {
  public:
    TextField(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign);
    TextField(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, unsigned int radius, CircularBorderType borderType, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign);
    TextField(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, bool drawBorder, uint16_t borderColor, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign);
    TextField(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, bool drawBorder, uint16_t borderColor, unsigned int radius, CircularBorderType borderType, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign);

    void setText(Adafruit_ILI9341& lcd, char* text, TextAlignment align = TextAlignment::CenterAlign);
    void setTextAlignment(Adafruit_ILI9341& lcd, TextAlignment align);
    void setTextColor(uint16_t textColor);
    void setTextPadding(Adafruit_ILI9341& lcd, int16_t textPaddingX, int16_t textPaddingY);

    char* getText();

    void render(Adafruit_ILI9341& lcd);

  protected:
    char* _text;
    uint16_t _textColor;
    int16_t _textPaddingX, _textPaddingY;
    TextAlignment _align;

    void setAlignedCursor(Adafruit_ILI9341& lcd);
};

class TextButton : public TextField {
  public:
    TextButton(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, button_callback_t buttonPressedCallback, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign);
    TextButton(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, button_callback_t buttonPressedCallback, unsigned int radius, CircularBorderType borderType, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign);

    bool isPressed(TS_Point touchPoint);
    void executeCallback(int sourceButtonIndex);

  private:
    // Function pointer to callback function that is called when button is pressed
    button_callback_t _buttonPressedCallback;
};

struct UIPage {
    Frame* frames;
    TextField* textFields;
    TextButton* textButtons;
    
    int framesSize, textFieldsSize, textButtonsSize;
    
    void renderAll(Adafruit_ILI9341& lcd, int framesSize, int textFieldsSize, int textButtonsSize);
};


#endif  // ILI9341_UI_H