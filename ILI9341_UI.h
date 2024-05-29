/* Simple UI library for the ILI9341
 * By Yousif Kndkji
 *
 * Features callbacks on button click events, render events, and text changed events
 * Also has the ability to align text on the left, center, or right with padding 
 * if needed, and can draw all-sided or one-sided circular UI elements on any side
 * of a UI element with circular or non-circular borders as well. Also features a 
 * container struct to store and keep track of all created UI elements.
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
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// 16-bit colors in RGB565 format
#define ILI9341_UI_DARKGREEN 0x04E0
#define ILI9341_UI_DARKRED 0x8800
#define ILI9341_UI_DARKBLUE 0x0010
#define ILI9341_UI_LIGHTBLUE 0x34DF

// Probably add corner border types like TopRight, BottomLeft, etc.
enum TextAlignment { LeftAlign, RightAlign, CenterAlign };
enum CircularBorderType { Top, Bottom, Left, Right, All };

typedef void (*render_callback_t)();
typedef void (*button_callback_t)(int);
typedef void (*text_callback_t)(const char[100]);

// Calibration values for touch screen derived from another program (under test folder)
const float xCalM = 0.09, xCalC = -19.88, yCalM = 0.07, yCalC = -21.17;

TS_Point convertTSCoords(Adafruit_ILI9341& lcd, TS_Point p);

class Frame {
  public:
    Frame(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint16_t frameColor, uint16_t borderThickness = 0, uint16_t borderColor = ILI9341_BLACK);
    Frame(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint16_t frameColor, unsigned int radius, CircularBorderType borderType, uint16_t borderThickness = 0, uint16_t borderColor = ILI9341_BLACK);

    void setRadius(unsigned int radius, CircularBorderType borderType = CircularBorderType::All);
    void setCircularBorderType(CircularBorderType borderType);
    void setFrameColor(uint16_t frameColor);
    void setBorderColor(uint16_t borderColor);
    void setBorderThickness(uint16_t borderThickness);

    void setOnRenderCallback(render_callback_t callback);

    void render(Adafruit_ILI9341& lcd);
    void unrender(Adafruit_ILI9341& lcd, uint16_t backgroundColor);

  protected:
    uint16_t _frameColor, _borderColor, _borderThickness;
    unsigned int _x, _y, _width, _height, _radius;
    CircularBorderType _borderType;
    
    render_callback_t _onRenderCallback;

    bool _isRendered = false;
};

class TextField : public Frame {
  public:
    TextField(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char text[100], uint16_t frameColor, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign, uint16_t borderThickness = 0, uint16_t borderColor = ILI9341_BLACK);
    TextField(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char text[100], uint16_t frameColor, unsigned int radius, CircularBorderType borderType, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign, uint16_t borderThickness = 0, uint16_t borderColor = ILI9341_BLACK);

    void setText(Adafruit_ILI9341& lcd, const char text[100], uint16_t textColor = 0);
    void setFormattedText(Adafruit_ILI9341& lcd, uint16_t size, uint16_t textColor, const char text[100], ...);
    void setTextColor(Adafruit_ILI9341& lcd, uint16_t textColor);
    void setTextAlignment(Adafruit_ILI9341& lcd, TextAlignment align);
    void setTextPadding(Adafruit_ILI9341& lcd, int16_t textPaddingX, int16_t textPaddingY);

    void setOnTextChangedCallback(text_callback_t callback);

    const char* getText();
    const uint16_t getTextColor();

    void render(Adafruit_ILI9341& lcd);

  protected:
    char _text[100];
    uint16_t _textColor;
    int16_t _textPaddingX, _textPaddingY;
    TextAlignment _align;

    text_callback_t _textChangedCallback;

private:
    void setAlignedCursor(Adafruit_ILI9341& lcd);
};

class TextButton : public TextField {
  public:
    TextButton(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char text[100], uint16_t frameColor, button_callback_t buttonPressedCallback, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign, uint16_t borderThickness = 0, uint16_t borderColor = ILI9341_BLACK);
    TextButton(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char text[100], uint16_t frameColor, button_callback_t buttonPressedCallback, unsigned int radius, CircularBorderType borderType, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign, uint16_t borderThickness = 0, uint16_t borderColor = ILI9341_BLACK);

    bool isPressed(TS_Point touchPoint);
    void executeCallback(int sourceButtonIndex);

  private:
    // Function pointer to callback function that is called when button is pressed
    button_callback_t _buttonPressedCallback;
};

// Could probably use a static map and keyvalue pairs in the future to make implementing and managing/interacting with multiple ui elements easier
struct UIPage {
public:
    int framesSize, textFieldsSize, textButtonsSize;

    Frame* getFrame(int frameIndex);
    TextField* getTextField(int textFieldIndex);
    TextButton* getTextButton(int textButtonIndex);

    void setElements(Frame* frames, TextField* textFields, TextButton* textButtons, int framesSize, int textFieldsSize, int textButtonsSize);
    
    void renderAll(Adafruit_ILI9341& lcd);
    void unrenderAll(Adafruit_ILI9341& lcd, uint16_t backgroundColor);

private:
    Frame* frames;
    TextField* textFields;
    TextButton* textButtons;
};


#endif  // ILI9341_UI_H