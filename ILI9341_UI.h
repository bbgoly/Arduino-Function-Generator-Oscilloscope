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

enum TextAlignment { LeftAlign, RightAlign, CenterAlign };
enum CircularBorderType { Top, Bottom, Left, Right, All };

class Frame {
public:
    Frame(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint16_t frameColor);
    Frame(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint16_t frameColor, unsigned int radius, CircularBorderType borderType);

    void setRadius(unsigned int radius, CircularBorderType borderType = CircularBorderType::All);
    void setCircularBorderType(CircularBorderType borderType);
    void setFrameColor(uint16_t frameColor);

    virtual void render(Adafruit_ILI9341& lcd);
    void unrender(Adafruit_ILI9341& lcd, uint16_t backgroundColor);

protected:
    uint16_t _frameColor;
    CircularBorderType _borderType;
	unsigned int _x, _y, _width, _height, _radius;
    
    bool _isRendered = false;
};

class TextField : public Frame {
public:
    TextField(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign);
    TextField(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, unsigned int radius, CircularBorderType borderType, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign);

    void setText(Adafruit_ILI9341& lcd, char* text, TextAlignment align = TextAlignment::CenterAlign);
    void setTextAlignment(Adafruit_ILI9341& lcd, TextAlignment align);
    void setTextColor(Adafruit_ILI9341 &lcd, int16_t textColor);

    void render(Adafruit_ILI9341& lcd);

protected:
	char* _text;
    uint16_t _textColor;
    TextAlignment _align;

    void setAlignedCursor(Adafruit_ILI9341& lcd);
};

class TextButton : public TextField {
public:
    TextButton(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign);
    TextButton(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, unsigned int radius, CircularBorderType borderType, uint16_t textColor = ILI9341_WHITE, TextAlignment align = TextAlignment::CenterAlign);
    
	bool isPressed(TS_Point touchPoint);

// private:
    // TODO: add callback when button is pressed probably
};

#endif // ILI9341_UI_H