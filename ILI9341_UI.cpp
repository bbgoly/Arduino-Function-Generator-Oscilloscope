#include "ILI9341_UI.h"

TS_Point convertTSCoords(Adafruit_ILI9341& lcd, TS_Point p) {
    int16_t xCoord = round(p.x * xCalM + xCalC);
    int16_t yCoord = round(p.y * yCalM + yCalC);

    if (xCoord < 0) 
        xCoord = 0;
    else if (xCoord >= lcd.width())
        xCoord = lcd.width() - 1;
    
    if (yCoord < 0)
        yCoord = 0;
    else if (yCoord >= lcd.height())
        yCoord = lcd.height() - 1;
    p.x = xCoord;
    p.y = yCoord;
    return p;
}



Frame::Frame(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint16_t frameColor)
    : _x(x), _y(y), _width(width), _height(height), _frameColor(frameColor), _radius(0), _borderType(CircularBorderType::All) { }

Frame::Frame(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint16_t frameColor, unsigned int radius, CircularBorderType borderType)
    : _x(x), _y(y), _width(width), _height(height), _frameColor(frameColor), _radius(radius), _borderType(borderType) { }

void Frame::setRadius(unsigned int radius, CircularBorderType borderType) {
    // _oldRadius = _radius;

    _radius = radius;
    _borderType = borderType;
}

void Frame::setCircularBorderType(CircularBorderType borderType) {
    _borderType = borderType;
}

void Frame::setFrameColor(uint16_t frameColor) {
    _frameColor = frameColor;
}

void Frame::render(Adafruit_ILI9341& lcd) {
    if (_radius > 0) {
        uint16_t boundaryOffset = min(5*_radius, _width / 2);
        switch (_borderType) {
            case Top:
                lcd.fillRoundRect(_x, _y, _width, boundaryOffset, _radius, _frameColor);
                lcd.fillRect(_x, _y + boundaryOffset / 2, _width, _height - boundaryOffset / 2, _frameColor);
                break;
            case Bottom:
                lcd.fillRoundRect(_x, _y + _height - boundaryOffset, _width, boundaryOffset, _radius, _frameColor);
                lcd.fillRect(_x, _y, _width, _height - boundaryOffset / 2, _frameColor);
                break;
            case Left:
                lcd.fillRoundRect(_x, _y, boundaryOffset, _height, _radius, _frameColor);
                lcd.fillRect(_x + boundaryOffset / 2, _y, _width - boundaryOffset / 2, _height, _frameColor);
                break;
            case Right:
                lcd.fillRoundRect(_x + _width - boundaryOffset, _y, boundaryOffset, _height, _radius, _frameColor);
                lcd.fillRect(_x, _y, _width - boundaryOffset / 2, _height, _frameColor);
                break;
            case All:
                lcd.fillRoundRect(_x, _y, _width, _height, _radius, _frameColor);
                break;
        }
    } else {
        lcd.fillRect(_x, _y, _width, _height, _frameColor);
    }
    _isRendered = true;
}

void Frame::unrender(Adafruit_ILI9341& lcd, uint16_t backgroundColor) {
    // dont need to care for radius
    lcd.fillRect(_x, _y, _width, _height, backgroundColor);
    _isRendered = false;
}



TextField::TextField(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, uint16_t textColor, TextAlignment align)
    : Frame(x, y, width, height, frameColor), _text(text), _textColor(textColor), _align(align), _textPaddingX(5), _textPaddingY(0) { }

TextField::TextField(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, unsigned int radius, CircularBorderType borderType, uint16_t textColor, TextAlignment align)
    : Frame(x, y, width, height, frameColor, radius, borderType), _text(text), _textColor(textColor), _align(align), _textPaddingX(5), _textPaddingY(0) { }

void TextField::setText(Adafruit_ILI9341& lcd, char* text, TextAlignment align) {
    if (_isRendered) {
        // Method may be called to display different text color, not just different text/alignment
        if (_text != text || _align != align) {
            // Overwrite old text
            this->setAlignedCursor(lcd);
            lcd.setTextColor(_frameColor);
            lcd.println(_text);
            _text = text;
            _align = align;
        }

        this->setAlignedCursor(lcd);
        lcd.setTextColor(_textColor);
        lcd.println(text);
    } else {
        // Skip all the graphics displaying and just update internal text value since button isn't rendered
        _text = text;
        _align = align;
    }
}

void TextField::setTextAlignment(Adafruit_ILI9341 &lcd, TextAlignment align) {
    if (_isRendered && align != _align) {
        // Overwrite text at previous alignment since alignment changed
        this->setAlignedCursor(lcd);
        lcd.setTextColor(_frameColor);
        lcd.println(_text);
        _align = align;

        this->setAlignedCursor(lcd);
        lcd.setTextColor(_textColor);
        lcd.println(_text);
    } else {
        _align = align;
    }
}

void TextField::setTextColor(uint16_t textColor) {
    _textColor = textColor;
    // this->setText(lcd, _text, _align);
}

void TextField::setTextPadding(Adafruit_ILI9341 &lcd, int16_t textPaddingX, int16_t textPaddingY) {
    if (_isRendered) {
        this->setAlignedCursor(lcd);
        lcd.setTextColor(_frameColor);
        lcd.println(_text);

        _textPaddingX = textPaddingX;
        _textPaddingY = textPaddingY;

        this->setAlignedCursor(lcd);
        lcd.setTextColor(_textColor);
        lcd.println(_text);
    } else {
        _textPaddingX = textPaddingX;
        _textPaddingY = textPaddingY;
    }
}

void TextField::setAlignedCursor(Adafruit_ILI9341& lcd) {
    uint16_t textWidth, textHeight;
    if (_align == TextAlignment::CenterAlign) {
        lcd.getTextBounds(_text, _x, _y, NULL, NULL, &textWidth, &textHeight);
    } else {
        lcd.getTextBounds(_text, _x + _textPaddingX, _y + _textPaddingY, NULL, NULL, &textWidth, &textHeight);
    }

    // To calculate center its xCursor = x_rect + (width_rect / 2) - (textWidth) / 2
    // and yCursor = y_rect + (height_rect / 2) - (textHeight / 2)
    // similar approaches for rest, no padding is applied for center alignment
    switch (_align) {
        case LeftAlign:
            lcd.setCursor(_x + _textPaddingX, _y + _textPaddingY + (_height - textHeight) / 2);
            break;
        case RightAlign:
            lcd.setCursor(_x - _textPaddingX + _width - textWidth, _y + _textPaddingY + (_height - textHeight) / 2);
            break;
        case CenterAlign:
            lcd.setCursor(_x + (_width - textWidth) / 2, _y + (_height - textHeight) / 2);
            break;
    }
}

char* TextField::getText() {
    return this->_text;
}

void TextField::render(Adafruit_ILI9341& lcd) {
    Frame::render(lcd);
    this->setText(lcd, _text, _align);
}



TextButton::TextButton(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, button_callback_t buttonPressedCallback, uint16_t textColor, TextAlignment align)
    : TextField(x, y, width, height, text, frameColor, textColor, align), _buttonPressedCallback(buttonPressedCallback) { }

TextButton::TextButton(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char* text, uint16_t frameColor, button_callback_t buttonPressedCallback, unsigned int radius, CircularBorderType borderType, uint16_t textColor, TextAlignment align)
    : TextField(x, y, width, height, text, frameColor, radius, borderType, textColor, align), _buttonPressedCallback(buttonPressedCallback) { }

bool TextButton::isPressed(TS_Point touchPoint) {
    return _isRendered && (touchPoint.x >= _x && touchPoint.x <= (_x + _width) && touchPoint.y >= _y && touchPoint.y <= (_y + _height));
}

void TextButton::executeCallback(int sourceButtonIndex) {
    _buttonPressedCallback(sourceButtonIndex);
}

// Ugly, but its one time and better than individually typing out a render for each UI element
void UIPage::renderAll(Adafruit_ILI9341 &lcd, int framesSize, int textFieldsSize, int textButtonsSize) {
    framesSize = framesSize;
    textFieldsSize = textFieldsSize;
    textButtonsSize = textButtonsSize;

    for (int i = 0; i < framesSize; i++) {
        frames[i].render(lcd);
    }

    for (int i = 0; i < textFieldsSize; i++) {
        textFields[i].render(lcd);
    }

    for (int i = 0; i < textButtonsSize; i++) {
        textButtons[i].render(lcd);
    }
}