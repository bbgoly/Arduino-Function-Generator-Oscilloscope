#include "ILI9341_UI.h"

// Converts touchscreen coordinates to pixel coordinates within the resolution of LCD
// using pre-calculated calibration values
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



Frame::Frame(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint16_t frameColor, uint16_t borderThickness, uint16_t borderColor)
  : _x(x), _y(y), _width(width), _height(height), _frameColor(frameColor), _borderThickness(borderThickness), _borderColor(borderColor), _radius(0), _borderType(CircularBorderType::All), _onRenderCallback(nullptr) {}

Frame::Frame(unsigned int x, unsigned int y, unsigned int width, unsigned int height, uint16_t frameColor, unsigned int radius, CircularBorderType borderType, uint16_t borderThickness, uint16_t borderColor)
  : _x(x), _y(y), _width(width), _height(height), _frameColor(frameColor), _borderThickness(borderThickness), _borderColor(borderColor), _radius(radius), _borderType(borderType), _onRenderCallback(nullptr) {}

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

void Frame::setBorderColor(uint16_t borderColor) {
    _borderColor = borderColor;
}

void Frame::setBorderThickness(uint16_t borderThickness) {
    _borderThickness = borderThickness;
}

void Frame::setOnRenderCallback(render_callback_t callback) {
    _onRenderCallback = callback;
}

void Frame::render(Adafruit_ILI9341& lcd) {
    if (_radius > 0) {
        uint16_t boundaryOffset = min(5 * _radius, _width / 2); // probably rewrite to be more accurate with a boundaryOffsetY too
        switch (_borderType) {
            case Top:
                // didnt feel like wasting time figuring out how to add a circular border to this case, especially since I don't even use this case
                // will add in future if I decide to use ILI9341 for other projects (aka if I decide to expand on this UI library)
                lcd.fillRoundRect(_x, _y, _width, boundaryOffset, _radius, _frameColor);
                lcd.fillRect(_x, _y + boundaryOffset / 2, _width, _height - boundaryOffset / 2, _frameColor);
                break;
            case Bottom:
                if (_borderThickness > 0) {
                    lcd.fillRoundRect(_x - _borderThickness, _y + _height - boundaryOffset / 2, _width + 2*_borderThickness, boundaryOffset / 2 + _borderThickness, _radius, _borderColor);
                    lcd.fillRect(_x - _borderThickness, _y - _borderThickness, _width + 2*_borderThickness, boundaryOffset / 2 + 2*_borderThickness, _borderColor);
                }
                lcd.fillRoundRect(_x, _y, _width, _height, _radius, _frameColor);
                lcd.fillRect(_x, _y, _width, boundaryOffset / 2, _frameColor);
                break;
            case Left:
                if (_borderThickness > 0) {
                    lcd.fillRoundRect(_x - _borderThickness, _y - _borderThickness, boundaryOffset, _height + 2*_borderThickness, _radius, _borderColor);
                    lcd.fillRect(_x - _borderThickness + boundaryOffset / 2, _y - _borderThickness, _width - boundaryOffset / 2 + _borderThickness, _height + 2*_borderThickness, _borderColor);
                }
                // test stuff - creates wireframe representation of how everything would look, to check if the math is correct
                // lcd.drawRect(_x, _y, _width, _height, ILI9341_WHITE);
                // lcd.drawRoundRect(_x, _y, boundaryOffset, _height, _radius, _frameColor);
                // lcd.drawRect(_x + boundaryOffset / 2, _y, _width - boundaryOffset / 2 - _borderThickness, _height, _frameColor);
                lcd.fillRoundRect(_x, _y, boundaryOffset, _height, _radius, _frameColor);
                lcd.fillRect(_x + boundaryOffset / 2, _y, _width - boundaryOffset / 2 - _borderThickness, _height, _frameColor);
                break;
            case Right:
                if (_borderThickness > 0) {
                    lcd.fillRoundRect(_x + _width - boundaryOffset + _borderThickness, _y - _borderThickness, boundaryOffset, _height + 2*_borderThickness, _radius, _borderColor);
                    lcd.fillRect(_x - _borderThickness, _y - _borderThickness, _width - boundaryOffset / 2 + _borderThickness, _height + 2*_borderThickness, _borderColor);
                }
                lcd.fillRoundRect(_x + _width - boundaryOffset, _y, boundaryOffset, _height, _radius, _frameColor);
                lcd.fillRect(_x, _y, _width - boundaryOffset / 2, _height, _frameColor);
                break;
            case All:
                if (_borderThickness > 0) {
                    lcd.fillRoundRect(_x - _borderThickness, _y - _borderThickness, _width + 2*_borderThickness, _height + 2*_borderThickness, _radius, _borderColor);
                }
                lcd.fillRoundRect(_x, _y, _width, _height, _radius, _frameColor);
                break;
        }
    } else {
        if (_borderThickness > 0) {
            lcd.fillRect(_x - _borderThickness, _y - _borderThickness, _width + 2*_borderThickness, _height + 2*_borderThickness, _borderColor);
        }
        lcd.fillRect(_x, _y, _width, _height, _frameColor);
    }
    
    // if current component has onRender event binded to a callback
    if (_onRenderCallback != nullptr) {
        _onRenderCallback();
    }
    _isRendered = true;
}

void Frame::unrender(Adafruit_ILI9341& lcd, uint16_t backgroundColor) {
    // dont need to care for radius
    if (_isRendered) {
        _isRendered = false;
        lcd.fillRect(_x - _borderThickness, _y - _borderThickness, _width + 2*_borderThickness, _height + 2*_borderThickness, backgroundColor);
    }
}



TextField::TextField(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char text[100], uint16_t frameColor, uint16_t textColor, TextAlignment align, uint16_t borderThickness, uint16_t borderColor)
  : Frame(x, y, width, height, frameColor, borderThickness, borderColor), _textColor(textColor), _align(align), _textPaddingX(5), _textPaddingY(0), _textChangedCallback(nullptr) {
    strcpy(_text, text);
  }

TextField::TextField(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char text[100], uint16_t frameColor, unsigned int radius, CircularBorderType borderType, uint16_t textColor, TextAlignment align, uint16_t borderThickness, uint16_t borderColor)
  : Frame(x, y, width, height, frameColor, radius, borderType, borderThickness, borderColor), _textColor(textColor), _align(align), _textPaddingX(5), _textPaddingY(0), _textChangedCallback(nullptr) {
    strcpy(_text, text);
  }

void TextField::setText(Adafruit_ILI9341& lcd, const char text[100], uint16_t textColor) {
    if (textColor > 0) {
        _textColor = textColor;
    }

    if (_isRendered) {
        // Method may also be called to display different text color, not just different text
        if (strcmp(_text, text) != 0) {
            // Overwrite old text
            this->setAlignedCursor(lcd);
            lcd.setTextColor(_frameColor);
            lcd.println(_text);

            // if text component has textChanged event binded to a callback
            if (_textChangedCallback != nullptr) {
                _textChangedCallback(_text);
            }
        }

        this->setAlignedCursor(lcd);
        lcd.setTextColor(_textColor);
        lcd.println(text);
    }
    // Skip all the graphics displaying and just update internal text value since button isn't rendered
    strcpy(_text, text);
}

// use textColor = 0 to avoid setting color if not needed (see TextField::setText)
void TextField::setFormattedText(Adafruit_ILI9341 &lcd, uint16_t size, uint16_t textColor, const char *text, ...) {
    char textBuffer[size];

    va_list args;
    va_start(args, text);
    vsnprintf(textBuffer, size, text, args);
    va_end(args);

    this->setText(lcd, textBuffer, textColor);
}

void TextField::setTextColor(Adafruit_ILI9341 &lcd, uint16_t textColor) {
    this->setText(lcd, _text, textColor);
}

void TextField::setTextAlignment(Adafruit_ILI9341& lcd, TextAlignment align) {
    if (_isRendered && align != _align) {
        // Overwrite text at previous alignment since alignment changed
        this->setAlignedCursor(lcd);
        lcd.setTextColor(_frameColor);
        lcd.println(_text);
        _align = align;

        this->setAlignedCursor(lcd);
        lcd.setTextColor(_textColor);
        lcd.println(_text);
    }
    _align = align;
}

void TextField::setTextPadding(Adafruit_ILI9341& lcd, int16_t textPaddingX, int16_t textPaddingY) {
    if (_isRendered) {
        this->setAlignedCursor(lcd);
        lcd.setTextColor(_frameColor);
        lcd.println(_text);

        _textPaddingX = textPaddingX;
        _textPaddingY = textPaddingY;

        this->setAlignedCursor(lcd);
        lcd.setTextColor(_textColor);
        lcd.println(_text);
    }
    _textPaddingX = textPaddingX;
    _textPaddingY = textPaddingY;
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

void TextField::setOnTextChangedCallback(text_callback_t callback) {
    _textChangedCallback = callback;
}

const char* TextField::getText() {
    return this->_text;
}

void TextField::render(Adafruit_ILI9341& lcd) {
    Frame::render(lcd);
    this->setText(lcd, _text);
}



TextButton::TextButton(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char text[100], uint16_t frameColor, button_callback_t buttonPressedCallback, uint16_t textColor, TextAlignment align, uint16_t borderThickness, uint16_t borderColor)
  : TextField(x, y, width, height, text, frameColor, textColor, align, borderThickness, borderColor), _buttonPressedCallback(buttonPressedCallback) {}

TextButton::TextButton(unsigned int x, unsigned int y, unsigned int width, unsigned int height, char text[100], uint16_t frameColor, button_callback_t buttonPressedCallback, unsigned int radius, CircularBorderType borderType, uint16_t textColor, TextAlignment align, uint16_t borderThickness, uint16_t borderColor)
  : TextField(x, y, width, height, text, frameColor, radius, borderType, textColor, align, borderThickness, borderColor), _buttonPressedCallback(buttonPressedCallback) {}

bool TextButton::isPressed(TS_Point touchPoint) {
    return _isRendered && (touchPoint.x >= _x && touchPoint.x <= (_x + _width) && touchPoint.y >= _y && touchPoint.y <= (_y + _height));
}

void TextButton::executeCallback(int sourceButtonIndex) {
    _buttonPressedCallback(sourceButtonIndex);
}



Frame* UIPage::getFrame(int frameIndex) {
    return &this->frames[frameIndex];
}

TextField* UIPage::getTextField(int textFieldIndex) {
    return &this->textFields[textFieldIndex];
}

TextButton* UIPage::getTextButton(int textButtonIndex) {
    return &this->textButtons[textButtonIndex];
}

void UIPage::setElements(Frame *frames, TextField *textFields, TextButton *textButtons, int framesSize, int textFieldsSize, int textButtonsSize) {
    this->framesSize = framesSize;
    this->textFieldsSize = textFieldsSize;
    this->textButtonsSize = textButtonsSize;

    this->frames = frames;
    this->textFields = textFields;
    this->textButtons = textButtons;
}

// In future could implement checks for UI element render priorities
// and render elements in opposite order of priority (highest priority will be
// rendered last to be ontop of other UI elements, so could use a stack)
void UIPage::renderAll(Adafruit_ILI9341& lcd) {
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

void UIPage::unrenderAll(Adafruit_ILI9341 &lcd, uint16_t backgroundColor) {
    for (int i = 0; i < framesSize; i++) {
        frames[i].unrender(lcd, backgroundColor);
    }

    for (int i = 0; i < textFieldsSize; i++) {
        textFields[i].unrender(lcd, backgroundColor);
    }

    for (int i = 0; i < textButtonsSize; i++) {
        textButtons[i].unrender(lcd, backgroundColor);
    }
}