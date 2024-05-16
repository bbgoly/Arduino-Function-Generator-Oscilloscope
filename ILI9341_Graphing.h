#ifndef ILI9341_GRAPHING_H
#define ILI9341_GRAPHING_H

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <stdint.h>

// To be worked on in future

struct Graph {
    void createGraph(Adafruit_ILI9341& lcd, unsigned int x, unsigned int y, unsigned int width, unsigned int height);

    void addChannelView(uint8_t channelNumber);

    void drawGraph(Adafruit_ILI9341& lcd, uint8_t channelNumber, uint16_t data[], uint16_t startX, uint16_t startY, float waveSlopeX, float waveSlopeY);

    void clearGraph(Adafruit_ILI9341);
};

#endif  // ILI9341_GRAPHING_H