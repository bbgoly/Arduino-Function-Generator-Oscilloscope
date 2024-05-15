# Arduino-Function-Generator-Oscilloscope
Final course project for EECS 2032 by Yousif Kndkji

Also my first embedded systems project, and my first C/C++ project in general.

This project makes use of three public libraries each under a MIT or BSD license, such as the [SdFat library](https://github.com/greiman/SdFat) by [Bill Greiman](https://github.com/greiman), [graphics libraries](https://github.com/adafruit/Adafruit-GFX-Library) for the [ILI9341](https://github.com/adafruit/Adafruit_ILI9341) by [Adafruit](https://github.com/adafruit), and the [XPT2046_Touchscreen library](https://github.com/PaulStoffregen/XPT2046_Touchscreen) by [Paul Stoffregen](https://github.com/PaulStoffregen). The rest of the code is my own work.

The SdFat library used is the same library used in the backend of Arduino's standard SD library, but a newer version of it to support exFAT MicroSDXC cards (which is what I am using).

As for the Adafruit graphics libraries, they are used to run the ILI9341 driver on the TFT LCD I have, and the XPT2046_Touchscreen library is used to interface with the XPT2046 touchscreen IC that comes on the breakout board of the TFT LCD I am using.

Most of the code has been explained with comments, to save readers from having to consult the datasheet as much to understand the code.
