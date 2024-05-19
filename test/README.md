# Test Folder
Contains some of the sketches and programs used to generate values for the main sketch or to test functionalities before they were fully implemented into the main sketch, all files can be ignored other than probably ili9341_xpt2046_touch_calibration.ino for the reason stated below (and because the main sketch contains the full, properly implemented and working functionalities of the other files in this folder, I mostly have the other files here to save them somewhere incase I need to reference them in some way)

Since the resistive touchscreen layer on my TFT LCD is not built into the ILI9341 LCD, and is actually just a seperate layer placed ontop of the LCD itself with a XPT2046 touch screen controller placed on the same breakout board that shares the SPI bus of the ILI9341, the coordinate systems of both components are not the same. Therefore, ili9341_xpt2046_touch_calibration.ino was used to generate calibration values that are used to convert touchscreen coordinates into pixel coordinates within the resolution of the ILI9341 LCD. The calibration values that were obtained from this sketch are used in the main sketch, Oscilloscope_FunctionGenerator.ino