# STM32F303_Sine_Square_Generator
An bare metal STM32F303CB project that generates  a perfect 1KHz sine wave and 4 90 degree phase shifted square waves at also 1KHz

This project is build with Netbeans 8.2.
Programming the STM32F303CB is done with an ST-Link V2. In Netbeans click the run button to upload to the device.

Output of the sine wave is found on PA4 (the DAC channel 1 output)
The 4 square waves are found on the pins PB6, PB7, PB8, PB9

A 1602, 1604, 2002 or 2004 display can be connected to the pins:
PC14 for RS, PC15 for E, PA0 - PA3 for databus

A rotary encoder can be connected to PB0 and PB1

On the display the current phase difference between the sine wave and the square waves is shown. With the rotray encoder it can be moved in steps of 5 a degree.
