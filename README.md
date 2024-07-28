# Tiny Tapeout 05 tetris

This firmware provides the necessary memory interface to allow the Tetris project on TT05 to work.

See the Raspberry Pi [Getting Started with Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) guide for information on how to get set up to build the firmware.

Note you must build in Release mode.

This project overclocks the RP2040 to 300MHz and overvolts it to 1.2V (0.1V above stock) to ensure that is stable.  This may void your warranty, use at your own risk.

## Controls

On boot, the RP2040 waits for either one of the 5 input buttons to be pressed, or a connection to USB serial.

If a USB connection is made then the RP2040 drives the inputs, the controls are:
| Key | Button |
| --- | ------ |
| a   | Left   |
| d   | Right  |
| s   | Drop   |
| w or q | Rotate anticlockwise |
| e | Rotate clockwise |

If a button is pressed, then the RP2040 starts the game without taking control of the input pins.
