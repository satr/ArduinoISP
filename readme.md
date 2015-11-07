# ArduinoISP
This is a sketch that allows your Arduino to work as a STK500v1 programmer.

See the excellent [Arduino as AVR ISP](http://arduino.cc/en/Tutorial/ArduinoISP)
tutorial on arduino.cc.

Please report issues on the [mega-isp](http://code.google.com/p/mega-isp)
site at code.google.com.

## Use a recent version!
Arduino IDE 1.00 was released 30 November 2011

Check out latest available Arduino IDE from: http://code.google.com/p/arduino/downloads/list

If your Arduino IDE version is <= Arduino-0023:

The ArduinoISP that ships as an example with the Arduino software is old.
You can download the ArduinoISP.ino here and replace the one you find
in ...\arduino-002x\examples\ArduinoISP.
This version is known to work with the avrdude that ships with Arduino-0023.
That is an older version of avrdude (5.04, latest is 5.11).

Features of this fork:
- Optional (regulated with "#define"): display for indication of mode, error, hearbit
-- Supports: LCD 5110
- SoftSerial to tranfer serial data to\from the client (the target board which is programmed); Baud: 19200 (same as for ISP)
-- When not in programming mode - Serial can be use to transfer data to and from the client
-- Automatically detects the request to program the client and blocks transfer data between ISP and the client
- Optional (regulated with "#define"): LED indication; pins are changed from original - to leave SPI pins free



