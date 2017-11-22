# MatrixClock

[![License](https://img.shields.io/github/license/middelink/MatrixClock.svg)](https://github.com/middelink/MatrixClock/blob/master/LICENSE)

## TL;DR

MatrixClock is both a hardware and software project to create an extensible
40x8 dot matrix display with some sensors attached because, uhm. I could?

Here you will find the both the schematics, pcb layout and source code of my
MatrixClock project.

## Features

* Based on generaly available red 8x8 dot matrix displays with MAX7219. E.g. on [eBay](https://www.ebay.com/itm/8x8-3mm-5mm-Dot-Matrix-Display-Red-Full-Color-RGB-LED-MAX7219-DIY-Kit-f-Arduino/401374736108) (the red one, with the yellow headers).
* Full source code (arduino, platformio)
* Full schematics, PCB layout and gerbers (kicad)
* ESP-12E/F module for driving display and ntp time keeping.
* USB powered
* User defined button
* Light sensor: APDS9301 and/or BH1750FVI
* Environment sensor: BME280 or BME680
* Extensible, units can be cascaded (only one should have active components though)

## Libraries

* Adafruit's BME680 library has the nasty habbit of initializing the Wire
  library instead of leaving that to the calling program. This means they
  override our carefully set non-standard I2C pins. Needs to be commented
  out around line 90 in Adafruit\_BME680.cpp.
* Same library, Adafruit\_BME680.h, you want to comment out the include of
  Adafruit\_sensor.h. Not there, not used, not needed.

## Installation

The source is written on a Linux box and compiled/uploaded via the excellent
[PlatformIO](https://platformio.org) toolkit. The schematics and pcb layout is
created in [KiCad](https://kicad-pcb.org).

I will not bother to tell you how to compile/upload the code, modify the
schema, change the layout, create gerbers and wait patiently for the PCBs
to arrive before starting your SMD soldering adventure. Simply because
you should not do this if you not exhaustively researched all of the
just mentioned area's.

Good luck!
  Pauline
