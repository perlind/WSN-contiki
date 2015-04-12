#!/bin/bash

make bc hex TARGET=avr-RSS2

avrdude -p m128rfa1 -c avr109 -P /dev/tty.usbserial -b 38400 -e -U flash:w:bc.hex
