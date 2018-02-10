# ANRG's Custom 3pi-serial-slave Program

This directory holds the customized slave program to be flashed on the 
Atmega328p in the 3pi base.

## Install Pre-reqs

    sudo apt-get install gcc-avr avr-libc binutils-avr avrdude

In order to make the necessary modifications, we imported all the relevant
.cpp and .h files from the https://github.com/pololu/libpololu-avr directly into
this repository. By doing this, we are able to repurpose some GPIO lines for
additional sensors. For example, we removed the LCD to clear up pins for the
Pololu magnetic wheel encoders.