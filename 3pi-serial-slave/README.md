# ANRG's Custom 3pi-serial-slave Program

This directory holds the customized slave program to be flashed on the 
Atmega328p in the 3pi base.

## Install Pre-reqs

    sudo apt-get install gcc-avr avr-libc binutils-avr avrdude

Clone https://github.com/pololu/libpololu-avr and read the top-level README in
the section "Installation using 'make install'" to compile and install the
pololu library. Example instructions:
    
    git clone https://github.com/pololu/libpololu-avr
    make show_prefix  #double check these are the right directories and they exist
    make
    sudo make install #permissions needed
