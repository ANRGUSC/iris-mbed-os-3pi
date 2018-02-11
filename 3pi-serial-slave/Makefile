DEVICE ?= atmega328p
MCU ?= atmega328p
AVRDUDE_DEVICE ?= atmega328p

CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) $(DEVICE_SPECIFIC_CFLAGS) -Os
CC=avr-g++
OBJ2HEX=avr-objcopy 
LDFLAGS=-Wl,-gc-sections -Wl,-relax

PORT ?= /dev/ttyACM0
AVRDUDE=avrdude

TARGET=slave
SOURCE_FILES := $(wildcard *cpp)
OBJECT_FILES=main.o OrangutanAnalog.o OrangutanDigital.o OrangutanLEDs.o OrangutanMotors.o OrangutanResources.o OrangutanSerial.o OrangutanTime.o Pololu3pi.o PololuMagneticEncoders.o PololuQTRSensors.o

all: $(TARGET).hex

clean:
	rm -f *.o *.hex *.obj *.hex

%.o: %.c
	$(CC) $(CFLAGS) $(LDFLAGS) -c $<

%.o: %.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) -c $<

%.hex: %.obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

%.obj: $(OBJECT_FILES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@

program: $(TARGET).hex
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c avrisp2 -P $(PORT) -U flash:w:$(TARGET).hex