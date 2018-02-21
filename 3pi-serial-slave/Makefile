DEVICE ?= atmega328p
MCU ?= atmega328p
AVRDUDE_DEVICE ?= m328p

CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) $(DEVICE_SPECIFIC_CFLAGS) -Os
CPP=avr-g++
OBJ2HEX=avr-objcopy 
LDFLAGS=-Wl,-gc-sections -Wl,-relax

PORT ?= /dev/ttyACM0
AVRDUDE=avrdude

TARGET=slave
SOURCE_FILES := $(wildcard *cpp)
OBJECT_FILES = slave.o OrangutanAnalog.o OrangutanDigital.o OrangutanLEDs.o 
OBJECT_FILES += OrangutanMotors.o OrangutanResources.o OrangutanSerial.o 
OBJECT_FILES += OrangutanTime.o Pololu3pi.o PololuWheelEncoders.o PololuQTRSensors.o

all: $(TARGET).hex

clean:
	rm -f *.o *.hex *.obj *.hex

%.o: %.cpp
	$(CPP) $(CFLAGS) $(LDFLAGS) -c $<

%.hex: %.obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

%.obj: $(OBJECT_FILES)
	$(CPP) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@

program: $(TARGET).hex
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c avrisp2 -P $(PORT) -U flash:w:$(TARGET).hex