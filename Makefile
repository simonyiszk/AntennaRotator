#   ________ 
#  < Hello! >
#   -------- 
#         \   ^__^
#          \  (oo)\_______
#             (__)\       )\/\
#                 ||----w |
#                 ||     ||
########################################

.DEFAULT_GOAL := rotator.elf

VERSION = $(shell git rev-parse HEAD)$(shell git diff --quiet || echo -n '-dirty')

CC = avr-gcc
CXX = avr-gcc
SIZE = avr-size

CPPFLAGS = -Wall -Wextra -fstack-usage -mmcu=atmega8 -O3 -g -Wno-array-bounds -Dmacro____commit=\"$(VERSION)\"
CXXFLAGS = -std=c++20 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -isystem avr-libstdcpp/include -fno-threadsafe-statics -isystem libavrpp/include

OBJECTS = $(addsuffix .o,$(basename $(wildcard *.cpp)))

rotator.elf: $(OBJECTS)
	avr-gcc -mmcu=atmega8 -o $@ $^

all: rotator.elf

clean:
	rm -rf *.o *.elf *.su

size: rotator.elf
	avr-size --mcu=atmega8 -C $<

refurbish: rotator.elf
	avrdude -p m8 -c stk500v2 -U flash:w:$<:e -P /dev/ttyUSB0 -B100000 -U lfuse:w:$<:e -U hfuse:w:$<:e -U eeprom:w:$<:e

program: rotator.elf
	avrdude -p m8 -c stk500v2 -U flash:w:$<:e -P /dev/ttyUSB0 -B100000

reset:
	avrdude -p m8 -c stk500v2 -P /dev/ttyUSB0 -B100000 

.PHONY: program refurbish size clean reset all
