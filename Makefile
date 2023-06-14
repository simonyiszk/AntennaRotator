rotator.elf: $(wildcard *.cpp)
	avr-gcc -Wall -Wextra -g -O3 -isystem avr-libstdcpp/include -mmcu=atmega8 -std=c++20 -o $@ $^

.PHONY: program size clean reset

all: rotator.elf

clean:
	rm -rf *o *elf

size: rotator.elf
	avr-size --mcu=atmega8 -C $<

program: rotator.elf
	avrdude -p m8 -c stk500v2 -U flash:w:$<:e -P /dev/ttyUSB0 -B100000  -U lfuse:w:0x1c:m -U hfuse:w:0xd9:m

reset:
	avrdude -p m8 -c stk500v2 -P /dev/ttyUSB0 -B100000 

