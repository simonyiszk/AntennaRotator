target = flash
srcs   = main.c
defs   = -DF_CPU=8000000L

objs   = $(srcs:.c=.o)

all: $(target).hex

.PHONY: clean program fuse

fuse:
	avrdude -c usbtiny -p atmega8 -U lfuse:w:0x1c:m -U hfuse:w:0xd9:m
	
program: $(target).hex
	avrdude -c usbtiny -p atmega8 -U flash:w:$(target).hex

$(target).hex: $(target).elf
	avr-objcopy -j .text -j .data -O ihex $^ $@
	
$(target).elf: $(objs)
	avr-gcc -g -mmcu=atmega8 -o $@ $^
	
%.o: %.c
	avr-gcc -g -Os -mmcu=atmega8 -c $? -o $@ $(defs) --std=gnu99
	
clean:
	rm *.o *.hex *.elf
