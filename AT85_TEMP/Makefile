TARGET = AT85_TEMP

MCU     = attiny85
DUDEMCU = t85
CLOCK      = 8000000UL
PROGRAMMER = -P gpio -c gpio -b 10000
OBJECTS    = $(TARGET).o 1wire.o
FUSES      = -U lfuse:w:0xe2:m -U hfuse:w:0xdd:m -U efuse:w:0xff:m
# generated by http://www.engbedded.com/fusecalc/
 
AVRDUDE = avrdude $(PROGRAMMER) -p $(DUDEMCU)
COMPILE = /usr/bin/avr-gcc -Wall -std=c99 -Os -DF_CPU=$(CLOCK) -mmcu=$(MCU)
OBJ2HEX = /usr/bin/avr-objcopy
 
all:	$(TARGET).hex
 
.c.o:
	$(COMPILE) -c $< -o $@
 
.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
 
.c.s:
	$(COMPILE) -S $< -o $@
 
flash:	all
	$(AVRDUDE) -U flash:w:$(TARGET).hex:i
 
fuse:
	$(AVRDUDE) $(FUSES)
 
install: flash fuse
 
clean:
	rm -f *.hex *.obj *.o *.elf
 
$(TARGET).elf: $(OBJECTS)
	$(COMPILE) -o $(TARGET).elf $(OBJECTS)
 
$(TARGET).hex: $(TARGET).elf
	rm -f $(TARGET).hex
	$(OBJ2HEX) -j .text -j .data -O ihex $(TARGET).elf $(TARGET).hex
 
disasm:	$(TARGET).elf
	avr-objdump -d $(TARGET).elf
 
cpp:
	$(COMPILE) -E $(TARGET).c