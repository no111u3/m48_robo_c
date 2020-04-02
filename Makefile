MCU_TARGET     = atmega48

OPTIMIZE       = -O2

DEFS           =
LIBS           =

# You should not have to change anything below here.

CC             = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -Iincludes
override LDFLAGS       = -Wl,-Map,$^.map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
SIZE		   = avr-size
AVRDUDE		   = avrdude

%.elf: src/%.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o build/$@ $^ $(LIBS)
	$(SIZE) --format=avr --mcu=$(MCU_TARGET) build/$@

clean:
	rm -rf build/*.o build/*.elf 
	rm -rf src/*.lst src/*.map

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

%.hex: build/%.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< build/$@

%.srec: build/%.elf
	$(OBJCOPY) -j .text -j .data -O srec $< build/$@

%.bin: build/%.elf
	$(OBJCOPY) -j .text -j .data -O binary $< build/$@

%.write: build/%.hex
	$(AVRDUDE) -c ft232r -p m48 -b2400 -U flash:w:$<:a

# Rules for building the .eeprom rom images

%_eeprom.hex: build/%.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< build/$@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.srec: build/%.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< build/$@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.bin: build/%.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< build/$@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.write: build/%_eeprom.hex
	$(AVRDUDE) -c ft232r -p m48 -b2400 -U eeprom:w:$<:a
