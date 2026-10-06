#******************************************************************************
#** Project......: Firmware USB AVR Si5351 controler.
#**
#** Platform.....: ATtiny1624 @ 16 MHz (internal oscillator)
#**
#** Description..: Makefile for the ATtiny1624/Si5351A port of the PE0FKO
#**                Softrock Si570 firmware V15.15.
#**
#** The AVR toolchain must support the ATtiny1624 (avr-gcc >= 9 with
#** avr-libc >= 2.0).  The tools are taken from PATH; set TOOLCHAIN_DIR to
#** the bin directory of a specific toolchain (relative or absolute path).
#**
#** All project paths are relative: run `make` inside this directory
#** (src/) or `make -C src` from the project top directory.
#******************************************************************************

# avr-gcc from PATH by default; TOOLCHAIN_DIR points at an explicit
# toolchain bin directory (relative or absolute).
TOOLCHAIN_DIR ?=
ifeq ($(TOOLCHAIN_DIR),)
CC      := avr-gcc
OBJCOPY := avr-objcopy
SIZE    := avr-size
else
CC      := $(TOOLCHAIN_DIR)/avr-gcc
OBJCOPY := $(TOOLCHAIN_DIR)/avr-objcopy
SIZE    := $(TOOLCHAIN_DIR)/avr-size
endif

AVRDUDE  ?= avrdude
PORT     ?= /dev/ttyUSB0
PROGRAMMER ?= serialupdi
# The default 115200 baud is marginal with this UPDI wiring; 57600 is stable.
AVRDUDE_BAUD ?= 57600
AVRDUDE_FLAGS = -c $(PROGRAMMER) -p t1624 -P $(PORT) -b $(AVRDUDE_BAUD)

MCU      = attiny1624
F_CPU    = 16000000
TARGET   = si5351avr

# Fuse values (avrdude 7 named aliases).  IMPORTANT: unused/reserved fuse
# bits default to 1 — never write 0 into them:
#   OSCCFG  = 0x7D : FREQSEL=01 (16 MHz); reserved bits kept 1 (default 0x7E)
#   SYSCFG0 = 0xF7 : default 0xF6 (RSTPINCFG=01 = UPDI mode!) | EESAVE.
#                    Writing 0x01 clears RSTPINCFG to 00 = GPIO mode and
#                    permanently disables UPDI (only HV entry can recover).
#   BODCFG  = 0x41 : BOD enabled in active mode, 2.6V threshold (default 0x00)
FUSE_OSCCFG  = 0x7D
FUSE_SYSCFG0 = 0xF7
FUSE_BODCFG  = 0x41

CSTANDARD = -std=gnu99

CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU)UL
CFLAGS += -Os -Wall $(CSTANDARD)
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -I. -Iusbdrv
CFLAGS += -Wa,-adhlns=$(<:.c=.lst),-gstabs
ASFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU)UL -I. -Iusbdrv -x assembler-with-cpp
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref

SRC = main.c DeviceSi5351.c si5351.c I2Ctwi.c \
      usbdrv/usbdrv.c usbdrv/usbdrvasm.S usbdrv/oddebug.c
OBJ = $(SRC:%.c=%.o)
OBJ := $(OBJ:%.S=%.o)


all: $(TARGET).hex $(TARGET).eep size

$(TARGET).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

%.o: %.S
	$(CC) -c $(ASFLAGS) $< -o $@

%.hex: %.elf
	$(OBJCOPY) -O ihex -R .eeprom -R .fuses -R .lock -R .signature $< $@

%.eep: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@

size: $(TARGET).elf
	$(SIZE) -C --mcu=$(MCU) $(TARGET).elf

fuse:
	$(AVRDUDE) $(AVRDUDE_FLAGS) \
	  -U osccfg:w:$(FUSE_OSCCFG):m \
	  -U syscfg0:w:$(FUSE_SYSCFG0):m \
	  -U bodcfg:w:$(FUSE_BODCFG):m

# NOTE: 'make flash' does NOT program the .eep image; the firmware writes
# its factory defaults to a blank EEPROM on first boot (same behaviour as
# the original firmware).
flash: all
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$(TARGET).hex:i

readback:
	$(AVRDUDE) $(AVRDUDE_FLAGS) \
	  -U flash:r:readback.hex:i -U osccfg:r:-:h -U syscfg0:r:-:h -U bodcfg:r:-:h

clean:
	rm -f $(OBJ) $(OBJ:.o=.lst)
	rm -f $(TARGET).hex $(TARGET).eep $(TARGET).elf $(TARGET).map readback.hex

.PHONY: all flash fuse size readback clean
