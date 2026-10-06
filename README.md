# Softrock Compatible Firmware for ATtiny1624 and Si5351A

Port of the PE0FKO/DG8SAQ Softrock Si570 USB firmware **V15.15** from the
ATtiny85 + Si570 to the **ATtiny1624 + Si5351A**.  The USB operation and
the functionality presented to the host are kept the same as the original:
same VID/PID (`16C0:05DC`, "DG8SAQ-I2C"), same vendor control-transfer
command set (0x00–0x51), same fixed point formats, same EEPROM layout.

I am not an embedded developer; the port was heavily assisted by GLM-5.3/Zcode.

The Si5351A driver (`si5351.c`) implements the frequency control using builtin algorithm by Jerry Gaffke KE7ER (no external Si5351 library):
PLLA + MS0/MS1 as a quadrature I/Q pair on CLK0/CLK1, even
integer divider 4–126, VCO ≤ 900 MHz, variable MS0/MS1, phase registers 165/166, PLLA reset
after divider change only.

### USB command Frequency convention (SoftRock compatible)

SoftRock hosts (hamlib model 25009, PowerSDR, HDSDR, CFGSR, ...) send
**4x the LO** over the DG8SAQ protocol — the original hardware divided the
Si570 output by 4 (two flip-flops) to make quadrature.  The Si5351 makes
I/Q on chip (phase registers), so this firmware programs CLK0/CLK1 at
**host/4**: the requested frequency in, the requested frequency out.

* `R.Freq` (0x3A) and the 0x3F register image keep the **host value**
  (the "Si570 output" the host expects to see), the Si5351 runs at 1/4 of
  it.  Smooth-tune (±`R.SmoothTunePPM`) and the 0x17 band-filter
  crossovers compare against the host value, so the SoftRock defaults
  (16/32/64 MHz = 4x 4/8/16 MHz LO) apply unchanged.
* hamlib: `rigctl -m 25009 F 5000000` → 5 MHz on CLK0/CLK1; `rigctl f`
  reports 5000000.  For a raw 1x host, hamlib takes
  `-C multiplier=1` (backend parameter, default 4).
* Clean output (VCO ≥ 600 MHz) for host values ≥ ~19 MHz, i.e. LOs
  ≥ ~4.7 MHz; below that the divider clamps at 126 (works, not
  guaranteed clean) — as documented for the reference sketch.

## Hardware

| Pin | Function |
|-----|----------|
| PA0 | UPDI programming  |
| PA1 | user I/O #1 — band filter bit 0 / PTT output (cmd 0x50) |
| PA2 | user I/O #2 — band filter bit 1 |
| PA3 | CW key 1 input (0x51 bit 0x08) |
| PA4 | CW key 2 input (0x51 bit 0x10) |
| PB0 | I2C SCL → Si5351A (TWI0, fixed pin route, 200 kHz) |
| PB1 | I2C SDA → Si5351A (TWI0, fixed pin route) |
| PB2 | USB D+ (via 68Ohm resistor and 3.6 V zener to GMD, see the Softrock Enasemble RX II schematic) |
| PB3 | USB D− (same schematic note, there is also a 1.5k resistor to VBUS) |

Clock: internal 16 MHz oscillator (OSCCFG fuse 0x7D).  USB: V-USB 16 MHz
assembler core (`usbdrv/usbdrvasm16.inc`) via the VPORTB fast I/O aliases.
A naked trampoline ISR (`__vector_7`, `sbi 0x07,3`) clears the PB3 pin
interrupt flag before entering the V-USB handler, emulating the classic
INT0 auto-clear that the tinyAVR 0/1/2-series does not provide.

Si5351 reference crystal: 25.000 MHz nominal (`R.FreqXtal`, 8.24 format);
calibrate with USB command 0x33 like the Si570 xtal was calibrated.

## Building

The firmware sources and the Makefile live in `src/`.  Build from the top
directory with `make -C src` (or `cd src && make`).  An AVR toolchain with
ATtiny1624 support is required (avr-gcc ≥ 9 / avr-libc ≥ 2.0) — the tools
(`avr-gcc`, `avr-objcopy`, `avr-size`) are taken from `PATH`; point
`TOOLCHAIN_DIR` at a specific toolchain's bin directory to override, e.g.

```
make -C src TOOLCHAIN_DIR=/opt/avr-gcc/bin
```
I downloaded the toolchain from `https://github.com/modm-io/avr-gcc/releases`

Output: `src/si5351avr.hex` / `src/si5351avr.eep`.

```
make -C src            # build, show memory usage
make -C src flash      # program the flash via serialupdi on /dev/ttyUSB0
make -C src fuse       # OSCCFG=0x7D SYSCFG0=0xF7 BODCFG=0x41 (16 MHz, EESAVE, BOD 2.6V)
make -C src readback   # dump flash + fuses for verification
make -C src clean      # remove build products
```

avrdude runs at 57600 baud (`AVRDUDE_BAUD`): the default 115200 was
marginal with my  UPDI wiring.  The programmer I use is a 4-channel FT4232H
board ("UPDI_programmer"); only channel ttyUSB0 is connected to UPDI.
Other serial UPDI programmers should work fine. Other UPDI programmers require 
adjustments in Makefile.

`make flash` deliberately does not program the `.eep` image — the
firmware writes its factory defaults to a blank EEPROM on first boot
(same behaviour as the original firmware).  Note that a flash write
(chip erase) wipes the EEPROM unless the EESAVE fuse is programmed.

## Firmware behaviour vs. the original V15.15

Identical host protocol (`usbavrcmd.h`): all commands 0x00–0x51 with the
same request/value/index/payload formats and reply lengths; unknown
commands return a single 0xFF byte.  The `var_t` EEPROM structure is
byte-for-byte layout compatible (63 bytes).

The Si570 is replaced by the Si5351A, with the original Si570 register
math retained to maintain a **virtual Si570 register image**:

* **0x20** (write register) — writes into the virtual register image; a
  write to an RFREQ register (7–12/13–18) decodes the frequency and
  retunes the Si5351.  Si570 control registers (135/137) are ignored.
* **0x30** (set frequency by registers) — decodes the 6 Si570-format
  register bytes with `R.FreqXtal` (round trip is exact with the 25 MHz
  reference) and retunes the Si5351.
* **0x3F** (read registers) — returns the synthesized 6-byte image,
  kept in sync by `SetFreq()` (the Si5351 is assumed present).
* **0x44** (grade/DCO) — accepted, stored and echoed like before; only
  affects the divider choice of the virtual register image.
* **0x33/0x3D** (xtal) — now the *Si5351 reference crystal* (8.24),
  clamped to 1–33.5 MHz (32-bit driver math limit).

### Documented deviations

* 0x01/0x02/0x03/0x04 operate on **PORTA** (user I/O port) instead of
  the single PORTB of the ATtiny85; USB and I2C now live on PORTB and
  are protected by being on a different port.
* 0x15/0x16 return the PA1/PA2 bits (2 lsbs of PORTA).
* 0x50/0x51 CW-key bits: key 1 = PA3 (bit 0x08), key 2 = PA4
  (bit 0x10); the original used PB5/PB1 (0x22).  With the band filters
  enabled the reply is the fixed "keys open" value (0x18), as before.
* 0x42 temperature: raw 12-bit ADC of the internal sensor, shifted to
  the 10-bit numeric range of the original.
* Startup defaults: xtal 25 MHz (8.24 = 0x19000000), synth I2C address
  0x60, `Si570RFREQIndex` = 7 (no Si570 bank auto-detection anymore).
* No OSCCAL/SOF calibration — tinyAVR 0/1/2-series has no runtime
  oscillator calibration.  `R.RC_OSCCAL` (EEPROM offset 0) is unused.

### Known limits

* Clean output down to ~4.7 MHz of CLK frequency (VCO ≥ 600 MHz with
  divider ≤ 126).

## Files

All firmware sources, headers and the Makefile live in `src/` (paths in
the build are relative to that directory; `usbdrv/` is `src/usbdrv/`).

| File | Role |
|------|------|
| `src/main.c` | USB command dispatch, EEPROM defaults, USB interrupt trampoline |
| `src/main.h` | pin map, `var_t` layout, feature switches |
| `src/DeviceSi5351.c` | `SetFreq()`/`DeviceInit()`, virtual Si570 register image (C 64-bit math) |
| `src/si5351.c/.h` | Si5351A driver (PLLA + quadrature MS0/MS1) |
| `src/I2Ctwi.c` | TWI0 hardware I2C host driver (200 kHz) |
| `src/CalcVFO.c` | LO (freq − sub) × mul math |
| `src/FreqFromSi570.c` | virtual Si570 register → frequency decode |
| `src/Temperature.c` | internal temperature via ADC0 |
| `src/usbdrv/` | V-USB 20100715 (small portability patches for the AVRxt core) |
| `src/usbconfig.h` | V-USB config for PB2/PB3, 16 MHz, PORTB pin interrupt |
| `test_usb.py` | host-side regression suite (pyusb; 27 checks incl. factory reset) |

## Host side testing

Enumerates as `16c0:05dc` "DG8SAQ-I2C" with serial "PE0FKO-n".  Access via
libusb `usb_control_msg` with `USB_TYPE_VENDOR | USB_RECIP_DEVICE` — works
with the original DG8SAQ/PE0FKO host tools as well as hamlib
(`rigctl -m 25009`).  `python3 test_usb.py` runs the regression suite
(command coverage 0x00–0x51, factory reset 0x41=255, EEPROM persistence;
device must be enumerated on a direct root port, low-speed hubs can be
unreliable).
