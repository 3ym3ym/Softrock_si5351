//************************************************************************
//**
//** Project......: Firmware USB AVR Si5351 controler.
//**
//** Platform.....: ATtiny1624 @ 16 MHz
//**
//** Licence......: This software is freely available for non-commercial
//**                use - i.e. for research and experimentation only!
//**
//** Description..: Si5351A driver library, based on the algorithm by Jerry Gaffke KE7ER
//**
//**************************************************************************

#ifndef _SI5351_H_
#define _SI5351_H_ 1

#include <stdint.h>

// I2C address of the Si5351 (typical). The actual address used is
// R.ChipCrtlData (settable by USB command 0x41); this is only the default
// used by si5351_init().
#define SI5351_ADDR             0x60

// 1:6pf  2:8pf  3:10pf crystal load capacitance (register 183)
#define SI5351_XTALPF           2

// Output enable bits for si5351_output_en(): CLK0, CLK1, CLK2
#define SI5351_CLK0             0x01
#define SI5351_CLK1             0x02
#define SI5351_CLK2             0x04

// One time power-up initialisation of the Si5351A. Must be called before
// any frequency is set. Spread spectrum off, outputs disabled, crystal
// load capacitance set, PLLA reset, CLK0/CLK1 configured (4 mA, PLLA,
// own multisynth, integer mode) and CLK2 powered down.
void si5351_init(uint8_t i2c_addr);

// Set CLK0 and CLK1 both to fout Hz in quadrature (90 degrees, CLK0 leads).
// Full retune: PLLA feedback multisynth, output multisynths, phase
// registers and a PLLA reset. Based on si5351bx_setfreq_iq() from the
// Arduino sketch si5351_rp2040_cat.ino.
void si5351_setfreq_iq(uint32_t fout);

// Smooth (glitch free) retune: keeps the PLLA frequency untouched and only
// rewrites the output multisynth MS0/MS1 ratio (fractional mode, no PLL
// reset). Returns 0 (and retunes nothing) when fout can not be reached
// with the current PLLA frequency, e.g. after more than +/- 2 divider
// steps or before the first full retune.
uint8_t si5351_retune_ms_only(uint32_t fout);

// Enable the CLK output drivers: bit0 = CLK0, bit1 = CLK1, bit2 = CLK2.
// Based on si5351_output_en() from the Arduino sketch.
void si5351_output_en(uint8_t en_bits);

// Set the reference crystal frequency in Hz (the calibrated frequency of
// the 25 MHz crystal on the Si5351 module). Must be called before
// si5351_setfreq_iq(). Values outside 1..33.5 MHz are replaced by 25 MHz
// (the P1/P2 math of the driver is 32 bit only).
void si5351_set_xtal(uint32_t hz);

// Read one register from the Si5351 (used for the chip presence check).
// Returns 0 on an I2C error (I2CErrors set).
uint8_t si5351_read_reg(uint8_t reg, uint8_t i2c_addr);

#endif // _SI5351_H_
