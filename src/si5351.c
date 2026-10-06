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
//**                - PLLA feedback multisynth (MSNA, regs 26..33)
//**                - output multisynths MS0/MS1 (regs 42..49, 50..57) as
//**                  an even integer divider pair
//**                - phase registers 165/166 for the 90 degree I/Q offset
//**                - PLLA reset (reg 177 = 0x20) after every full retune
//**
//**************************************************************************

#include "main.h"
#include "si5351.h"

#if INCLUDE_I2C

#define BB0(x) ((uint8_t)x)             // Bust int32 into Bytes
#define BB1(x) ((uint8_t)(x>>8))
#define BB2(x) ((uint8_t)(x>>16))

static	uint32_t	xtal_f = 25000000;		// Reference crystal [Hz]
static	uint32_t	vco_f;					// PLLA (=VCO) frequency of the last full retune [Hz]
static	uint8_t		even_div;				// Output divider of the last full retune
static	uint8_t		si5351_addr = SI5351_ADDR;

/* ------------------------------------------------------------------------- */
/* ------------------ i2c layer (from the sketch, "Wire" replaced) --------- */
/* ------------------------------------------------------------------------- */

static void
i2cWrite(uint8_t reg, uint8_t val)		// write one register via i2c
{
	I2CSendStart();
	I2CSendByte((si5351_addr<<1)|0);		// send device address (write)
	if (I2CErrors == 0)
	{
		I2CSendByte(reg);					// send register address
		I2CSendByte(val);					// send data
	}
	I2CSendStop();
}

static void
i2cWriten(uint8_t reg, const uint8_t *vals, uint8_t vcnt)	// write register block
{
	I2CSendStart();
	I2CSendByte((si5351_addr<<1)|0);		// send device address (write)
	if (I2CErrors == 0)
	{
		I2CSendByte(reg);					// send register address
		while (vcnt--)
			I2CSendByte(*vals++);			// auto increment block write
	}
	I2CSendStop();
}

/* ------------------------------------------------------------------------- */
/* ------------------------------ driver API -------------------------------- */
/* ------------------------------------------------------------------------- */

void
si5351_set_xtal(uint32_t hz)
{
	// The driver math is 32 bit only: 128*msb must fit in a uint32_t, so
	// the crystal frequency may not exceed 2^32/128 = 33.55 MHz.
	if (hz < 1000000UL || hz > 33554432UL)
		hz = 25000000UL;
	xtal_f = hz;
}

void
si5351_init(uint8_t i2c_addr)
{
	si5351_addr = i2c_addr;
	vco_f = 0;								// no frequency set yet

	i2cWrite(149, 0);						// SpreadSpectrum off
	i2cWrite(3, 0xFF);						// Disable all CLK output drivers
	i2cWrite(183, SI5351_XTALPF<<6);		// Set 25mhz crystal load capacitance
	i2cWrite(177, 0x20);					// Reset PLLA  (0x80 resets PLLB)
	i2cWrite(16, 0x4d);	// CLK0: 4mA drive, MS0 as source for CLK0, PLLA as src for MS0, integer mode, CLK0 powerup
	i2cWrite(17, 0x4d);	// same for CLK1, except MS1 as source (same code for CLK_SRC)
	i2cWrite(18, 0x0f);	// CLK2 OFF
}

void
si5351_setfreq_iq(uint32_t fout)
{											// Set CLK0 and CLK1 to fout Hz, 90 deg I/Q
	uint32_t	msa, msb, msc, msxp1, msxp2, msxp3p2top, ms0p1;
	uint32_t	int_div;

	// Calculate output divisor, so that PLL freq is below 900M
	// divisor can be even integer in  4..126 range
	vco_f = 900000000;						// init every time function is called
	int_div = vco_f / fout;
	uint32_t even_div_32 = int_div - (int_div % 2);
	if (even_div_32 > 126) even_div_32 = 126;	// whatever it is, but clean signal not guaranteed below 4.7MHz
	if (even_div_32 < 4) even_div_32 = 4;		// keep the divider in the legal range
	even_div = (uint8_t)even_div_32;

	vco_f = fout * even_div_32;

	// calculate feedback multisynth (fractional divider) parameters, AN619 section 3.2
	msa = vco_f / xtal_f;					// Integer part of vco/fout
	msb = vco_f % xtal_f;					// Fractional part of vco/fout
	msc = xtal_f;							// Divide by 2 till fits in reg
	while (msc & 0xfff00000) {msb=msb>>1; msc=msc>>1;}

	msxp1 =(128*msa + 128*msb/msc - 512);
	msxp2 = 128*msb - 128*msb/msc * msc;	// msxp3 == msc;
	msxp3p2top = (((msc & 0x0F0000) <<4) | msxp2);		// 2 top nibbles
	uint8_t vals_fb[8] = { BB1(msc), BB0(msc), BB2(msxp1), BB1(msxp1),
		BB0(msxp1), BB2(msxp3p2top), BB1(msxp2), BB0(msxp2) };
	i2cWriten(26, vals_fb, 8);				// Write to 8 FB msynth regs. PLLA base register address is 26

	// Output dividers
	ms0p1 = (even_div_32<<7) - 512;
	uint8_t vals_od[8] = {0, 1, BB2(ms0p1), BB1(ms0p1), BB0(ms0p1), 0, 0, 0};
	i2cWriten(42, vals_od, 8);				// Write to 8 MS0 regs
	i2cWriten(50, vals_od, 8);				// Write to 8 MS1 regs
	i2cWrite(165, even_div);				// CLK0 phase
	i2cWrite(166, 0);						// CLK1 phase
	i2cWrite(177, 0x20);					// Reset PLLA
}

uint8_t
si5351_retune_ms_only(uint32_t fout)
{											// Glitch free retune, PLLA not touched
	uint32_t	num, den, div2, msxp1, msxp2, msxp3p2top;

	if (vco_f == 0 || fout == 0)
		return 0;							// no full retune done yet

	// Output ratio = vco_f / fout, written as (a + b/c) with 'a' even.
	// The smooth tune window of the firmware guarantees that the ratio is
	// within +/-2 of the original even divider.
	div2 = (2UL * vco_f) / fout;			// floor(2 * ratio), fits in 32 bit
	uint32_t a = div2 & ~1UL;				// 'a' = even floor(ratio)
	if (a < 4 || a > 126)
		return 0;							// divider change needed: full retune

	num = vco_f - (a>>1)*fout;				// remainder < 2*fout
	den = fout;
	while (den & 0xfff00000UL) {num=num>>1; den=den>>1;}
											// fit P3 (=den) in 20 bits

	msxp1 = ((a>>1)<<7) + 128*num/den - 512;
	msxp2 = 128*num - 128*num/den * den;
	msxp3p2top = (((den & 0x0F0000) <<4) | msxp2);		// 2 top nibbles
	uint8_t vals_od[8] = { BB1(den), BB0(den), BB2(msxp1), BB1(msxp1),
		BB0(msxp1), BB2(msxp3p2top), BB1(msxp2), BB0(msxp2) };
	i2cWriten(42, vals_od, 8);				// Write to 8 MS0 regs
	i2cWriten(50, vals_od, 8);				// Write to 8 MS1 regs

	// Fractional mode is needed to apply b/c (integer mode would ignore
	// it); the phase registers 165/166 stay effective in fractional mode.
	i2cWrite(16, 0x0d);						// CLK0: 4mA, PLLA, own multisynth, fractional mode
	i2cWrite(17, 0x0d);						// CLK1: same

	return 1;
}

void
si5351_output_en(uint8_t en_bits)
{											// Enable the CLK output drivers
	i2cWrite(3, 0xff & ~en_bits);
}

uint8_t
si5351_read_reg(uint8_t reg, uint8_t i2c_addr)
{
	uint8_t val = 0;

	I2CSendStart();
	I2CSendByte((i2c_addr<<1)|0);			// send device address (write)
	if (I2CErrors == 0)
	{
		I2CSendByte(reg);					// send register address
		I2CSendStart();						// repeated start
		I2CSendByte((i2c_addr<<1)|1);		// send device address (read)
		if (I2CErrors == 0)
		{
			val = I2CReceiveByte();
			I2CSend1();						// NAK, last byte
		}
	}
	I2CSendStop();

	return I2CErrors ? 0 : val;
}

#endif // INCLUDE_I2C
