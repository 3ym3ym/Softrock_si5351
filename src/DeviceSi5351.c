//************************************************************************
//**
//** Project......: Firmware USB AVR Si5351 controler.
//**
//** Platform.....: ATtiny1624 @ 16 MHz
//**
//** Licence......: This software is freely available for non-commercial
//**                use - i.e. for research and experimentation only!
//**                Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
//**                Based on ObDev's AVR USB driver by Christian Starkjohann
//**
//** Programmer...: F.W. Krom, PE0FKO (original DeviceSi570.c),
//**                ported to the ATtiny1624/Si5351A By Misha Svoiski AF7KR.
//**
//** Description..: Frequency calculations and the programming algorithm
//**                for the Si5351A.  The Si570 register math of the
//**                original firmware is kept to maintain a "virtual Si570"
//**                register image (Si570_Data).  This image is what the
//**                register based USB commands (0x20 write, 0x30 set by
//**                register, 0x3f read) work on, so the host side software
//**                sees the same functionality as with a real Si570.
//**
//**                The Si5351A itself is controlled through si5351.c
//**                (library based on the algorithm by Jerry Gaffke KE7ER).
//**
//**************************************************************************

#include "main.h"
#include "si5351.h"

#if INCLUDE_SI570

static	uint16_t	Si570_N;				// Total division (N1 * HS_DIV)
static	uint8_t		Si570_N1;				// The slow divider
static	uint8_t		Si570_HS_DIV;			// The high speed divider
#if INCLUDE_SMOOTH
		uint32_t	FreqSmoothTune;			// The smooth tune center frequency
#endif

#include "CalcVFO.c"						// Include code is small size

// --------------------------------------------------------------
// Fixed point conversions ([MHz]*2^n -> Hz), 64 bit intermediate.
// The ATtiny1624 has plenty of flash; no hand written asm needed.
// --------------------------------------------------------------

static uint32_t							// [MHz]*2^21 -> Hz (rounded)
fx21_to_Hz(uint32_t f)
{
	return (uint32_t)(((uint64_t)f * 1000000ULL + (1UL<<20)) >> 21);
}

static uint32_t							// [MHz]*2^24 -> Hz (rounded)
fx24_to_Hz(uint32_t f)
{
	return (uint32_t)(((uint64_t)f * 1000000ULL + (1UL<<23)) >> 24);
}

// Cost: 140us
// This function only works for the "C" & "B" grade of the Si570 chip.
// It will not check the frequency gaps for the "A" grade chip!!!
// (Virtual Si570: the dividers for the register image.)
static uint8_t
Si570CalcDivider(uint32_t freq)
{
	// Register finding the lowest DCO frequenty
	uint8_t		xHS_DIV;
	sint16_t	xN1;
	uint16_t	xN;

	// Registers to save the found dividers
	uint8_t		sHS_DIV	= 0;
	uint8_t		sN1		= 0;
	uint16_t	sN		= 11*128;		// Total dividing
	uint16_t	N0;						// Total divider needed (N1 * HS_DIV)
	sint32_t	Freq;

	Freq.dw = freq;

	// Find the total division needed.
	// It is always one to low (not in the case reminder is zero, reminder not used here).
	// 16.0 bits = 13.3 bits / ( 11.5 bits >> 2)
#if INCLUDE_SI570_GRADE
	N0 = (R.Si570DCOMin * (uint16_t)(_2(3))) / (Freq.w1.w >> 2);
#else
	N0 = (DCO_MIN * _2(3)) / (Freq.w1.w >> 2);
#endif

	for(xHS_DIV = 11; xHS_DIV > 3; --xHS_DIV)
	{
		// Skip the unavailable divider's
		if (xHS_DIV == 8 || xHS_DIV == 10)
			continue;

		// Calculate the needed low speed divider
		xN1.w = N0 / xHS_DIV + 1;

		if (xN1.w > 128)
			continue;

		// Skip the unavailable N1 divider's
		if (xN1.b0 != 1 && (xN1.b0 & 1) == 1)
			xN1.b0 += 1;

#if INCLUDE_SI570_GRADE
		if (R.Si570Grade == CHIP_SI570_A)
		{
			// No divider restrictions!
		}
		else
		if (R.Si570Grade == CHIP_SI570_B)
		{
			if ((xN1.b0 == 1 && xHS_DIV == 4)
			||	(xN1.b0 == 1 && xHS_DIV == 5))
			{
				continue;
			}
		}
		else
		if (R.Si570Grade == CHIP_SI570_C)
		{
			if ((xN1.b0 == 1 && xHS_DIV == 4)
			||	(xN1.b0 == 1 && xHS_DIV == 5)
			||	(xN1.b0 == 1 && xHS_DIV == 6)
			||	(xN1.b0 == 1 && xHS_DIV == 7)
			||	(xN1.b0 == 1 && xHS_DIV == 11)
			||	(xN1.b0 == 2 && xHS_DIV == 4)
			||	(xN1.b0 == 2 && xHS_DIV == 5)
			||	(xN1.b0 == 2 && xHS_DIV == 6)
			||	(xN1.b0 == 2 && xHS_DIV == 7)
			||	(xN1.b0 == 2 && xHS_DIV == 9)
			||	(xN1.b0 == 4 && xHS_DIV == 4))
			{
				continue;
			}
		}
		else
		if (R.Si570Grade == CHIP_SI570_D)
		{
			if ((xN1.b0 == 1 && xHS_DIV == 4)
			||	(xN1.b0 == 1 && xHS_DIV == 5)
			||	(xN1.b0 == 1 && xHS_DIV == 6)
			||	(xN1.b0 == 1 && xHS_DIV == 7)
			||	(xN1.b0 == 1 && xHS_DIV == 11)
			||	(xN1.b0 == 2 && xHS_DIV == 4)
			||	(xN1.b0 == 2 && xHS_DIV == 5)
			||	(xN1.b0 == 2 && xHS_DIV == 6)
			||	(xN1.b0 == 2 && xHS_DIV == 7)
			||	(xN1.b0 == 2 && xHS_DIV == 9))
			// Removing the 4*4 is out of the spec of the C grade chip, it may work!
//			||	(xN1.b0 == 4 && xHS_DIV == 4))
			{
				continue;
			}
		}
		else
		{
		}
#endif


		xN = xHS_DIV * xN1.b0;
		if (sN > xN)
		{
			sN		= xN;
			sN1		= xN1.b0;
			sHS_DIV	= xHS_DIV;
		}
	}

	if (sHS_DIV == 0)
		return false;

	Si570_N      = sN;
	Si570_N1     = sN1;
	Si570_HS_DIV = sHS_DIV;

	return true;
}

// Cost: ~500us
// Compute the (virtual) Si570 RFREQ register image for the host frequency.
// frequency [MHz] * 2^21
// The original V15.15 inline asm (shift-add multiply + 40 bit restoring
// divide) is replaced by plain C 64 bit math: gcc 5.x allocated the b4
// accumulator output and the Si570_N input of the multiply asm to the same
// register, so the product (and with it the whole RFREQ image) came out
// zero.  The image layout is the one host software decodes:
//   reg 7 : [7:5] HS_DIV-4, [4:0] (N1-1)>>2
//   reg 8 : [7:6] (N1-1)&3, [5:0] RFREQ[37:32]
//   reg 9..12 : RFREQ[31:0] big endian, RFREQ = DCO/xtal in 12.28
static
uint8_t
Si570CalcRFREQ(uint32_t freq)
{
	uint8_t		sN1;
	uint64_t	prod;
	uint64_t	rfreq;

	// Convert divider ratio to SI570 register value
	sN1 = Si570_N1 - 1;
	Si570_Data.N1      = sN1 >> 2;
	Si570_Data.HS_DIV  = Si570_HS_DIV - 4;

	//============================================================================
	// RFREQ = freq * N / Xtal,  DCO = freq * N
	//============================================================================
	prod = (uint64_t)Si570_N * freq;			// DCO [MHz] * 2^21

	// Check if DCO is lower than the Si570 max specied (in 1/8 MHz units,
	// like the original, the low 3 bits are not used).
#if INCLUDE_SI570_GRADE
	if ((uint32_t)(prod >> 24) > (uint32_t)((R.Si570DCOMax + 4) / 8))
		return 0;
#else
	if ((uint32_t)(prod >> 24) > (uint32_t)((DCO_MAX + 4) / 8))
		return 0;
#endif

	// RFREQ [12.28] = prod * 2^31 / xtal_fixed   (xtal_fixed = [8.24])
	rfreq  = (prod / R.FreqXtal) << 31;
	rfreq += (((prod % R.FreqXtal) << 31) + (R.FreqXtal >> 1)) / R.FreqXtal;

	// Si570 register 8 :  76543210
	//                     ||^^^^^^------< RFREQ[37:32]
	//                     ^^------------< N1[1:0]
	Si570_Data.RFREQ_b4    = ((uint8_t)(rfreq >> 32) & 0x3F) | ((sN1 & 0x03) << 6);
	Si570_Data.RFREQ.w0.b0 = (uint8_t)(rfreq >> 24);	// RFREQ[31:24]
	Si570_Data.RFREQ.w0.b1 = (uint8_t)(rfreq >> 16);	// RFREQ[23:16]
	Si570_Data.RFREQ.w1.b0 = (uint8_t)(rfreq >> 8);		// RFREQ[15:8]
	Si570_Data.RFREQ.w1.b1 = (uint8_t)rfreq;			// RFREQ[7:0]

	return 1;
}


#if INCLUDE_SMOOTH

static uint8_t
Si570_Small_Change(uint32_t current_Frequency)
{
	uint32_t delta_F, delta_F_MAX;
	sint32_t previous_Frequency;

	// Get previous_Frequency   -> [11.21]
	previous_Frequency.dw = FreqSmoothTune;

	// Delta_F (MHz) = |current_Frequency - previous_Frequency|  -> [11.21]
	delta_F = current_Frequency - previous_Frequency.dw;
	if (delta_F >= _2(31)) delta_F = 0 - delta_F;

	// Delta_F (Hz) = (Delta_F (MHz) * 1_000_000) >> 16 not possible, overflow
	// replaced by:
	// Delta_F (Hz) = (Delta_F (MHz) * (1_000_000 >> 16)
	//              = Delta_F (MHz) * 15  (instead of 15.258xxxx)
	// Error        = (15 - 15.258) / 15.258 = 0.0169 < 1.7%

	delta_F = delta_F * 15;          // [27.5] = [11.21] * [16.0]

	// Compute delta_F_MAX (Hz)= previous_Frequency(MHz) * 3500 ppm
	delta_F_MAX = (uint32_t)previous_Frequency.w1.w * R.SmoothTunePPM;
	//   [27.5] =                          [11.5] * [16.0]

	// return TRUE if output changes less than ±3500 ppm from the previous_Frequency
	return (delta_F <= delta_F_MAX) ? true : false;
}

#endif

#if INCLUDE_IBPF

static uint8_t
GetFreqBand(uint32_t freq)
{
	uint8_t n;
	sint32_t Freq;

	Freq.dw = freq;

	for(n=0; n < MAX_BAND-1; ++n)
		if (Freq.w1.w < R.FilterCrossOver[n].w)
			return n;

	return MAX_BAND-1;
}

void
SetFilter(uint8_t filter)
{
	if (FilterCrossOverOn)
	{
		bit_1(IO_DDR, IO_P1);
		bit_1(IO_DDR, IO_P2);

		if (filter & 0x01)
			bit_1(IO_PORT, IO_P1);
		else
			bit_0(IO_PORT, IO_P1);

		if (filter & 0x02)
			bit_1(IO_PORT, IO_P2);
		else
			bit_0(IO_PORT, IO_P2);
	}
}

#endif

void
SetFreq(uint32_t freq)		// frequency [MHz] * 2^21
{
	R.Freq = freq;			// Save the asked freq

#if INCLUDE_IBPF

	uint8_t band = GetFreqBand(freq);

	freq = CalcFreqMulAdd(freq, R.BandSub[band], R.BandMul[band]);

	SetFilter(R.Band2Filter[band]);

#endif

//#ifdef INCLUDE_ABPF	<<-- Bug in V15.12
#if INCLUDE_ABPF
	if (FilterCrossOverOn)
	{
		sint32_t Freq;
		Freq.dw = R.Freq;			// Freq.w1 is 11.5bits

		bit_1(IO_DDR, IO_P1);
		bit_1(IO_DDR, IO_P2);

		if (Freq.w1.w < R.FilterCrossOver[0].w)
		{
			bit_0(IO_PORT, IO_P1);
			bit_0(IO_PORT, IO_P2);
		}
		else
		if (Freq.w1.w < R.FilterCrossOver[1].w)
		{
			bit_1(IO_PORT, IO_P1);
			bit_0(IO_PORT, IO_P2);
		}
		else
		if (Freq.w1.w < R.FilterCrossOver[2].w)
		{
			bit_0(IO_PORT, IO_P1);
			bit_1(IO_PORT, IO_P2);
		}
		else
		{
			bit_1(IO_PORT, IO_P1);
			bit_1(IO_PORT, IO_P2);
		}
	}
#endif

#if INCLUDE_FREQ_SM

	freq = CalcFreqMulAdd(freq, R.FreqSub, R.FreqMul);

#endif

	// Keep the virtual Si570 register image in sync (used by the commands
	// 0x20 write, 0x30 set by register and 0x3f read).
	if (Si570CalcDivider(freq))
		Si570CalcRFREQ(freq);

	// Retune the Si5351A.  Hosts (hamlib, PowerSDR, HDSDR, CFGSR, ...) follow
	// the SoftRock convention and send 4x the LO, because the SoftRock
	// hardware divided the Si570 output by 4 to generate quadrature.  The
	// Si5351 generates I/Q on chip (phase registers), so the outputs are
	// programmed at 1x the requested frequency: CLK0/CLK1 = host/4.
	// The register image above stays at the host value (the host view of the
	// "Si570 output" is unchanged).
	uint32_t fout = fx21_to_Hz(freq >> 2);

#if INCLUDE_SMOOTH

	if ((R.SmoothTunePPM != 0) && Si570_Small_Change(freq)
		&& si5351_retune_ms_only(fout))
	{
		return;						// smooth tune: only MS0/MS1 rewritten
	}
#endif

	si5351_setfreq_iq(fout);

#if INCLUDE_SMOOTH
	FreqSmoothTune = freq;
#endif
}

void
DeviceInit(void)
{
	// Check if the Si5351 is online and initialize if nessesary
	// The Si5351 is always assumed present: the old SoftRock V9 trick of
	// sensing the chip through the SCL line level is gone with the TWI
	// (the peripheral owns the bus pins).  While the TWI reports errors
	// the init is retried from the main loop.
	if (SI570_OffLine)
	{
#if INCLUDE_SMOOTH
			FreqSmoothTune = 0;				// Next SetFreq call no smoodtune
#endif
			si5351_set_xtal(fx24_to_Hz(R.FreqXtal));
			si5351_init(R.ChipCrtlData);	// (re)initialize the Si5351A (outputs off)
			SetFreq(R.Freq);				// first tune while the outputs are off
			si5351_output_en(SI5351_CLK0 | SI5351_CLK1);	// then enable CLK0/CLK1 (like the sketch)

			SI570_OffLine = I2CErrors;		// retry as long as the TWI reports errors
	}
}

// Write a byte into the (virtual) Si570 register set, command 0x20.
// If the register is one of the RFREQ registers (7-12 or 13-18), the
// frequency is decoded from the register image and the Si5351 is retuned.
// Si570 specific control registers (135 RECALL / Freeze-M, 137 Freeze DCO
// / NewFreq) have no Si5351 equivalent and are ignored.
void
Si570CmdReg(uint8_t reg, uint8_t data)
{
	uint8_t idx = R.Si570RFREQIndex & RFREQ_INDEX;	// 7 or 13

	if (reg >= idx && reg < idx + 6)
	{
		uint8_t buf[6];

		Si570_Data.bData[reg - idx] = data;		// Update the register image

		memcpy(buf, Si570_Data.bData, 6);
		CalcFreqFromRegSi570(buf);				// Freq from the (virtual) Si570 registers
		SetFreq(*(uint32_t*)buf);				// and retune the Si5351 with it
	}
}

// Return the (virtual) Si570 RFREQ registers, command 0x3f.
// The register image is kept in sync by SetFreq(); the chip is assumed
// present, so the full image is always returned (V15.15 length semantics
// without the old I2C presence probe).
uint8_t
Si570ReadRFREQ(uint8_t index)
{
	(void)index;		// one register image; index 7 or 13 selects the same

	return sizeof(Si570_t);
}

#endif
