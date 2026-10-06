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
//** Programmer...: F.W. Krom, PE0FKO and
//**                thanks to Tom Baier DG8SAQ for the initial program.
//**
//** Description..: Control the Si5351 Freq. PLL chip over the USB port.
//**                Port of the V15.15 ATtiny85/Si570 firmware to the
//**                ATtiny1624 with Si5351A frequency control.
//**
//** History......: V15.1 02/12/2008: First release of PE0FKO.
//**                Check the main.c file
//**
//**************************************************************************

#ifndef _PE0FKO_MAIN_H_
#define _PE0FKO_MAIN_H_ 1

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "usbconfig.h"
#include "usbdrv.h"
#include "usbavrcmd.h"


#define	VERSION_MAJOR	15
#define	VERSION_MINOR	15


// Switch's to set the code needed
#define	INCLUDE_NOT_USED		1			// Compatibility old firmware, I/O functions
#define	INCLUDE_SI570			1			// Code generation for the Si570 register
											// (virtual Si570 register) image and Si5351 chip
#define	INCLUDE_AD9850			0			// Code generation for the DDS AD9850 chip

// A costommised USB serial number must be enabled in the usbconfig.h file!
// Firmware changable USB serial number.
#define INCLUDE_SN				(USB_CFG_DESCR_PROPS_STRING_SERIAL_NUMBER & USB_PROP_IS_RAM)


#if	INCLUDE_SI570							// Need i2c for the Si5351 chip
#define	DEVICE_XTAL		( 25 * _2(24) )		// Si5351 reference crystal (25 MHz) [8.24]
#define	DEVICE_I2C		( 0x60 )			// Default Si5351 I2C address (can change per device)
#define	INCLUDE_I2C				1			// Include the i2c code
#define	INCLUDE_IBPF			1			// Include inteligent band-pass-filter code.
#define	INCLUDE_ABPF			0			// Include automatic band pass filter selection
#define	INCLUDE_FREQ_SM			0			// Include frequency subtract multiply values
#define INCLUDE_SMOOTH			1			// Include automatic smooth tune for the Si5351 chip
#define INCLUDE_TEMP			1			// Include the temperature code
#define INCLUDE_SI570_GRADE		1			// Include Si570 Grade select
#endif


#if	INCLUDE_AD9850							// Code generation for the DDS AD9850 chip
#define	DEVICE_XTAL		( 100.0 * _2(24) )	// Clock of the DDS chip [8.24]
#define	DEVICE_I2C		( 0x00 )			// Used for DDS control / phase word
#define	INCLUDE_ABPF			0			// No automatic band pass filter selection
#define	INCLUDE_IBPF			0			// Code generation for the inteligent band-pass-filter
#define	INCLUDE_SMOOTH			0			// No smooth tune if the is no Si570 chip
#define	INCLUDE_FREQ_SM			1			// Include frequency subtract multiply values
#define	INCLUDE_SI570_GRADE		0			// Include Si570 Grade select
#endif

#if INCLUDE_IBPF
#undef	INCLUDE_ABPF
#define	INCLUDE_ABPF			0			// ABPF is part of the new IBPF
#undef	INCLUDE_FREQ_SM
#define	INCLUDE_FREQ_SM			0			// Freq offset/multiply is part of the new IBPF
#endif


// --------------------------------------------------------------
// ATtiny1624 pin assignment (all user I/O on PORTA, USB and the
// hardware TWI0 bus to the Si5351 on PORTB):
//
//  PA0  UPDI (programming; not used as I/O)
//  PA1  user I/O #1 (band filter bit 0 / PTT output)
//  PA2  user I/O #2 (band filter bit 1 / was CW key 1 input)
//  PA3  CW key 1 input (was free; the key moved off PA2/PB1)
//  PA4  CW key 2 input (was free; was the I2C SDA line before the TWI)
//  PA5..PA7  free
//  PB0  TWI0 SCL to Si5351A (fixed pin route on the 1624)
//  PB1  TWI0 SDA to Si5351A (fixed pin route on the 1624)
//  PB2  USB D+
//  PB3  USB D- (pin interrupt)
// --------------------------------------------------------------

#define IO_DDR			VPORTA.DIR
#define IO_PORT			VPORTA.OUT
#define IO_PIN			VPORTA.IN
#define	IO_BIT_START	1					// First I/O line used (PA1)
#define	IO_BIT_LENGTH	2					// Using 2 I/O line (PA1, PA2)
#define	IO_BIT_MASK	( (1<<IO_BIT_LENGTH)-1 )
#define IO_P1			( IO_BIT_START+0 )
#define IO_P2			( IO_BIT_START+1 )


#if INCLUDE_I2C
//#define	I2C_KBITRATE	400.0			// I2C Bus speed in Kbs
#define	I2C_KBITRATE	200.0				// 400 was to high?!?

// CW key inputs on free PORTA pins (they used to share IO_P2/PA2 and the
// I2C SDA line PB1; PB1 is the hardware TWI SDA now).
#define	CWKEY1_BIT		3					// PA3, CW key 1
#define	CWKEY2_BIT		4					// PA4, CW key 2

extern	void		I2CInit(void);
extern	uint8_t		I2CErrors;
extern	void		I2CSendStart(void);
extern	void		I2CSendStop(void);
extern	void		I2CSendByte(uint8_t b);
extern	void 		I2CSend0(void);
extern	void 		I2CSend1(void);
extern	uint8_t		I2CReceiveByte(void);
#endif

#if INCLUDE_AD9850
#define	DDS_PORT		VPORTA.OUT
#define	DDS_DATA		1					// PA1
#define	DDS_W_CLK		2					// PA2
#define	DDS_FQ_UD		3					// PA3
#endif

#define	true			1
#define	false			0

#define	_2(x)		((uint32_t)1<<(x))	// Take power of 2

#define	bit_1(port,bit)	port |= _BV(bit)	// Set bit to one
#define	bit_0(port,bit)	port &= ~_BV(bit)	// Set bit to zero

#define	MAX_BAND		(_2(IO_BIT_LENGTH))	// Max of 4 band's


typedef union {
	uint16_t		w;
	struct {
		uint8_t		b0;
		uint8_t		b1;
	};
} sint16_t;

typedef union {
	uint32_t		dw;
	struct {
		sint16_t	w0;
		sint16_t	w1;
	};
} sint32_t;

typedef union {
	uint8_t			bData[6];
	uint16_t		wData[3];
	struct {
		uint8_t		N1:5;
		uint8_t		HS_DIV:3;
		uint8_t		RFREQ_b4;				// N1[1:0] RFREQ[37:32]
		sint32_t	RFREQ;					// RFREQ[31:0]
	};
} Si570_t;

typedef struct
{
		uint8_t		RC_OSCCAL;				// CPU osc tune value (not used, layout compatible)
	uint32_t	FreqXtal;				// crystal frequency[MHz] (8.24bits)
	uint32_t	Freq;					// Running frequency[MHz] (11.21bits)
#if INCLUDE_SMOOTH
		uint16_t	SmoothTunePPM;			// Max PPM value for the smooth tune
#endif
#if INCLUDE_FREQ_SM
		uint32_t	FreqSub;				// Freq subtract value[MHz] (11.21bits)
	uint32_t	FreqMul;				// Freq multiply value (11.21bits)
#endif
#if INCLUDE_ABPF | INCLUDE_IBPF
	sint16_t	FilterCrossOver[MAX_BAND];// Filter cross over points [0..2] (11.5bits)
#endif										// Filter control on/off [3]
#if INCLUDE_IBPF
	uint8_t		Band2Filter[MAX_BAND];	// Filter number for band 0..3
	uint32_t	BandSub[MAX_BAND];		// Freq subtract value[MHz] (11.21bits) for band 0..3
	uint32_t	BandMul[MAX_BAND];		// Freq multiply value (11.21bits) for band 0..3
#endif										// Filter control on/off [3]
#if INCLUDE_SN
	uint8_t		SerialNumber;			// Default serial number last char! ("PE0FKO-2.0")
#endif
#if INCLUDE_SI570_GRADE
	uint16_t	Si570DCOMin;			// Si570 Minimal DCO value
	uint16_t	Si570DCOMax;			// Si570 Maximal DCO value
	uint8_t		Si570Grade;				// Si570 chip grade
	uint8_t		Si570RFREQIndex;		// [0..6bit]Index to be used for the RFFREQ registers (7-12, 13-18), [7bit] Use freeze RFREQ register
#endif
	uint8_t		ChipCrtlData;			// I2C addres, default 0x60 (96 dec)

} var_t;


extern	var_t		R;						// Variables in Ram
extern	sint16_t	replyBuf[4];			// USB Reply buffer
extern	Si570_t		Si570_Data;				// Registers 7..12 value for the (virtual) Si570
extern	uint8_t		SI570_OffLine;			// Si5351 offline

extern	uint8_t	Si570ReadRFREQ(uint8_t index);
extern	void		SetFreq(uint32_t freq);
extern	void		DeviceInit(void);
extern	void		CalcFreqFromRegSi570(uint8_t* reg);	// Freq from (virtual) Si570 registers

#if INCLUDE_SI570

#define	DCO_MIN		4850					// min VCO frequency 4850 MHz
#define DCO_MAX		5670					// max VCO frequency 5670 MHz

// The divider restrictions for the 3 Si57x speed grades or frequency grades are as follows
// - Grade A covers 10 to 945 MHz, 970 to 1134 MHz, and 1213 to 1417.5 MHz. Speed grade A
//   device have no divider restrictions.
// - Grade B covers 10 to 810 MHz. Speed grade B devices disable the output in the following
//   N1*HS_DIV settings: 1*4, 1*5
// - Grade C covers 10 to 280 MHz. Speed grade C devices disable the output in the following
//   N1*HS_DIV settings: 1*4, 1*5, 1*6, 1*7, 1*11, 2*4, 2*5, 2*6, 2*7, 2*9, 4*4
#define	CHIP_SI570_A			1			// Si570 Grade A device is used. (10 - 1417MHz)
#define	CHIP_SI570_B			2			// Si570 Grade B device is used. (10 - 810MHz)
#define	CHIP_SI570_C			3			// Si570 Grade C device is used. (10 - 280MHz)
// Removing the 4*4 is out of the spec of the C grade chip, it may work!
#define	CHIP_SI570_D			4			// Si570 Grade C device is used. (10 - 354MHz)

// The Si5351 replaces the Si570; there is no register bank auto-detection
// anymore. Default is the "old" RFREQ register bank 7-12 used by most host
// tools. Still changeable with command 0x44 (affects the virtual register
// image only).
#define	RFREQ_AUTO_INDEX		0
#define	RFREQ_7_INDEX			7
#define	RFREQ_13_INDEX			13
#define	RFREQ_INDEX				0x7F
#define	RFREQ_FREEZE			0x80

extern	void		Si570CmdReg(uint8_t reg, uint8_t data);
#endif

#if INCLUDE_SMOOTH
extern	uint32_t	FreqSmoothTune;			// The smooth tune center frequency
#endif

#if INCLUDE_ABPF | INCLUDE_IBPF
#define	FilterCrossOverOn	(R.FilterCrossOver[MAX_BAND-1].b0 != 0)
#endif

#if 0
#   define SWITCH_START(cmd)       switch(cmd){{
#   define SWITCH_CASE(value)      }break; case (value):{
#   define SWITCH_CASE2(v1,v2)     }break; case (v1): case(v2):{
#   define SWITCH_CASE3(v1,v2,v3)  }break; case (v1): case(v2): case(v3):{
#   define SWITCH_CASE6(v1,v2,v3,v4,v5,v6)  }break; case (v1): case(v2): case(v3): case(v4): case(v5): case(v6):{
#   define SWITCH_DEFAULT          }break; default:{
#   define SWITCH_END              }}
#else
#   define SWITCH_START(cmd)       {uchar _cmd = cmd; if(0){
#   define SWITCH_CASE(value)      }else if(_cmd == (value)){
#   define SWITCH_CASE2(v1,v2)     }else if(_cmd == (v1) || _cmd == (v2)){
#   define SWITCH_CASE3(v1,v2,v3)  }else if(_cmd == (v1) || _cmd == (v2) || (_cmd == v3)){
#   define SWITCH_CASE6(v1,v2,v3,v4,v5,v6)  }else if(_cmd == (v1) || _cmd == (v2) || (_cmd == v3) || (_cmd == v4) || (_cmd == v5) || (_cmd == v6)){
#   define SWITCH_DEFAULT          }else{
#   define SWITCH_END              }}
#endif


#endif
