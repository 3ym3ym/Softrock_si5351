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
//**                Port of firmware V15.15 from the ATtiny85/Si570 to the
//**                ATtiny1624/Si5351A.  The USB functionality presented to
//**                the host is kept the same as V15.15.
//**
//** History......: V15.15-Si5351: ATtiny1624 + Si5351A port.
//**                                  - V-USB 16 MHz core on PB2/PB3 through
//**                                    a pin interrupt trampoline (the
//**                                    tinyAVR 0/1/2-series does not clear
//**                                    the pin interrupt flag in hardware).
//**                                  - No OSCCAL: internal 16 MHz oscillator.
//**                                  - Si570 replaced by Si5351A; the Si570
//**                                    register commands work on a virtual
//**                                    register image.
//**
//**************************************************************************
//
//        ATtiny1624
//        +--+-+--+
//  PA0(UPDI)|  |_| VDD
//       PA1 |       | GND
//       PA2 |       | PB3 <-- USB D-
//       PA3 |       | PB2 --> USB D+
//       PA4 |       | PB1 --> I2C SDA to Si5351
//       PA5 |       | PB0 --> I2C SCL to Si5351
//       PA6 |       | ...
//        +-----------+
//
// Pin assignment:
// PB2 = USB +Data line
// PB3 = USB -Data line (pin interrupt, 1k5 pull-up, 3V6 zeners)
// PB0 = TWI0 SCL to Si5351 (fixed TWI0 pin route on the 1624)
// PB1 = TWI0 SDA to Si5351 (fixed TWI0 pin route on the 1624)
// PA1 = user defined (band filter bit 0 / PTT output)
// PA2 = user defined (band filter bit 1)
// PA3 = CW key 1 input
// PA4 = CW key 2 input
//
// Fuses (program with 'make fuse', which uses exactly these values).
// IMPORTANT: unused/reserved fuse bits default to 1 - never write 0 into
// them; in particular SYSCFG0 bit 2 must stay 1 (RSTPINCFG = UPDI), a 0
// there permanently disables UPDI (only high-voltage entry recovers).
// OSCCFG  = 0x7D : FREQSEL = 16 MHz internal oscillator, reserved = 1
// SYSCFG0 = 0xF7 : RSTPINCFG = UPDI + EESAVE (preserve EEPROM on erase)
// BODCFG  = 0x41 : BOD enabled in active mode, 2.6V threshold
//
//**************************************************************************

#include "main.h"

EEMEM	var_t		E;									// Variables in eeprom
			var_t		R									// Variables in ram
						=									// Variables in flash rom
{		.RC_OSCCAL			= 0xFF						// (not used, layout compatible)
,		.FreqXtal			= DEVICE_XTAL				// crystal frequency[MHz] (8.24bits)
,		.Freq				= 0x03866666				// Running frequency[MHz] (11.21bits)
#if INCLUDE_SMOOTH
,		.SmoothTunePPM		= 3500						// SmoothTunePPM
#endif
#if INCLUDE_FREQ_SM
,		.FreqSub			= 0.0 * _2(21)				// Freq subtract value is 0.0MHz (11.21bits)
,		.FreqMul			= 1.0 * _2(21)				// Freq multiply value os 1.0    (11.21bits)
#endif
#if INCLUDE_ABPF | INCLUDE_IBPF
,		.FilterCrossOver[0]	= {  4.0 * 4.0 * _2(5) }	// Default filter cross over
,		.FilterCrossOver[1]	= {  8.0 * 4.0 * _2(5) }	// frequency for softrock V9 (host
,		.FilterCrossOver[2]	= { 16.0 * 4.0 * _2(5) }	// sends 4x the LO). Four value array.
,		.FilterCrossOver[3]	= { true }					// ABPF is default enabled
#endif													// Filter control on/off [3]
#if INCLUDE_IBPF
,		.Band2Filter		= {	0,            1,            2,            3            }
,		.BandSub			= {	0.0 * _2(21), 0.0 * _2(21), 0.0 * _2(21), 0.0 * _2(21) }
,		.BandMul			= {	1.0 * _2(21), 1.0 * _2(21), 1.0 * _2(21), 1.0 * _2(21) }
#endif
#if INCLUDE_SN
,		.SerialNumber		= '0'						// Default USB SerialNumber ID.
#endif
#if INCLUDE_SI570_GRADE
,		.Si570DCOMin		= DCO_MIN					// min VCO frequency 4850 MHz
,		.Si570DCOMax		= DCO_MAX					// max VCO frequency 5670 MHz
,		.Si570Grade			= CHIP_SI570_C				// Si570 chip grade C default (save)
,		.Si570RFREQIndex	= RFREQ_7_INDEX				// Virtual Si570 register bank 7-12
#endif
,		.ChipCrtlData		= DEVICE_I2C				// I2C address or ChipCrtlData
};


			sint16_t	replyBuf[4];				// USB Reply buffer
			Si570_t		Si570_Data;					// (Virtual) Si570 register values
			uint8_t		SI570_OffLine;				// Si5351 offline
static	uint8_t		bIndex;
static	uint8_t		usbRequest;					// usbFunctionWrite command


EMPTY_INTERRUPT( BADISR_vect );					// Redirect all unused interrupts to reti

#include "FreqFromSi570.c"						// Include code is small size
#include "Temperature.c"						// Include code is small size

#if INCLUDE_SN
int	usbDescriptorStringSerialNumber[] = {
    USB_STRING_DESCRIPTOR_HEADER(USB_CFG_SERIAL_NUMBER_LEN),
    USB_CFG_SERIAL_NUMBER
};
#endif


/* ------------------------------------------------------------------------- */
/* --------------------------- USB interrupt glue -------------------------- */
/* ------------------------------------------------------------------------- */

/* The V-USB assembler core expects the classic INT0 behaviour where the
 * interrupt flag is cleared by hardware when the vector is executed.
 * The tinyAVR 0/1/2-series pin interrupt flag (PORTB.INTFLAGS bit 3 for
 * PB3) is only cleared by writing a '1' to it.  This naked trampoline
 * clears the flag and jumps to the V-USB handler (label USB_INTR_VECTOR
 * = usbInterruptHandler, defined in usbdrv/usbdrvasm.S).  "sbi" writes a
 * single 1-bit (the INTFLAGS register is write-1-to-clear) without
 * needing a register; the 5 extra cycles are absorbed by the adaptive
 * sync pattern search at the start of the assembler receive routine.
 */
extern void usbInterruptHandler(void) __attribute__((used));

ISR(PORTB_PORT_vect, ISR_NAKED)
{
	asm volatile(
		"	sbi		%[flags], %[bit]			\n\t"	// clear the pending PB3 flag
		"	jmp		usbInterruptHandler		\n\t"
		::
		[flags] "I" (_SFR_IO_ADDR(VPORTB.INTFLAGS)),
		[bit]   "I" (3)
	);
	(void)usbInterruptHandler;
}


/* ------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* ------------------------------------------------------------------------- */

uchar usbFunctionWrite(uchar *data, uchar len) //sends len bytes to SI5351
{
	SWITCH_START(usbRequest)

	SWITCH_CASE(CMD_SET_FREQ_REG)
		if (len == sizeof(Si570_t)) {
			CalcFreqFromRegSi570(data);			// Calc the freq from the Si570 register value
			SetFreq(*(uint32_t*)data);			// and call the SetFreq(..) with the freq!
		}

#if  INCLUDE_FREQ_SM
	SWITCH_CASE(CMD_SET_LO_SM)					// Write the frequency subtract multiply to the eeprom
		if (len == 2*sizeof(R.FreqSub)) {
			memcpy(&R.FreqSub, data, 2*sizeof(uint32_t));
			eeprom_write_block(data, &E.FreqSub, 2*sizeof(uint32_t));
		}
#endif

#if  INCLUDE_IBPF
	SWITCH_CASE(CMD_SET_LO_SM)					// Write the frequency subtract multiply to the eeprom
		if (len == 2*sizeof(uint32_t)) {
			bIndex &= MAX_BAND-1;
			memcpy(&R.BandSub[bIndex], &data[0], sizeof(uint32_t));
			eeprom_write_block(&data[0], &E.BandSub[bIndex], sizeof(uint32_t));
			memcpy(&R.BandMul[bIndex], &data[4], sizeof(uint32_t));
			eeprom_write_block(&data[4], &E.BandMul[bIndex], sizeof(uint32_t));
		}
#endif

	SWITCH_CASE(CMD_SET_FREQ)					// Set frequency by value and load Si5351
		if (len == sizeof(uint32_t)) {
			SetFreq(*(uint32_t*)data);
		}

	SWITCH_CASE(CMD_SET_XTAL)					// write new crystal frequency to EEPROM and use it.
		if (len == sizeof(R.FreqXtal)) {
			R.FreqXtal = *(uint32_t*)data;
			eeprom_write_block(data, &E.FreqXtal, sizeof(E.FreqXtal));
		}

	SWITCH_CASE(CMD_SET_STARTUP)				// Write new startup frequency to eeprom
		if (len == sizeof(R.Freq)) {
			eeprom_write_block(data, &E.Freq, sizeof(E.Freq));
		}

#if  INCLUDE_SMOOTH
	SWITCH_CASE(CMD_SET_PPM)					// Write new smooth tune to eeprom and use it.
		if (len == sizeof(R.SmoothTunePPM)) {
			R.SmoothTunePPM = *(uint16_t*)data;
			eeprom_write_block(data, &E.SmoothTunePPM, sizeof(E.SmoothTunePPM));
		}
#endif

	SWITCH_END

	return 1;
}


usbMsgLen_t
usbFunctionSetup(uchar data[8])
{
	usbRequest_t* rq = (usbRequest_t*)data;
	usbRequest = rq->bRequest;

    usbMsgPtr = (uchar*)replyBuf;
	replyBuf[0].b0 = 0xff;						// return value 0xff => command not supported


	SWITCH_START(usbRequest)


	SWITCH_CASE(CMD_GET_VERSION)				// Return software version number
		replyBuf[0].w = (VERSION_MAJOR<<8)|(VERSION_MINOR);
		return sizeof(uint16_t);


//	SWITCH_CASE(CMD_ECHO_WORD)					// ECHO value
//		replyBuf[0].w = rq->wValue.word;		// rq->bRequest identical data[1]!
//		return sizeof(uint16_t);


#if  INCLUDE_NOT_USED
	SWITCH_CASE(CMD_SET_DDR)					// set port directions
		IO_DDR = data[2];						// USB & I2C are on PORTB, protected
		return 0;								// by being on an other port
#endif


#if  INCLUDE_NOT_USED
	SWITCH_CASE(CMD_GET_PIN)					// read ports (pe0fko changed)
		replyBuf[0].b0 = IO_PIN;
		return sizeof(uint8_t);
#endif


#if  INCLUDE_NOT_USED
	SWITCH_CASE(CMD_GET_PORT)					// read port states
		replyBuf[0].b0 = IO_PORT;
		return sizeof(uint8_t);


	SWITCH_CASE(CMD_SET_PORT)					// set ports
#if  INCLUDE_ABPF | INCLUDE_IBPF
		if (!FilterCrossOverOn)
#endif
		{
			IO_PORT = data[2];					// USB & I2C are on PORTB, protected
		}										// by being on an other port
		return 0;
#endif


	SWITCH_CASE(CMD_REBOOT)						// Watchdog reset
		while(true) ;


	SWITCH_CASE(CMD_SET_IO)						// Set IO port with mask and data bytes
#if  INCLUDE_ABPF | INCLUDE_IBPF
		if (!FilterCrossOverOn)
#endif
		{	// Two I/O pins (PA1, PA2) are available.
			uint8_t msk,dat;
			msk = (rq->wValue.bytes[0] << IO_BIT_START) & (IO_BIT_MASK << IO_BIT_START);
			dat = (rq->wIndex.bytes[0] << IO_BIT_START) & (IO_BIT_MASK << IO_BIT_START);
			IO_DDR  = (IO_DDR & ~(IO_BIT_MASK << IO_BIT_START)) | msk;
			IO_PORT = (IO_PORT & ~msk) | dat;
		}
		// Return I/O pin's
		replyBuf[0].w = (IO_PIN>>IO_BIT_START) & IO_BIT_MASK;
        return sizeof(uint16_t);

	SWITCH_CASE(CMD_GET_IO)						// Read I/O bits
		replyBuf[0].w = (IO_PIN>>IO_BIT_START) & IO_BIT_MASK;
        return sizeof(uint16_t);


#if  INCLUDE_ABPF | INCLUDE_IBPF
	SWITCH_CASE(CMD_SET_FILTER)					// Read and Write the Filter Cross over point's and use it.
		uint8_t index = rq->wIndex.bytes[0];

		if (rq->wIndex.bytes[1] == 0) {
			// RX Filter cross over point table.

			if (index < 4)
			{
				R.FilterCrossOver[index].w = rq->wValue.word;

				eeprom_write_block(&R.FilterCrossOver[index].w,
						&E.FilterCrossOver[index].w,
						sizeof(E.FilterCrossOver[0].w));
			}

			usbMsgPtr = (uint8_t*)&R.FilterCrossOver;
			return 4 * sizeof(uint16_t);
		}
		else {
			// TX Filter cross over point table.

			return 0;
		}
#endif


#if  INCLUDE_SI570
	SWITCH_CASE(CMD_SET_SI570)					// [DEBUG] Write byte to Si570 register
		Si570CmdReg(rq->wValue.bytes[1], rq->wIndex.bytes[0]);
#if  INCLUDE_SMOOTH
		FreqSmoothTune = 0;						// Next SetFreq call no smoodtune
#endif
		replyBuf[0].b0 = I2CErrors;				// return I2C transmission error status
        return sizeof(uint8_t);
#endif


	SWITCH_CASE6(CMD_SET_FREQ_REG,CMD_SET_LO_SM,CMD_SET_FREQ,CMD_SET_XTAL,CMD_SET_STARTUP,CMD_SET_PPM)
		//	0x30						      	// Set frequnecy by register and load Si5351
		//	0x31								// Write the FREQ mul & add to the eeprom
		//	0x32								// Set frequency by value and load Si5351
		//	0x33								// write new crystal frequency to EEPROM and use it.
		//	0x34								// Write new startup frequency to eeprom
		//	0x35								// Write new smooth tune to eeprom and use it.
		bIndex = rq->wIndex.bytes[0];
		return USB_NO_MSG;						// use usbFunctionWrite to transfer data


#if  INCLUDE_FREQ_SM
	SWITCH_CASE(0x39)							// Return the frequency subtract multiply
		usbMsgPtr = (uint8_t*)&R.FreqSub;
        return 2 * sizeof(uint32_t);
#endif


#if  INCLUDE_IBPF
	SWITCH_CASE(CMD_GET_LO_SM)					// Return the frequency subtract multiply
		uint8_t band = rq->wIndex.bytes[0] & (MAX_BAND-1);	// 0..3 only
		memcpy(&replyBuf[0].w, &R.BandSub[band], sizeof(uint32_t));
		memcpy(&replyBuf[2].w, &R.BandMul[band], sizeof(uint32_t));
        return 2 * sizeof(uint32_t);
#endif


	SWITCH_CASE(CMD_GET_FREQ)					// Return running frequnecy
		usbMsgPtr = (uint8_t*)&R.Freq;
        return sizeof(uint32_t);


#if  INCLUDE_SMOOTH
	SWITCH_CASE(CMD_GET_PPM)					// Return smooth tune ppm value
		usbMsgPtr = (uint8_t*)&R.SmoothTunePPM;
        return sizeof(uint16_t);
#endif


	SWITCH_CASE(CMD_GET_STARTUP)				// Return the startup frequency
		eeprom_read_block(replyBuf, &E.Freq, sizeof(E.Freq));
		return sizeof(uint32_t);


	SWITCH_CASE(CMD_GET_XTAL)					// Return the XTal frequnecy
		usbMsgPtr = (uint8_t*)&R.FreqXtal;
        return sizeof(uint32_t);


//	SWITCH_CASE(CMD_GET_REGS)					// read out calculated frequency control registers
//		usbMsgPtr = (uint8_t*)&Si570_Data;
//		return sizeof(Si570_t);


	SWITCH_CASE(CMD_GET_SI570)					// read out chip frequency control registers
		usbMsgPtr = (uint8_t*)&Si570_Data;		// return the (virtual) Si570 registers
		return Si570ReadRFREQ(rq->wIndex.bytes[0] != 0 ? rq->wIndex.bytes[0] : R.Si570RFREQIndex );


#if  INCLUDE_I2C
	SWITCH_CASE(CMD_GET_I2C_ERR)				// return I2C transmission error status
		replyBuf[0].b0 = I2CErrors;
		return sizeof(uint8_t);
#endif


	SWITCH_CASE(CMD_SET_I2C_ADDR)				// Set the new i2c address or factory default (pe0fko: function changed)
		replyBuf[0].b0 = R.ChipCrtlData;		// Return the old I2C address (V15.12)
		if (rq->wValue.bytes[0] != 0) {			// Only set if Value != 0
			R.ChipCrtlData = rq->wValue.bytes[0];
			eeprom_write_byte(&E.ChipCrtlData, R.ChipCrtlData);
		}
		return sizeof(R.ChipCrtlData);


#if INCLUDE_TEMP
	SWITCH_CASE(CMD_GET_CPU_TEMP)				// Read the temperature mux 0
		replyBuf[0].w = GetTemperature();
		return sizeof(uint16_t);
#endif


#if INCLUDE_SN
	SWITCH_CASE(CMD_GET_USB_ID)					// Get/Set the USB SeialNumber ID
		replyBuf[0].b0 = R.SerialNumber;
		if (rq->wValue.bytes[0] != 0) {			// Only set if Value != 0
			R.SerialNumber = rq->wValue.bytes[0];
			eeprom_write_byte(&E.SerialNumber, R.SerialNumber);
		}
		return sizeof(R.SerialNumber);
#endif


#if INCLUDE_SI570_GRADE
	SWITCH_CASE(CMD_SET_SI570_GRADE)
		if (rq->wValue.bytes[0] != 0)
		{
			// Set Si570 grade (A,B,C) (Option code 3nd)
			R.Si570Grade = rq->wValue.bytes[0];
			eeprom_write_byte(&E.Si570Grade, R.Si570Grade);

			// Set the RFREQ register index, Option code 2nd
			R.Si570RFREQIndex = rq->wValue.bytes[1];
			eeprom_write_byte(&E.Si570RFREQIndex, R.Si570RFREQIndex);

			SI570_OffLine = true;				// Si5351 is offline, not initialized
			DeviceInit();						// Initialize the Si5351 device.
		}
		if (rq->wIndex.word != 0)
		{
			if (rq->wValue.bytes[1] == 0)
			{
				R.Si570DCOMin = rq->wIndex.word;
				eeprom_write_word(&E.Si570DCOMin, R.Si570DCOMin);
			}
			else
			{
				R.Si570DCOMax = rq->wIndex.word;
				eeprom_write_word(&E.Si570DCOMax, R.Si570DCOMax);
			}
		}
		usbMsgPtr = (uint8_t*)&R.Si570DCOMin;
        return sizeof(R.Si570Grade)+sizeof(R.Si570DCOMin)+sizeof(R.Si570DCOMax)+sizeof(R.Si570RFREQIndex);
#endif


#if INCLUDE_IBPF
	SWITCH_CASE(CMD_SET_RX_BAND_FILTER)			// Set the Filters for band 0..3
		uint8_t band = rq->wIndex.bytes[0] & (MAX_BAND-1);	// 0..3 only
		uint8_t filter = rq->wValue.bytes[0];
		eeprom_write_byte(&E.Band2Filter[band], filter);
		R.Band2Filter[band] = filter;
		usbMsgPtr = (uint8_t*)R.Band2Filter;	// Length from
        return sizeof(R.Band2Filter);

	SWITCH_CASE(CMD_GET_RX_BAND_FILTER)			// Read the Filters for band 0..3
		usbMsgPtr = (uint8_t*)R.Band2Filter;	// Length from
        return sizeof(R.Band2Filter);
#endif


	SWITCH_CASE2(CMD_SET_USRP1,CMD_GET_CW_KEY)	// set IO_P1 (cmd=0x50) and read CW key level (cmd=0x50 & 0x51)
		// CW key 1 is PA3, CW key 2 is PA4 (they used to share IO_P2/PA2
		// and the I2C SDA line PB1; PB1 is the hardware TWI SDA now)
		replyBuf[0].b0 = (_BV(CWKEY1_BIT) | _BV(CWKEY2_BIT));	// CW key's open (V15.10)
#if  INCLUDE_ABPF | INCLUDE_IBPF
		if (!FilterCrossOverOn)
#endif
		{
			if (usbRequest == 0x50)
			{
			    if (rq->wValue.bytes[0] == 0)
					bit_0(IO_PORT, IO_P1);
				else
					bit_1(IO_PORT, IO_P1);
			}

			replyBuf[0].b0 = (_BV(CWKEY1_BIT) & IO_PIN)	// read the CW key level's
						   | (_BV(CWKEY2_BIT) & IO_PIN);
		}
        return sizeof(uint8_t);

	SWITCH_END

    return 1;
}


/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int
main(void)
{
	// Disable the CLK_MAIN prescaler (CCP-protected register).  V-USB
	// timing requires the full 16 MHz CPU clock; this part was observed
	// to start with the prescaler active (CLK_MAIN ~2.7 MHz), which
	// breaks all USB bit timing.
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0x00);

	// Reset flags are informative only; clear them for a clean state.
	RSTCTRL.RSTFR = RSTCTRL_PORF_bm | RSTCTRL_BORF_bm | RSTCTRL_EXTRF_bm
	              | RSTCTRL_WDRF_bm | RSTCTRL_SWRF_bm | RSTCTRL_UPDIRF_bm;

	// Check if eeprom is initialized, use only the field ChipCrtlData.
	if (eeprom_read_byte(&E.ChipCrtlData) == 0xFF)
		eeprom_write_block(&R, &E, sizeof(E));	// Initialize eeprom to "factory defaults".
	else
		eeprom_read_block(&R, &E, sizeof(E));	// Load the persistend data from eeprom.

	SI570_OffLine = true;						// Si5351 is offline, not initialized

	// User I/O port init: all pins input, no pullups, except the IO_P1 output.
	IO_DDR  = _BV(IO_P1);						// All port pins inputs except IO_P1 switching output
	IO_PORT = 0;								// Inp on startup, no pullups

	// CW key inputs (PA3, PA4): inputs, keys pull the pin to ground.
	PORTA.PIN3CTRL = PORT_PULLUPEN_bm;			// CW key 1 (PA3)
	PORTA.PIN4CTRL = PORT_PULLUPEN_bm;			// CW key 2 (PA4)

	I2CInit();									// TWI0 host on PB0(SCL)/PB1(SDA)

#if INCLUDE_SN
	// Update the USB SerialNumber string with the correct ID from eprom.
	usbDescriptorStringSerialNumber[
		sizeof(usbDescriptorStringSerialNumber)/sizeof(int)-1] = R.SerialNumber;
#endif

	// Configure the USB D- pin interrupt (PB3): falling edge detection.
	// On the tinyAVR 0/1/2-series the ISC bits also enable the interrupt;
	// usbInit() clears a possibly pending flag afterwards.
	PORTB.PIN3CTRL = PORT_ISC_FALLING_gc;

	DeviceInit();								// Initialize the Si5351 device.

	// Start USB enumeration
	_delay_ms(100);								// First wait USB connection is stable
	usbDeviceDisconnect();
	_delay_ms(400);
	usbDeviceConnect();

	wdt_enable(WDTO_250MS);						// Watchdog 250ms

	usbInit();									// Init the USB used ports

	sei();										// Enable interupts

	while(true)
	{
	    wdt_reset();
	    usbPoll();

#if  INCLUDE_SI570
		DeviceInit();
#endif
	}
}
