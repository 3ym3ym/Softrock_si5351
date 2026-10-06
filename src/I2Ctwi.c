//************************************************************************
//**
//** Project......: Firmware USB AVR Si5351 controler.
//**
//** Platform.....: ATtiny1624 @ 16 MHz
//**
//** Licence......: This software is freely available for non-commercial
//**                use - i.e. for research and experimentation only!
//**                Based on ObDev's AVR USB driver by Christian Starkjohann
//**
//** Programmer...: F.W. Krom, PE0FKO and
//**                thanks to Tom Baier DG8SAQ for the initial program.
//**
//** Description..: I2C host (master) driver on the hardware TWI0
//**                peripheral.  On the ATtiny1624 the TWI0 pin route is
//**                fixed: SCL = PB0, SDA = PB1 (the same pins the old
//**                bit-bang driver used, so the board wiring is unchanged).
//**
//**                The byte-level API of the replaced PE0FKO bit-bang
//**                driver (I2Copencollector.c) is kept - I2CSendStart,
//**                I2CSendByte, I2CSendStop, I2CReceiveByte and the
//**                I2CErrors flag - so the callers in si5351.c and
//**                DeviceSi5351.c are unchanged.  A byte-level sequence
//**                  I2CSendStart(); I2CSendByte(addr); ... I2CSendStop();
//**                maps onto one TWI transaction: the first SendByte is
//**                written to MADDR (START + address + R/W), further bytes
//**                go to MDATA, and SendStop issues NACK|STOP.
//**
//**                Error reporting for command 0x40 (I2CErrors holds the
//**                status of the LAST transaction, cleared by
//**                I2CSendStart, derived from the TWI's own error flags):
//**                  bit 0 - bus error: bounded wait expired, arbitration
//**                          lost or bus error (TWI MSTATUS ARBLOST/BUSERR,
//**                          TWI bus timeout on a stuck SCL)
//**                  bit 1 - NACK received from the client (MSTATUS RXACK),
//**                          same bit meaning as the bit-bang driver's
//**                          SDA-high-at-acknowledge read.
//**
//**                Reading the bus level to detect the (SoftRock V9 style)
//**                "chip powered through the I2C lines" state is gone: the
//**                Si5351 is always assumed present (DeviceInit retries
//**                while the TWI reports errors).
//**
//**************************************************************************

#include "main.h"

#if INCLUDE_I2C

// Host baud: MBAUD = F_CPU/(2*F_SCL) - 5 (short rise time assumed)
#define TWI_BAUD_VALUE	((F_CPU / (uint32_t)(I2C_KBITRATE * 1000)) / 2 - 5)

// Bounded wait loop (~15 ms at 16 MHz, 5 cycles/iteration worst case).
// The TWI's own 100 us bus timeout releases a stuck SCL first; this loop
// only keeps the firmware hang-proof (the 250 ms watchdog is the backstop).
#define TWI_WAIT_LOOPS	20000u

uint8_t	I2CErrors;

static uint8_t	twiBegin;				// next byte is the address byte

// Wait until any of the MSTATUS bits in mask is set.
static uint8_t
twi_wait(uint8_t mask)
{
	uint16_t n = TWI_WAIT_LOOPS;

	do {
		if (TWI0.MSTATUS & mask)
			return 1;
	} while (--n);
	return 0;							// timed out
}

// Force the TWI back to a known idle state after an error.
static void
twi_recover(void)
{
	TWI0.MCTRLA &= ~TWI_ENABLE_bm;
	TWI0.MCTRLA |= TWI_ENABLE_bm;
	TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

void
I2CInit(void)
{
	PORTB.PIN0CTRL = PORT_PULLUPEN_bm;	// SCL idle bias (the Si5351 board
	PORTB.PIN1CTRL = PORT_PULLUPEN_bm;	// SDA also has 10k pull-ups)
	TWI0.CTRLA   = TWI_SDAHOLD_50NS_gc;
	TWI0.MBAUD   = TWI_BAUD_VALUE;		// ~200 kHz at 16 MHz
	TWI0.MCTRLA  = TWI_ENABLE_bm | TWI_TIMEOUT_100US_gc;	// polled, no irq
	TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;			// start the bus machine
}

void
I2CSendStart(void)
{
	I2CErrors = false;					// reset error flag
	twiBegin = 1;						// next byte is the address
}

void
I2CSendStop(void)
{
	uint16_t n = TWI_WAIT_LOOPS;

	TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;
	while ((TWI0.MSTATUS & TWI_BUSSTATE_gm) == (TWI_BUSSTATE_OWNER_gc & TWI_BUSSTATE_gm)) {
		if (--n == 0) {
			I2CErrors |= 1;				// bus did not return to idle
			twi_recover();
			break;
		}
	}
	twiBegin = 0;
}

void
I2CSendByte(uint8_t b)
{
	uint8_t	err = TWI_ARBLOST_bm | TWI_BUSERR_bm;

	if (twiBegin) {
		twiBegin = 0;
		TWI0.MADDR = b;					// START + address + R/W bit
		err |= TWI_RIF_bm;				// an address-read ends with RIF
	} else {
		TWI0.MDATA = b;
	}

	if (! twi_wait(TWI_WIF_bm | err)) {
		I2CErrors |= 1;					// stuck bus (TWI timeout + this loop)
		twi_recover();
		return;
	}
	if (TWI0.MSTATUS & err) {
		I2CErrors |= 1;					// arbitration lost / bus error
		twi_recover();
		return;
	}
	if (TWI0.MSTATUS & TWI_RXACK_bm)
		I2CErrors |= 2;					// client NACKed
}

uint8_t
I2CReceiveByte(void)
{
	if (! twi_wait(TWI_RIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm)) {
		I2CErrors |= 1;
		twi_recover();
		return 0;
	}
	if (TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm)) {
		I2CErrors |= 1;
		twi_recover();
		return 0;
	}
	return TWI0.MDATA;					// read clears RIF, master ACKs
}

void
I2CSend1(void)
{
	// Was 'send a 1 bit'; the si5351.c read path uses it to mark the final
	// NACK before the stop.  The TWI driver sends NACK|STOP together in
	// I2CSendStop(), so this is a no-op kept for API compatibility.
}

void
I2CSend0(void)
{
	// Was 'send a 0 bit'; not needed on the TWI, API compatibility only.
}

#endif // INCLUDE_I2C
