//************************************************************************
//**
//** Project......: Firmware USB AVR Si5351 controler.
//**
//** Platform.....: ATtiny1624
//**
//** Licence......: This software is freely available for non-commercial
//**
//** Programmer...: F.W. Krom, PE0FKO (original ATtiny45 version)
//**
//** Description..: Read the (internal ATtiny1624 chip) temperature.
//**
//** History......: Check the main.c file
//**
//**************************************************************************

#include "main.h"

#if INCLUDE_TEMP

// The original ATtiny45/85 firmware returned the raw 10 bit ADC value of
// the internal temperature sensor (command 0x42).  The ATtiny1624 ADC0 is
// a 12 bit converter; the result is shifted two bits down so the host sees
// the same 10 bit numeric range.

uint16_t
GetTemperature()
{
	uint16_t temp;

	ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;		// Mux: internal temperature sensor
	ADC0.CTRLB  = ADC_REFSEL_1024MV_gc;		// Internal 1.024V reference (t85 used 1.1V)
	ADC0.CTRLA  = ADC_ENABLE_bm				// Enable the ADC
	            | ADC_MODE_SINGLE_12BIT_gc;		// Single conversion, 12 bit result
	ADC0.COMMAND = ADC_START_IMMEDIATE_gc;	// Start one conversion

	while(!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {}

	temp = ((uint16_t)ADC0.RESULT) >> 2;		// Scale the 12 bit result to 10 bits

	ADC0.INTFLAGS = ADC_RESRDY_bm;				// Clear the result ready flag
	ADC0.CTRLA = 0;								// Disable the ADC again

	return temp;
}

#endif
