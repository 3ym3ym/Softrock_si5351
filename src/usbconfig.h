//************************************************************************
//**
//** Project......: Firmware USB AVR Si5351 controler.
//**
//** Platform.....: ATtiny1624 @ 16 MHz (internal oscillator)
//**
//** Licence......: This software is freely available for non-commercial
//**                use - i.e. for research and experimentation only!
//**                Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
//**                Based on ObDev's AVR USB driver by Christian Starkjohann
//**
//** Programmer...: Port of the PE0FKO Si570 firmware V15.15 usbconfig.h
//**
//** Description..: V-USB configuration for the ATtiny1624.
//**
//** History......: V15.15-Si5351: First ATtiny1624/Si5351A version.
//**
//**************************************************************************

#ifndef __usbconfig_h_included__
#define __usbconfig_h_included__

/* ---------------------------- Hardware Config ---------------------------- */

/* The USB bus lines are on PORTB:
 *   PB2 = USB D+ (through 3.6V zener to the bus line)
 *   PB3 = USB D- (through 3.6V zener, 1.5k pull-up resistor to Vcc)
 * D- uses the PORTB pin interrupt (INT0 group, falling edge).  The vector
 * is entered through a small trampoline in main.c that clears the pin
 * interrupt flag (the tinyAVR 0/1/2-series does not clear it in hardware
 * when the vector executes, unlike the classic AVR INT0).
 */
#define USB_CFG_IOPORTNAME          B
#define USB_CFG_DMINUS_BIT          3
#define USB_CFG_DPLUS_BIT           2

/* Access the port through the fast VPORT aliases.  In assembler context
 * these expand to the plain I/O addresses (VPORTB_DIR=4, _OUT=5, _IN=6)
 * so "in"/"out"/"sbi" instructions can be used by the V-USB asm core.
 * (The double define trick of usbdrv.h - USB_CONCAT_EXPANDED - is needed
 * so the inner paste happens before the outer token concatenation.)
 */
#define USB_OUTPORT(name)           USB_CONCAT_EXPANDED(VPORT, USB_CONCAT_EXPANDED(name, _OUT))
#define USB_INPORT(name)            USB_CONCAT_EXPANDED(VPORT, USB_CONCAT_EXPANDED(name, _IN))
#define USB_DDRPORT(name)           USB_CONCAT_EXPANDED(VPORT, USB_CONCAT_EXPANDED(name, _DIR))

/* Pin interrupt wiring for the USB D- line (PB3):
 * On the tinyAVR 0/1/2-series the pin interrupt is enabled by the ISC bits
 * in PORTB.PIN3CTRL (done in main() before usbInit()); there is no
 * separate interrupt mask register.  V-USB's usbInit() executes
 * "USB_INTR_ENABLE |= (1 << USB_INTR_ENABLE_BIT)"; pointing that macro at
 * PORTB.INTFLAGS (write-1-to-clear) makes it clear the pending PB3 flag,
 * which is exactly what is wanted before the interrupt is activated.
 * USB_INTR_PENDING: VPORTB.INTFLAGS - in assembler this is the plain I/O
 * address 7, so the asm core can clear the flag with "out".
 */
#define USB_INTR_CFG                 PORTB.PIN3CTRL
#define USB_INTR_CFG_SET             0
#define USB_INTR_CFG_CLR             0
#define USB_INTR_ENABLE              PORTB.INTFLAGS
#define USB_INTR_ENABLE_BIT          3
#define USB_INTR_PENDING             VPORTB_INTFLAGS
#define USB_INTR_PENDING_BIT         3

/* The assembler core defines the ISR entry label with this name; the real
 * vector (PORTB_PORT_vect) is bound to it by the naked trampoline in main.c.
 */
#define USB_INTR_VECTOR              usbInterruptHandler

#define USB_CFG_CLOCK_KHZ            (F_CPU/1000)
/* 16 MHz -> usbdrvasm16.inc is selected by usbdrvasm.S.
 * The internal 16 MHz oscillator is used (OSCCFG fuse = 0x01); there is no
 * runtime OSCCAL on the tinyAVR 0/1/2-series, so the V1.5 oscillator
 * calibration against USB SOF is not available anymore.
 */

/* ----------------------- Optional Hardware Config ------------------------ */

/* The 1.5k pull-up resistor on D- is hard-wired to Vcc; dis/connecting is
 * done by releasing/driving the D- pin itself (default V-USB behaviour). */

/* --------------------------- Functional Range ---------------------------- */

#define USB_CFG_MAX_BUS_POWER        20  // mA
#define USB_CFG_IMPLEMENT_FN_WRITE   1
/* No USB_RESET_HOOK / usbMeasureFrameLength(): no oscillator calibration
 * is possible (or needed) on the internal 16 MHz oscillator. */

/* -------------------------- Device Description --------------------------- */
/* Device identity must stay identical to the original firmware so the host
 * side software (libusb based DG8SAQ/PE0FKO tools) recognise the device. */

#define USB_CFG_VENDOR_ID            0xc0, 0x16
#define USB_CFG_DEVICE_ID            0xdc, 0x05
#define USB_CFG_DEVICE_VERSION       0x00, 0x01

#define USB_CFG_VENDOR_NAME          'w', 'w', 'w', '.', 'o', 'b', 'd', 'e', 'v', '.', 'a', 't'
#define USB_CFG_VENDOR_NAME_LEN      12
#define USB_CFG_DEVICE_NAME          'D', 'G', '8', 'S', 'A', 'Q', '-', 'I', '2', 'C'
#define USB_CFG_DEVICE_NAME_LEN      10
#define USB_CFG_SERIAL_NUMBER        'P','E','0','F','K','O','-','0'
#define USB_CFG_SERIAL_NUMBER_LEN    8

// Firmware changable USB serial number (last character, command 0x43).
#define USB_CFG_DESCR_PROPS_STRING_SERIAL_NUMBER    (USB_PROP_IS_RAM | (2 * USB_CFG_SERIAL_NUMBER_LEN + 2))

/* The remainder follows the stock usbconfig-prototype.h defaults. */

#define USB_CFG_HAVE_INTRIN_ENDPOINT     0
#define USB_CFG_HAVE_INTRIN_ENDPOINT3    0
#define USB_CFG_EP3_NUMBER               3
#define USB_CFG_IMPLEMENT_HALT           0
#define USB_CFG_SUPPRESS_INTR_CODE       0
#define USB_CFG_INTR_POLL_INTERVAL       10
#define USB_CFG_IS_SELF_POWERED          0
#define USB_CFG_IMPLEMENT_FN_READ        0
#define USB_CFG_IMPLEMENT_FN_WRITEOUT    0
#define USB_CFG_HAVE_FLOWCONTROL         0
#define USB_CFG_LONG_TRANSFERS           0
#define USB_COUNT_SOF                    0
#define USB_CFG_CHECK_DATA_TOGGLING      0
#define USB_CFG_HAVE_MEASURE_FRAME_LENGTH 0

#define USB_CFG_DEVICE_CLASS             0xff    /* vendor specific */
#define USB_CFG_DEVICE_SUBCLASS          0
#define USB_CFG_INTERFACE_CLASS          0
#define USB_CFG_INTERFACE_SUBCLASS       0
#define USB_CFG_INTERFACE_PROTOCOL       0

#define USB_CFG_DESCR_PROPS_DEVICE                   0
#define USB_CFG_DESCR_PROPS_CONFIGURATION            0
#define USB_CFG_DESCR_PROPS_STRINGS                  0
#define USB_CFG_DESCR_PROPS_STRING_0                 0
#define USB_CFG_DESCR_PROPS_STRING_VENDOR            0
#define USB_CFG_DESCR_PROPS_STRING_PRODUCT           0
#define USB_CFG_DESCR_PROPS_HID                      0
#define USB_CFG_DESCR_PROPS_HID_REPORT               0
#define USB_CFG_DESCR_PROPS_UNKNOWN                  0

#endif /* __usbconfig_h_included__ */
