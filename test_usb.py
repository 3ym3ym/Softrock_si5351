#!/usr/bin/env python3
"""Functional USB test for the ATtiny1624/Si5351A firmware (DG8SAQ protocol).

Covers: version, get/set frequency, xtal, PPM, band tables, IO/PTT,
serial number, I2C status, EEPROM persistence via watchdog reboot,
factory reset to defaults.

Frequency wire format: [MHz] * 2^21 (PE0FKO V15.15 convention).
Xtal format: 8.24 fixed point.

Note on I2CErrors (0x40): the I2C layer resets the flag on every START
(I2CSendStart), so 0x40 reports the error status of the *last* I2C
transaction, not a cumulative count.
"""
import sys
import time
import struct
import usb.core
import usb.util

VID, PID = 0x16C0, 0x05DC
REPLY_FF = b"\xff"
TMO = 3000          # per-transfer timeout [ms]

# Firmware defaults (flash initializer of var_t R in main.c)
DEF_XTAL    = 0x19000000            # 25 MHz, 8.24
DEF_PPM     = 3500                  # SmoothTunePPM
DEF_I2C     = 0x60
DEF_FREQ    = 0x03866666            # 28.2 MHz wire format
DEF_BANDFIL = bytes([0, 1, 2, 3])

PASS, FAIL, INFO = "PASS", "FAIL", "INFO"
results = []


def report(name, ok, detail=""):
    results.append((name, ok))
    print(f"[{PASS if ok else FAIL}] {name}" + (f"  ({detail})" if detail else ""))


def info(name, detail):
    print(f"[{INFO}] {name}  ({detail})")


def find_dev(timeout=15.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = usb.core.find(idVendor=VID, idProduct=PID)
        if d is not None:
            try:
                d.get_active_configuration()
            except usb.core.USBError:
                try:
                    d.reset()
                    time.sleep(0.5)
                except usb.core.USBError:
                    return None
            return d
        time.sleep(0.25)
    return None


def cmd_in(dev, req, wValue=0, wIndex=0, length=1):
    return bytes(dev.ctrl_transfer(0xC0, req, wValue, wIndex, length, TMO))


def cmd_out(dev, req, wValue=0, wIndex=0, payload=None):
    if payload is None:
        dev.ctrl_transfer(0x40, req, wValue, wIndex, 0, TMO)   # wLength=0!
        return b""
    return bytes(dev.ctrl_transfer(0x40, req, wValue, wIndex, payload, TMO))


def safe(fn, *a, **kw):
    """Run a transfer; on USB error reopen the device once and retry
    (a fresh open recovers from an occasional wire-level transaction
    error). Returns None only if both attempts fail."""
    global dev
    for attempt in (0, 1):
        try:
            return fn(*a, **kw)
        except usb.core.USBError as e:
            info("USBError", f"attempt {attempt}: errno={e.errno} ({e})")
            if attempt == 0:
                time.sleep(0.3)
                nd = find_dev(10.0)
                if nd is not None:
                    try:
                        usb.util.dispose_resources(dev)
                    except Exception:
                        pass
                    dev = nd
                    if a:
                        a = (dev,) + tuple(a[1:])   # rebind stale device arg
    return None


def rd(dev, req, length, wValue=0, wIndex=0):
    """Read `length` bytes; all-0x00 sentinel on transfer failure."""
    b = safe(cmd_in, dev, req, wValue, wIndex, length)
    return b if b is not None and len(b) == length else b"\x00" * length


def hz_to_wire(hz):
    return int(round(hz * 2097152.0 / 1e6)) & 0xFFFFFFFF


def wire_to_hz(w):
    return w * 1e6 / 2097152.0


def reboot_and_wait():
    """Watchdog reboot (0x0F); the control transfer times out by design."""
    global dev
    try:
        dev.ctrl_transfer(0x40, 0x0F, 0, 0, 0, 500)   # wLength=0, short timeout
    except usb.core.USBError:
        pass
    time.sleep(0.3)                                 # WDT fires within 250 ms
    try:
        usb.util.dispose_resources(dev)
    except Exception:
        pass
    dev = find_dev(20.0)
    return dev is not None


def factory_reset():
    """0x41 <- 0xFF marks the EEPROM uninitialized; after the reboot the
    firmware rewrites the whole EEPROM from the flash defaults (main.c)."""
    old = rd(dev, 0x41, 1)
    info("EEPROM before reset", f"xtal={rd(dev, 0x3D, 4).hex()} "
         f"ppm={struct.unpack('<H', rd(dev, 0x3B, 2))[0]} "
         f"i2c={old[0]:#x} band={rd(dev, 0x19, 4).hex()}")
    safe(cmd_in, dev, 0x41, 0x00FF, 0, 1)           # write 0xFF (returns old)
    return reboot_and_wait()


def main():
    global dev
    dev = find_dev()
    if dev is None:
        print("device not found")
        sys.exit(1)
    print(f"found: bus {dev.bus} device {dev.address}")

    # --- version / unknown command -----------------------------------------
    v = safe(cmd_in, dev, 0x00, 0, 0, 2)
    report("0x00 version = 15.15", v == b"\x0f\x0f", v.hex() if v else "no reply")

    r = safe(cmd_in, dev, 0x77, 0, 0, 1)
    report("unknown cmd 0x77 -> 0xFF", r == REPLY_FF, r.hex() if r else "no reply")

    # --- factory reset: purge diag garbage, verify boot-defaults path -------
    if not factory_reset():
        print("FATAL: device did not come back after factory-reset reboot")
        sys.exit(1)

    x = struct.unpack("<I", rd(dev, 0x3D, 4))[0]
    report("after reset: 0x3D xtal = 25 MHz (8.24)", x == DEF_XTAL, f"{x:#010x}")
    ppm = struct.unpack("<H", rd(dev, 0x3B, 2))[0]
    report("after reset: 0x3B PPM = 3500", ppm == DEF_PPM, f"ppm={ppm}")
    a = rd(dev, 0x41, 1)[0]
    report("after reset: 0x41 I2C addr = 0x60", a == DEF_I2C, f"{a:#x}")
    b = rd(dev, 0x19, 4)
    report("after reset: 0x19 band filter = 00 01 02 03", b == DEF_BANDFIL, b.hex())
    su = struct.unpack("<I", rd(dev, 0x3C, 4))[0]
    report("after reset: 0x3C startup = 28.2 MHz",
          abs(wire_to_hz(su) - 28.2e6) < 2.0, f"{wire_to_hz(su):,.0f} Hz")
    sn = rd(dev, 0x43, 1)[0]
    report("after reset: 0x43 serial char = '0'", sn == 0x30, f"{sn:#04x}")

    # --- xtal set/get round trip --------------------------------------------
    safe(cmd_out, dev, 0x33, 0, 0, struct.pack("<I", DEF_XTAL))
    x2 = struct.unpack("<I", rd(dev, 0x3D, 4))[0]
    report("0x33 xtal set round trip", x2 == DEF_XTAL, f"{x2:#010x}")

    # --- PPM set/get round trip ---------------------------------------------
    safe(cmd_out, dev, 0x35, 0, 0, struct.pack("<H", DEF_PPM))
    ppm2 = struct.unpack("<H", rd(dev, 0x3B, 2))[0]
    report("0x35 PPM set round trip", ppm2 == DEF_PPM, f"ppm={ppm2}")

    # --- frequency set/get round trips --------------------------------------
    for f in (7.2e6, 10.0e6, 28.2e6, 28.5e6):
        w = hz_to_wire(f)
        safe(cmd_out, dev, 0x32, 0, 0, struct.pack("<I", w))
        time.sleep(0.05)
        rb = struct.unpack("<I", rd(dev, 0x3A, 4))[0]
        ok = abs(wire_to_hz(rb) - f) < 2.0
        report(f"0x32/0x3A freq {f/1e6:.3f} MHz", ok,
               f"wire {rb:#010x} = {wire_to_hz(rb):.1f} Hz")

    # --- startup freq write (0x34) + read ------------------------------------
    su_orig = struct.unpack("<I", rd(dev, 0x3C, 4))[0]
    safe(cmd_out, dev, 0x34, 0, 0, struct.pack("<I", su_orig))
    su2 = struct.unpack("<I", rd(dev, 0x3C, 4))[0]
    report("0x34 startup freq write round trip", su2 == su_orig,
           f"{wire_to_hz(su2):,.0f} Hz")

    # --- virtual Si570 register image (0x3F) ---------------------------------
    # The chip is assumed present (HW TWI refactor): the 6-byte image is
    # always returned; it is maintained by SetFreq().
    rf = safe(cmd_in, dev, 0x3F, 0, 0, 6)
    report("0x3F register image 6 bytes", rf is not None and len(rf) == 6,
           rf.hex() if rf else "no reply")

    # --- band filter 0x18 (value in wValue, band in wIndex, NO data stage) ---
    safe(cmd_out, dev, 0x18, 0x00, 0x00)  # band 0 <- filter 0 (default)
    b2 = rd(dev, 0x19, 4)
    report("0x18/0x19 band filter round trip", b2 == DEF_BANDFIL, b2.hex())

    # --- crossover table 0x17 -------------------------------------------------
    # CAUTION: any call with wIndex.bytes[1]==0 and index<4 WRITES wValue into
    # slot[index] (RAM+EEPROM) and returns the whole table.  Reads must repeat
    # the slot's current (default) value as wValue; read back a written slot
    # through a DIFFERENT slot's index to make the check meaningful.
    expect = (int(4.0 * 4.0 * 32), int(8.0 * 4.0 * 32),
              int(16.0 * 4.0 * 32), 1)
    t = rd(dev, 0x17, 8, expect[2], 0x02)           # rewrites slot 2 w/ default
    cx = struct.unpack("<4H", t)
    report("0x17 crossover defaults 4/16/64 MHz (host = 4x LO)", cx == expect,
           f"{cx} (slot 2 self-healed by this read)")
    mod = expect[2] ^ 0x0100
    safe(cmd_out, dev, 0x17, mod, 0x02)             # write slot 2
    cx2 = struct.unpack("<4H", rd(dev, 0x17, 8, expect[0], 0x00))
    report("0x17 crossover write (band 2)", cx2[2] == mod, f"{cx2}")
    safe(cmd_out, dev, 0x17, expect[2], 0x02)       # restore default

    # --- IO / PTT / CW key ----------------------------------------------------
    r = safe(cmd_out, dev, 0x15, 0x03, 0x00)        # mask=3, data=0 -> both low
    report("0x15 SET_IO", r is not None)            # OUT: no data stage back
    r = rd(dev, 0x16, 2)
    info("0x16 GET_IO", r.hex() + " (PA2 may float)")
    r = safe(cmd_out, dev, 0x50, 0x01)              # PTT on
    report("0x50 PTT on", r is not None)
    r = safe(cmd_out, dev, 0x50, 0x00)              # PTT off
    r = rd(dev, 0x51, 1)
    # keys open -> both pins pulled high -> 0x08|0x10 = 0x18
    report("0x51 CW key read, open = 0x18", r == b"\x18", r.hex())

    # --- serial number char: rewrite same value (avoid extra EEPROM wear) -----
    safe(cmd_in, dev, 0x43, sn, 0, 1)
    sn2 = rd(dev, 0x43, 1)[0]
    report("0x43 serial char round trip", sn2 == sn, f"{sn2:#04x} ('{chr(sn2)}')")

    # --- Si570 grade block -----------------------------------------------------
    g = rd(dev, 0x44, 6)
    report("0x44 grade block 6 bytes", any(g), g.hex())

    # --- temperature ------------------------------------------------------------
    t = struct.unpack("<H", rd(dev, 0x42, 2))[0]
    info("0x42 temperature", f"raw={t}")

    # --- TWI status ---------------------------------------------------------------
    # bit0 = bus error/timeout, bit1 = NACK.  0 means the last transaction
    # (typically the DeviceInit retry or the last frequency set) succeeded,
    # i.e. the Si5351 answered.  Not asserted here: with the chip absent the
    # status reads 2 by design.
    err = rd(dev, 0x40, 1)[0]
    if err == 0:
        report("0x40 TWI status = 0 (Si5351 answered)", True, "status=0")
    else:
        info("0x40 TWI status != 0", f"status={err} "
             "(1=bus error/timeout, 2=NACK; expected while the Si5351 does "
             "not answer, e.g. chip absent)")

    # --- EEPROM persistence: PPM via watchdog reboot --------------------------
    safe(cmd_out, dev, 0x35, 0, 0, struct.pack("<H", 1234))
    if not reboot_and_wait():
        report("0x0F reboot + re-enumeration", False, "device did not return")
    else:
        ppm3 = struct.unpack("<H", rd(dev, 0x3B, 2))[0]
        report("0x0F reboot + PPM persists (EEPROM)", ppm3 == 1234, f"ppm={ppm3}")

    safe(cmd_out, dev, 0x35, 0, 0, struct.pack("<H", DEF_PPM))
    reboot_and_wait()
    ppm4 = struct.unpack("<H", rd(dev, 0x3B, 2))[0]
    report("PPM restored to default", ppm4 == DEF_PPM, f"ppm={ppm4}")

    # --- summary ----------------------------------------------------------------
    fails = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(fails)}/{len(results)} passed")
    if fails:
        print("FAILED:", ", ".join(fails))
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
