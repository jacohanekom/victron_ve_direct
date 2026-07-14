#!/usr/bin/env python3
"""Fake SmartSolar MPPT on a PTY: streams VE.Direct text frames and answers
HEX protocol ping/get/set with correct checksums. Prints the slave PTY path."""
import os, pty, sys, threading, time

master, slave = pty.openpty()
print(os.ttyname(slave), flush=True)

REGS = {  # reg -> (size_bytes, value)
    0x0200: (1, 1),      # device_mode: charger on
    0xEDEF: (1, 12),     # battery_voltage 12V
    0xEDF0: (2, 500),    # max charge current 50.0A
    0xEDF1: (1, 2),      # battery type gel
    0xEDF7: (2, 1440),   # absorption 14.40V
    0xEDF6: (2, 1380),   # float 13.80V
    0xEDAB: (1, 4),      # load always on
    0xED9C: (2, 1110),
    0xED9D: (2, 1320),
}
UNSUPPORTED_OK = True  # unknown regs answered with flags bit0

def hexframe(cmd, payload):
    s = cmd & 0xF
    body = f":{cmd:X}"
    for b in payload:
        body += f"{b:02X}"
        s = (s + b) & 0xFF
    chk = (0x55 - s) & 0xFF
    return (body + f"{chk:02X}\n").encode()

def text_frame():
    fields = [("PID","0xA067"),("FW","161"),("SER#","HQ2241TEST1"),
              ("V","12540"),("I","150"),("VPV","18200"),("PPV","4"),
              ("CS","5"),("MPPT","2"),("ERR","0"),("LOAD","ON"),("IL","150"),
              ("H19","12345"),("H20","12"),("H21","45"),("H22","98"),
              ("H23","52"),("HSDS","5")]
    out = b""
    for k, v in fields:
        out += b"\r\n" + k.encode() + b"\t" + v.encode()
    out += b"\r\nChecksum\t"
    chk = (256 - sum(out)) & 0xFF
    return out + bytes([chk])

def le(v, n):
    return [(v >> (8*i)) & 0xFF for i in range(n)]

def reader():
    buf = b""
    inhex = False
    while True:
        c = os.read(master, 1)
        if not c:
            return
        if inhex:
            if c == b"\n":
                handle_hex(buf.decode(errors="replace"))
                buf = b""; inhex = False
            elif c != b"\r":
                buf += c
        elif c == b":":
            inhex = True; buf = b""

def handle_hex(body):
    cmd = int(body[0], 16)
    data = bytes.fromhex(body[1:])
    # verify checksum
    if (cmd + sum(data)) & 0xFF != 0x55:
        os.write(master, hexframe(4, [0xAA]))  # frame error
        return
    data = data[:-1]
    if cmd == 0x1:  # ping -> :5 app version
        os.write(master, hexframe(5, [0x61, 0x41]))  # v1.61 candidate-ish
    elif cmd == 0x7:  # get
        reg = data[0] | (data[1] << 8)
        if reg in REGS:
            size, val = REGS[reg]
            os.write(master, hexframe(7, le(reg,2) + [0x00] + le(val, size)))
        else:
            os.write(master, hexframe(7, le(reg,2) + [0x01]))  # unknown id
    elif cmd == 0x8:  # set
        reg = data[0] | (data[1] << 8)
        val_bytes = list(data[3:])
        if reg not in REGS:
            os.write(master, hexframe(8, le(reg,2) + [0x01]))
        else:
            size, _ = REGS[reg]
            v = 0
            for i, b in enumerate(val_bytes): v |= b << (8*i)
            if reg == 0xEDF7 and not (1300 <= v <= 1600):  # simulate range check
                os.write(master, hexframe(8, le(reg,2) + [0x04]))
            else:
                REGS[reg] = (size, v)
                os.write(master, hexframe(8, le(reg,2) + [0x00] + le(v, size)))

threading.Thread(target=reader, daemon=True).start()
while True:
    os.write(master, text_frame())
    time.sleep(0.5)
