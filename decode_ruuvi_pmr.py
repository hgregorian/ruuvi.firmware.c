#!/usr/bin/env python3
import sys

SOURCES = {
    0: "NONE",
    1: "APP_FATAL",
    2: "CART_GESTURE",
    3: "BOOTLOADER_FALLBACK",
    4: "DIAG_DIRECT",
    5: "FLASH_INIT_FAILURE",
}

ERRORS = {
    0x00000000: "RD_SUCCESS",
    0x00000001: "RD_ERROR_INTERNAL",
    0x00000002: "RD_ERROR_NO_MEM",
    0x00000004: "RD_ERROR_NOT_FOUND",
    0x00000008: "RD_ERROR_NOT_SUPPORTED",
    0x00000010: "RD_ERROR_INVALID_PARAM",
    0x00000020: "RD_ERROR_INVALID_STATE",
    0x00000400: "RD_ERROR_TIMEOUT",
    0x00004000: "RD_ERROR_BUSY",
    0x00008000: "RD_ERROR_RESOURCES",
}

def u16(b, i): return int.from_bytes(b[i:i+2], 'little')
def u32(b, i): return int.from_bytes(b[i:i+4], 'little')

def decode(hexstr):
    b = bytes.fromhex(''.join(hexstr.split()))
    if len(b) != 20:
        raise SystemExit(f"expected 20 bytes, got {len(b)}")
    if b[:3] == b'PM\x02':
        source = b[3]
        flags = b[4]
        resetreas = u32(b, 5)
        err = u32(b, 9)
        line = u16(b, 13)
        h = u16(b, 15)
        load = u16(b, 17)
        print("type=PM v2")
        print(f"source={source} ({SOURCES.get(source, 'UNKNOWN')})")
        print(f"flags=0x{flags:02x} record_valid={bool(flags & 1)} fatal_valid={bool(flags & 2)}")
        print(f"RESETREAS=0x{resetreas:08x}")
        print(f"error=0x{err:08x} ({ERRORS.get(err, 'bitfield/unknown')})")
        print(f"line={line}")
        print(f"file_hash=0x{h:04x}")
        print(f"load_status=0x{load:04x} ({ERRORS.get(load, 'bitfield/unknown')})")
    elif b[:3] == b'JR\x01':
        idx = b[3]
        source = b[4]
        flags = b[5]
        seq = u32(b, 6)
        err = u32(b, 10)
        line = u16(b, 14)
        h = u16(b, 16)
        print("type=JR v1")
        print(f"reverse_index={idx}")
        print(f"source={source} ({SOURCES.get(source, 'UNKNOWN')})")
        print(f"fatal_valid={bool(flags & 1)}")
        print(f"sequence={seq}")
        print(f"error=0x{err:08x} ({ERRORS.get(err, 'bitfield/unknown')})")
        print(f"line={line}")
        print(f"file_hash=0x{h:04x}")
    else:
        raise SystemExit(f"unknown packet prefix/version: {b[:3].hex()}")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise SystemExit(f"usage: {sys.argv[0]} HEX20")
    decode(sys.argv[1])
