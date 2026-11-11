#!/usr/bin/env python3
#
# st2hfe.py - convert a plain Atari ST sector image (.st) into an HFE v1 image
#
#   st2hfe.py disk.st disk.hfe
#
# Every track is written in the standard Atari layout (9 or 10 sectors of
# 512 bytes, MFM at 250 kbps, 300 rpm, 6250 bytes per track), so the image
# reads exactly like the .st through the HFE floppy path. Handy for testing
# that path with any unprotected disk; it adds no protection.
#
# The geometry comes from the boot sector (sectors per track, sides) and
# the image size; anything that does not fit 9/10 sectors is refused.

import struct
import sys

TRACK_BYTES = 6250          # 250 kbps, 300 rpm


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Mfm:
    """MFM cells, 16 per byte (clock, data), first cell first."""

    def __init__(self):
        self.cells = []
        self.last = 0

    def byte(self, v, n=1):
        for _ in range(n):
            for i in range(7, -1, -1):
                d = (v >> i) & 1
                self.cells += [1 if (self.last == 0 and d == 0) else 0, d]
                self.last = d

    def data(self, bs):
        for b in bs:
            self.byte(b)

    def a1(self):
        # A1 with a missing clock: cells 0x4489
        for i in range(15, -1, -1):
            self.cells.append((0x4489 >> i) & 1)
        self.last = 1


def track(trk, side, spt, sectors):
    m = Mfm()
    gap1, gap3 = (60, 40) if spt == 9 else (20, 24)
    m.byte(0x4E, gap1)
    for s in range(1, spt + 1):
        m.byte(0x00, 12)
        for _ in range(3):
            m.a1()
        idf = bytes([0xFE, trk, side, s, 2])
        m.data(idf)
        c = crc16(b'\xA1\xA1\xA1' + idf)
        m.data([c >> 8, c & 0xFF])
        m.byte(0x4E, 22)
        m.byte(0x00, 12)
        for _ in range(3):
            m.a1()
        d = b'\xFB' + sectors[s - 1]
        m.data(d)
        c = crc16(b'\xA1\xA1\xA1' + d)
        m.data([c >> 8, c & 0xFF])
        m.byte(0x4E, gap3)
    n = len(m.cells) // 16
    if n > TRACK_BYTES:
        raise SystemExit(f"track {trk}.{side} does not fit ({n} bytes)")
    m.byte(0x4E, TRACK_BYTES - n)
    # pack 8 cells per HFE byte, first cell in bit 0
    out = bytearray()
    for i in range(0, len(m.cells), 8):
        v = 0
        for k, c in enumerate(m.cells[i:i + 8]):
            v |= c << k
        out.append(v)
    return bytes(out)


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: st2hfe.py disk.st disk.hfe")
    st = open(sys.argv[1], 'rb').read()
    spt = struct.unpack_from('<H', st, 0x18)[0] if len(st) >= 0x1C else 0
    sides = struct.unpack_from('<H', st, 0x1A)[0] if len(st) >= 0x1C else 0
    if spt not in (9, 10) or sides not in (1, 2) or len(st) % (512 * spt * sides):
        # fall back to the size
        for spt, sides in ((9, 2), (9, 1), (10, 2), (10, 1)):
            if len(st) % (512 * spt * sides) == 0 and 70 <= len(st) // (512 * spt * sides) <= 86:
                break
        else:
            raise SystemExit("unknown geometry (only 9 or 10 sectors per track)")
    tracks = len(st) // (512 * spt * sides)
    print(f"{sys.argv[1]}: {tracks} tracks, {sides} side(s), {spt} sectors per track")

    side_len = TRACK_BYTES * 2                       # HFE bytes per side
    blocks = (side_len + 255) // 256                 # 512 byte blocks per track
    hdr = bytearray(b'\xFF' * 512)
    hdr[0:8] = b'HXCPICFE'
    hdr[8] = 0                                       # format revision
    hdr[9] = tracks
    hdr[10] = sides
    hdr[11] = 0                                      # ISO/IBM MFM
    struct.pack_into('<HH', hdr, 12, 250, 300)       # kbps, rpm
    hdr[16] = 2                                      # Atari ST DD
    hdr[17] = 1
    struct.pack_into('<H', hdr, 18, 1)               # track table in block 1
    hdr[20] = 0xFF                                   # write allowed
    lut = bytearray(b'\xFF' * 512)
    body = bytearray()
    for t in range(tracks):
        struct.pack_into('<HH', lut, 4 * t, 2 + t * blocks, side_len * 2)
        sides_data = []
        for s in range(2):
            if s < sides:
                base = (t * sides + s) * spt * 512
                secs = [st[base + i * 512: base + (i + 1) * 512] for i in range(spt)]
                sides_data.append(track(t, s, spt, secs))
            else:
                sides_data.append(bytes(side_len))   # no flux on a missing side
        for b in range(blocks):
            for s in range(2):
                chunk = sides_data[s][b * 256:(b + 1) * 256]
                body += chunk + bytes(256 - len(chunk))
    open(sys.argv[2], 'wb').write(bytes(hdr) + bytes(lut) + bytes(body))
    print(f"{sys.argv[2]}: {512 * 2 + len(body)} bytes")


if __name__ == '__main__':
    main()
