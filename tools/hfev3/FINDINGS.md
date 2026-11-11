# HFE test set: findings and FPGA requirements

Nine images, decoded with `hfe_model` (per-disk reports in `reports/`).

| Image | HFE | Notable content |
|---|---|---|
| Dungeon Master | v1 | T0: sector 7 data CRC error, extra sector numbered 247. Side 1 unformatted |
| Magic Pockets | v1 | T0: sector 6 data CRC error (at nominal speed: v1 has no density) |
| Lemmings | v3 | T0 sector 6 at ~95% rate with CRC error. Custom format from T0.1: five 1024-byte sectors numbered 101-105 plus ten 128-byte ID-only fields (sector numbers 0-10, no data), side byte 2 in IDs on side 1 |
| R-Type disk 1 / 2 | v3 | Side 1 and T80-83 unformatted, stored as solid RAND (weak) streams. Disk 1 T79.0: a full track of `00` bytes, no marks, cells 72-76 |
| Roller Coaster Rumbler disk 1 / 2 | v3 | T0 sector 6 at ~95% rate with CRC error. Side 1 unformatted (solid RAND) |
| Toki | v3 | T0 sector 6 at ~96% rate, CRC error **and weak bits**. T1-79: five 1024-byte sectors numbered 0-4, side byte 0 on both sides |
| Total Recall | v3 | T0 sector 6 at ~96% rate with CRC error. Many tracks with a few weak cells in the gaps |

The common protection is the track 0 sector 6 check: the data field is
written about 5% slow (cells at x=76 instead of 72, 17.2 ms instead of
16.4 ms to read) and fails its CRC; Toki also has weak bits inside it.
Toki, Total Recall, Lemmings and Roller Coaster carry it with the timing;
in the two v1 images (Dungeon Master, Magic Pockets) the CRC error is there
but HFE v1 cannot store density or weak bits.

Not present in this set: long tracks (all revolutions 198.7-201.4 ms),
duplicate sector numbers, IDs inside other sectors' data, ID CRC errors,
deleted data marks, index marks (C2 C2 C2 FC).

## What the FPGA controller must do

1. **Track buffer**: largest side stream is 20,149 bytes (Total Recall
   T83, Toki T82). Formatted sides go up to 19,474 bytes (Toki T79.0);
   plain disks are ~12.6 KB. So 20 KB per side, or 32 KB to keep a
   power of two. The HFE side interleave (256-byte halves of each 512-byte
   block) means a whole track is read from SD and one side is kept.
2. **Opcodes in real time**: up to 14,208 opcodes per side. Each stream
   byte lasts 8 cells (about 16 us), so an opcode plus its parameter byte
   fits easily between cells at 32 MHz.
3. **Cell timer**: BITRATE values x = 50-76 seen (x/36 us per cell;
   72 = 2 us). Formatted tracks use 70-76; the extremes are on unformatted
   noise tracks. A fractional counter (32 MHz, 9 per clock, cell ends at
   8x) gives exact cell lengths.
4. **DPLL**: the increment reaches both limits (117 and 140) on noise
   tracks and stays within 120-132 on formatted ones. Port `TWD1772Dpll`
   exactly; the slow sector 6 decodes the same as in Steem.
5. **Weak bits**: needed for RAND opcodes (Toki sector 6, gap bits on Total
   Recall) and for whole unformatted sides stored as RAND (R-Type, Roller
   Coaster). An LFSR per cell is enough.
6. **No geometry assumptions**: 1024- and 128-byte sectors, sector numbers
   0, 101-105 and 247, side bytes 0 and 2 in IDs, ID-only fields with no
   data. The FDC must work purely from the ID fields it finds, like a real
   WD1772 (which has no side compare).
7. **Real-time data**: sector bytes must reach the CPU at the rate the
   DPLL recovers them, so the slow sector 6 takes measurably longer, and
   CRC error status must be reported as recovered.
8. **READ TRACK**: re-frame on A1 and C2 marks like the WD1772 AM detector.
   Total Recall T0 shows the classic false `14 C2` sync before each ID
   sync, which a faithful READ TRACK reproduces.
9. **Index**: from the INDEX opcode (v3) or the track start (v1).

## Cross-check against Steem SSE with HFEv3

`steem_dump` (built from Steem's `disk_hfe.cpp` with its `tests/hfe` mocks)
and `hfe_model -s` list the same fields for all nine images: 11,787 fields,
identical except Toki T0 sector 6, whose data is random by design (weak
bits). Steem decodes from its nominal-cell word array without a DPLL; the
model plays cells in real time through the DPLL, so the match also shows
the DPLL recovers the variable-density data correctly.
