# HFE reference model

`hfe_model` is a bit-level C++ model of the chain the MiSTeryNano core
needs in order to read HFE (v1 and v3) floppy images, including copy
protected ones:

```
HFE track stream -> track player -> flux -> WD1772 DPLL -> cells -> mark detector -> ID/data + CRC
```

- **Track player**: de-interleaves a side from the HFE blocks, handles the
  v3 opcodes (NOP, INDEX, BITRATE, SKIPBITS, RAND) the same way as Steem SSE
  with HFEv3 (`DecodeV3Track`), and times every cell with the current
  bitrate, as the hardware will. Weak (RAND) bits get fresh random values on
  every revolution.
- **DPLL**: a port of Steem SSE's `TWD1772Dpll` (patent US 4808884, via
  MAME's wd_fdc). It steps once per 8 MHz cycle, so it maps directly onto
  logic.
- **Mark detector / fields**: A1 (`0x4489`) and C2 (`0x5224`) marks, three
  A1s before an ID or data mark, CRC-CCITT, data mark within 43 bytes of
  the ID, as on a WD1772 in MFM mode.

It is meant as the reference for the Verilog: the same HFE can go through
this model and through the Verilator simulation, and the results are
compared.

## st2hfe.py

Converts a plain `.st` sector image into an HFE v1 image with the standard
Atari track layout (9 or 10 sectors), so any unprotected disk can be used
to try the HFE floppy path; it reads exactly like the `.st`.

```
./st2hfe.py disk.st disk.hfe
```

## Build

```
make
```

## Use

```
./hfe_model image.hfe ...          # report unusual sides + summary per disk
./hfe_model -v image.hfe           # every side and every ID field
./hfe_model -t 0 image.hfe         # one track only
./hfe_model -r 0 0 image.hfe       # what READ TRACK returns for track 0 side 0
./hfe_model -s image.hfe           # fixed-format field list, for diffing decoders
```

Sector flags in the report: `/size` not 512, `@T.S` ID differs from the
physical track/side, `!id` ID CRC bad, `!dat` data CRC bad, `-` no data
field, `del` deleted data mark, `W` weak (reads differently each pass),
`~pct` data rate compared with 250 kbps, `^` ID inside another sector's
data, `*` duplicate sector number.

## Cross-check

`-s` output was compared with an equivalent dump driven by Steem SSE with
HFEv3's own `TImageHFE` decoder (built with its `tests/hfe` mocks) over nine
real images: all 11,787 ID/data fields match, apart from one sector whose
weak bits are random by design. See `FINDINGS.md`.

## Licence

GPLv3 (same as the MiSTeryNano floppy code and Steem SSE).
