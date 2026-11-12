# HFE floppy blocks: unit tests

Verilator tests for the MFM-level floppy path in `src/fdc_hfe`, compared
cycle for cycle with the C++ reference model in `tools/hfev3`
(`hfe_model.cpp` is compiled into the testbench as is, see `ref.h`).

```
SD card --> hfe_floppy: header, track table, loader, heads, rotation, raw track buffer
                |
HFE track       v
(both sides) --> hfe_player --> flux --> wd1772_dpll --> cells --> mfm_amdetect --> wd1772_mfm --> CPU
   (track buffer)   (v3 opcodes,            (TWD1772Dpll             (A1/C2 marks,
                     cell timer,             port, 8 MHz)             framing, CRC)   (WD1772
                                                                                       commands)
                     weak bits)
```

## Build and run

Needs Verilator 5.015 or later.

```
make
./tb_hfe image.hfe ...              # every side of every image
./tb_hfe -t 0 -s 1 image.hfe        # one track / side
./tb_hfe -q image.hfe ...           # failures and per-image totals only
./tb_hfe -d 20 -t 0 image.hfe       # dump the first flux/cells next to the model
```

For every side the RTL plays three revolutions (the span `hfe_model -s`
decodes, with the same weak-bit seed) and checks, against the model:

| Block | Checked |
|---|---|
| `hfe_player` | 8 MHz cycle of every flux transition and index pulse, weak bits included; decoder never falls behind |
| `wd1772_dpll` | value and 8 MHz cycle of every recovered cell; increment range |
| `mfm_amdetect` | READ TRACK of the second revolution (every byte, A1/C2 mark and cycle, as `read_track()`); every ID/data field and its CRC status, read with `sync_en` driven as the FDC will, against `sector_lines()` (the `hfe_model -s` listing) |

The field reader in `tb_hfe.cpp` stands in for the FDC state machine:
A1 A1 A1 FE with sync on, six ID bytes with sync off, A1 A1 A1 F8-FB with
the first A1 within 43+3 bytes of the ID, then the data with sync off.

## Controller test

`tb_fdc` runs the whole chain with `wd1772_mfm` on top. The testbench is
the CPU (register accesses, DRQ served ~0.5 us after it rises) and the
drive mechanics (the head moves on step pulses and the track is
reloaded).

```
make tb_fdc
./tb_fdc image.hfe ...              # every side: READ ADDRESS, READ SECTOR, READ TRACK
./tb_fdc -1 image.hfe               # type I (restore/seek/step, verify, spin up),
                                    # multi-sector, write protect, force interrupt,
                                    # motor timeout, no disk
```

Per side, against the model: READ ADDRESS returns every ID of the
revolution and nothing else; READ SECTOR for every track/sector pair gives
the model's data, CRC error, deleted and lost-data bits, or record not
found when no such ID has data (checked once per side, as each costs five
revolutions), and the data arrives at the model's rate within 1% (the
slow sector 6); READ TRACK gives every byte in the same 8 MHz cycle as
`read_track()`.

## Whole drive test

`tb_floppy` runs the whole path with `hfe_floppy` in front: the testbench
is the CPU and the SD card (blocks of the image files served with the
core's `sd_rd`/`sd_ack`/byte-strobe handshake, ~100 us latency, ~4 MB/s).

```
make tb_floppy
./tb_floppy image.hfe [other.hfe] [not_hfe.st]
```

It mounts the image in drive A (header and track table from the card),
then on every track SEEKs with verify (seek error only where the model
has no good ID for the track) and, on each side, does READ ADDRESS and
READ SECTOR of every track/sector pair with data against the model. Every
index-to-index revolution, including those across seeks and loads, must
be within 5% of the track's own. It also switches sides in the middle of
a READ ADDRESS, and checks that, as in the core (HFE on drive A: only), the
second image in drive B is neither taken nor read and A: still reads
after B: was selected, and that a non-HFE image is not taken as HFE.

## Interfaces

- `hfe_floppy`: 32 MHz with `clk8m_en`. Image side: `img_mounted`,
  `img_size` and the SD signals of `fdc1772.v` (`sd_lba` relative to the
  image); `hfe_valid` tells which drives hold an HFE image (by its
  signature, so an `.st` is left alone). Drive side: `floppy_drive`
  (active low), `side` (the head, 1 = side 1), the controller's step
  pulses; gives `dr_present`, `dr_track0`, `dr_index` and `flux`. One raw
  track buffer (`BUF_BYTES`, 40 KB) for the selected drive, both sides,
  so side changes are immediate; a track loads on every step, drive
  change or mount. Index pulses never stop: they are kept from the last
  revolution while nothing plays. A newly loaded track starts at once
  from the side's INDEX opcode (or the stream start), fast forwarded
  (16x, hidden from the controller) until it has caught up with the
  time since the index, so the index keeps its period across seeks and
  the drive reads within a few ms of the track being in.

- `hfe_player`: 32 MHz. Reads one side of the raw HFE track (as stored
  in the file) from a synchronous RAM (one clock read latency), the
  de-interleave done in the address. The loader supplies `v3`,
  `nominal_x` (72 for 250 kbps), `stream_len` (bytes per side),
  `idx_at_start` (1 for v1, or for a v3 side without an INDEX opcode),
  `side` and `start_pos` (first byte after a reset: the side's INDEX
  opcode; 0 in the bit-exact tests). `start` marks time 0.
- `wd1772_dpll`: 32 MHz with `ce8`, the 8 MHz enable (last clock of each
  8 MHz cycle). Takes the player's `flux` pulses.
- `mfm_amdetect`: takes the DPLL cells. `sync_en` high while searching
  for a mark or doing READ TRACK; `restart` at the index for READ TRACK.
  `crc` is 0 after a good field and its two CRC bytes.
- `wd1772_mfm`: the CPU side of `src/fdc1772/fdc1772.v` (registers,
  IRQ/DRQ, step rates, status). Drives `sync_en`/`restart` of the
  detector, takes its bytes and the index pulse, and gives step pulses
  and direction to the drive. Read only for now: writes report write
  protect.
