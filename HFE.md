# HFE floppy images

Besides plain `.st` images, drive A: takes HFE images (HxC
Floppy Emulator format, v1 and v3), so copy protected disks can be used:
fuzzy (weak) bits, sectors written at a different speed, extra or
missing sectors, unformatted tracks and the like are all played back as
they are on the disk.

## Use

Select an `.hfe` file for Disk A: in the menu, as you would an `.st`. The
core looks at the file itself: a file with the HFE signature goes through
the HFE floppy path, anything else through the existing `.st` floppy.
There is no drive B: in this build: the HFE path and the `.st` floppy
together fill the Tang Nano 20K, so the second drive (and the Cubase
dongles) are left out to give the build room. Swap disks in A:.

No FPGA Companion update is needed; the menu entry for the disks comes
from the core.

## What it does

The HFE path works at the level of the real disk, not of sectors:

```
SD card -> track loader -> track player -> flux -> WD1772 data separator
        -> address mark detector -> WD1772 commands -> ST
```

- The track the head is on is loaded from the SD card into a 40 KB buffer
  (both sides), on every step and when a disk is inserted.
- The track is played round and round at its own speed, opcodes of the
  HFE v3 format included, and weak bits come out random on every read.
- A copy of the WD1772's own data separator recovers the bits, and the
  WD1772 commands (seek, read sector, read address, read track, ...) run
  on what comes off the disk, with the same timing, so a protection that
  measures how long a sector takes sees the real thing.

## Limits

- Read only: the disk is write protected. Saving to an HFE disk is not
  possible yet; for saves, swap an `.st` into A:.
- One drive (A:) only.
- Selecting the drive or stepping the head loads the new track from the SD
  card first (a few tens of milliseconds, depending on the card and the
  MCU), a little longer than a real drive takes; the track then plays
  from where the disk is.
- Changing sides keeps the position on the track, so the first byte after
  a side change may read as blank, much like a real drive.

## Test images

`tools/hfev3/st2hfe.py` turns any `.st` into an HFE image with the
standard layout, which reads exactly like the `.st`:

```
tools/hfev3/st2hfe.py disk.st disk.hfe
```

The design and its tests are described in `sim/hfe/README.md` and
`tools/hfev3/`.
