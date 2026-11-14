# HFE floppy images

Besides plain `.st` images, drive A: takes HFE images (HxC
Floppy Emulator format, v1 and v3), so copy protected disks can be used:
fuzzy (weak) bits, sectors written at a different speed, extra or
missing sectors, unformatted tracks and the like are all played back as
they are on the disk.

This is the `hfev3-fdc-v1.2.10` branch: MiSTeryNano v1.2.10 for the Tang
Nano 20K with the HFE drive added.

## Use

Select an `.hfe` file for Disk A: in the menu, as you would an `.st`, and
reset if the disk boots. The core looks at the file itself: a file with
the HFE signature goes through the HFE floppy path, anything else through
the existing `.st` floppy.

- To change disks, select the next one for Disk A:. The drive reports the
  change the way a real one does (the write protect signal changes for a
  moment), so games and TOS notice the new disk.
- Games that start from an `AUTO` folder (The Chaos Engine, Pushover) only
  start by themselves when A: is the boot drive. With an ACSI hard disk
  mounted its driver makes C: the boot drive: remove the ACSI images (or
  hold Alternate at reset, which most hard disk drivers honour), or run
  the program from A:'s `AUTO` folder. Disks with a boot sector that
  starts the game (Lemmings, R-Type, Toki, ...) are not affected.
- Keep the image files contiguous on the SD card (a file copied onto a
  card normally is). The core then reads the tracks from the card itself;
  a fragmented file still works, but every block goes through the MCU and
  games that time their disk access closely (Lemmings) may stall.

No FPGA Companion update is needed; the menu entries for the disks come
from the core.

## What is left out to make room

The HFE path and the `.st` floppy together do not fit the Tang Nano 20K
with everything else, so this build leaves out:

- **Drive B:.** Selecting B: finds a drive without a disk (TOS boots as
  quickly as before). The menu has no Disk B: entry and Disk prot. offers
  None or A:.
- **The Cubase 2/3 dongles.** The cartridge port reads $FF. Build with
  `CUBASE_DONGLE` defined to have them back.

ACSI hard disks, the Mega ST (blitter) and the STE modes are all kept.

## Tested

On a Tang Nano 20K with a MiSTeryShield20k and a Pico running FPGA
Companion, TOS 1.04, 4 MB, HFE v1 and v3 images in general load and play
in A:. A few that showed something worth knowing:

- Lemmings times its disk access closely: it needs the image contiguous
  on the SD card (see above).
- Pushover has tracks of nearly 48 KB, the size of the track buffer.
- The Chaos Engine and Pushover start from an `AUTO` folder: no ACSI hard
  disk, or A: is not the boot drive (see above).

## What it does

The HFE path works at the level of the real disk, not of sectors:

```
SD card -> track loader -> track player -> flux -> WD1772 data separator
        -> address mark detector -> WD1772 commands -> ST
```

- The track the head is on is loaded from the SD card into a 48 KB buffer
  (both sides), on every step and when a disk is inserted. It is read
  straight from the card when the image is contiguous.
- A loaded track starts playing at once, brought round to where the disk
  is (it plays from the index at 16 times the speed, unseen, until it has
  caught up with the time since the index), so a seek with verify finds
  an ID field as soon as on a real drive and the index keeps its period.
- The track is played round and round at its own speed, opcodes of the
  HFE v3 format included, and weak bits come out random on every read.
- A copy of the WD1772's own data separator recovers the bits, and the
  WD1772 commands (seek, read sector, read address, read track, ...) run
  on what comes off the disk, with the same timing, so a protection that
  measures how long a sector takes sees the real thing.

## Limits

- Read only: the disk is write protected and writes are refused. Saving
  to an HFE disk is not possible yet; for saves, swap an `.st` into A:.
- One drive (A:) only.
- After a step the new track is loaded first (a few milliseconds from a
  contiguous image, tens through the MCU), a little longer than a real
  drive's head settling time.
- Changing sides keeps the position on the track, so the first byte after
  a side change may read as blank, much like a real drive.

## Building

Build with the command line flow, from `src`:

```
gw_sh build.tcl
```

Gowin V1.9.11.03 Education works (it is the Education version Gowin
offers now). Earlier builds of this branch with it came out with a dead
keyboard or a core that did not start TOS, depending on the build; that
was the link to the MCU (`misc/mcu_spi.v`) bringing the MCU's bytes into
the core's clock without proper synchronizers, now fixed.

## Test images

`tools/hfev3/st2hfe.py` turns any `.st` into an HFE image with the
standard layout, which reads exactly like the `.st`:

```
tools/hfev3/st2hfe.py disk.st disk.hfe
```

The design and its tests are described in `sim/hfe/README.md` and
`tools/hfev3/`. The full core simulation (`sim/atari_tb`) can trace the
floppy controller (`make VDEFS=+define+FDC_TRACE`) and press a key at a
given time (`KEYAT=ms`).
