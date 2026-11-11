// hfe_model.cpp - bit-level reference model for HFE (v1/v3) floppy support
// in the MiSTeryNano core.
//
// Copyright (c) 2026 kodak80 and contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// What this models (the same chain the FPGA will implement):
//
//   HFE track stream --> track player --> flux transitions --> WD1772 DPLL
//        (v3 opcodes)    (real time,      (8 MHz cycles)       (patent
//                         cell timer)                          US 4808884)
//   --> recovered cells --> A1/C2 mark detector --> ID/data fields + CRC
//
// The HFEv3 opcode handling follows Steem SSE with HFEv3 (disk_hfe.cpp,
// DecodeV3Track) and the DPLL is a port of Steem SSE's TWD1772Dpll
// (fdc.cpp), which itself follows MAME's wd_fdc. Unlike Steem, the player
// does not normalise cells to a nominal grid: like the hardware it simply
// times each cell with the current BITRATE value, so density changes reach
// the DPLL as real timing.
//
// Time units: the player counts in 1/72 us (HFE cell values are 1/36 us,
// the DPLL clock is 8 MHz = 9 units), so all conversions are exact integers.

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <algorithm>

// ---------------------------------------------------------------- HFE file

static const uint8_t OP_MASK = 0x0F;  // opcodes are bit-reversed in the file
static const uint8_t OP_NOP = 0x0F, OP_INDEX = 0x8F, OP_BITRATE = 0x4F,
                     OP_SKIPBITS = 0xCF, OP_RAND = 0x2F;

static uint8_t rev8(uint8_t x) {
  x = (uint8_t)((x & 0xF0) >> 4 | (x & 0x0F) << 4);
  x = (uint8_t)((x & 0xCC) >> 2 | (x & 0x33) << 2);
  return (uint8_t)((x & 0xAA) >> 1 | (x & 0x55) << 1);
}

struct Hfe {
  std::vector<uint8_t> d;
  bool v3 = false;
  int ntracks = 0, nsides = 0, bitrate = 0, encoding = 0, interface = 0;
  int lut = 0;

  bool load(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    d.resize(n);
    bool ok = fread(d.data(), 1, n, f) == (size_t)n;
    fclose(f);
    if (!ok || n < 1024) return false;
    if (!memcmp(d.data(), "HXCHFEV3", 8)) v3 = true;
    else if (memcmp(d.data(), "HXCPICFE", 8)) return false;
    ntracks = d[9];
    nsides = d[10];
    encoding = d[11];
    bitrate = d[12] | d[13] << 8;
    interface = d[16];
    lut = (d[18] | d[19] << 8) * 512;
    return true;
  }
  int trk_off(int t) const { return (d[lut + 4 * t] | d[lut + 4 * t + 1] << 8) * 512; }
  int trk_len(int t) const { return d[lut + 4 * t + 2] | d[lut + 4 * t + 3] << 8; }

  // raw stream bytes of one side, de-interleaved from the 512-byte blocks
  std::vector<uint8_t> side_stream(int t, int s) const {
    std::vector<uint8_t> out;
    int base = trk_off(t), n = trk_len(t) / 2;
    for (int i = 0; i < n; i++) {
      size_t idx = (size_t)base + (i / 256) * 512 + s * 256 + i % 256;
      if (idx >= d.size()) break;
      out.push_back(d[idx]);
    }
    return out;
  }
};

// ---------------------------------------------------------------- player
//
// Expands a side stream into cells: value, weak flag, duration. This is what
// the FPGA track player produces on the fly from its buffer.

struct Cell {
  uint8_t v, weak;
  uint16_t dur;  // 1/72 us
};

struct Track {
  std::vector<Cell> cells;
  int index_cell = 0;     // where the INDEX opcode is (0 for v1)
  int stream_bytes = 0;   // side stream size: what the FPGA must buffer
  int opcodes = 0, op_bitrate = 0, op_rand = 0, op_skip = 0, op_index = 0;
  int min_x = 999, max_x = 0;
  int64_t rev_time = 0;   // 1/72 us
  int weak_cells = 0;
};

static Track play(const Hfe &h, int t, int s) {
  Track tr;
  std::vector<uint8_t> st = h.side_stream(t, s);
  tr.stream_bytes = (int)st.size();
  int nominal = 72;
  if (h.bitrate >= 50 && h.bitrate <= 2000) nominal = (18000 + h.bitrate / 2) / h.bitrate;
  int x = nominal, skip = 0, index_cell = -1;
  for (size_t i = 0; i < st.size();) {
    uint8_t raw = st[i++];
    uint8_t weak = 0;
    if (h.v3 && (raw & OP_MASK) == OP_MASK) {
      tr.opcodes++;
      switch (raw) {
        case OP_BITRATE:
          tr.op_bitrate++;
          if (i < st.size()) {
            uint8_t v = rev8(st[i++]);
            if (v) x = v;
          }
          skip = 0;
          continue;
        case OP_SKIPBITS:
          tr.op_skip++;
          if (i < st.size()) skip = rev8(st[i++]) & 7;
          continue;
        case OP_INDEX:
          tr.op_index++;
          if (index_cell < 0) index_cell = (int)tr.cells.size();
          skip = 0;
          continue;
        case OP_RAND:
          tr.op_rand++;
          weak = 1;
          break;
        default:  // NOP and unknown opcodes are ignored, as FlashFloppy does
          skip = 0;
          continue;
      }
    }
    for (int k = skip; k < 8; k++) {  // bit 0 is first on the disk
      Cell c;
      c.v = weak ? 0 : (raw >> k) & 1;
      c.weak = weak;
      c.dur = (uint16_t)(2 * x);
      tr.cells.push_back(c);
      tr.rev_time += c.dur;
      tr.weak_cells += weak;
      if (x < tr.min_x) tr.min_x = x;
      if (x > tr.max_x) tr.max_x = x;
    }
    skip = 0;
  }
  tr.index_cell = index_cell < 0 ? 0 : index_cell;
  return tr;
}

// ---------------------------------------------------------------- flux source
//
// Plays the track round and round (like a spinning disk) and returns the
// absolute time of each flux transition in 8 MHz cycles. Weak cells get a
// fresh random value on every pass.

struct FluxSource {
  const Track &tr;
  size_t pos;
  int64_t t72 = 0;            // time at the end of the current cell
  int64_t rev = 0;            // completed revolutions (counted at index)
  uint32_t rng;
  std::vector<int64_t> index_times;  // 1/72 us, each time the index passes

  bool has_flux = false;
  bool real = true;           // false: the last event only marked time, no flux
  FluxSource(const Track &t, uint32_t seed) : tr(t), pos(0), rng(seed ? seed : 1) {
    for (const Cell &c : tr.cells)
      if (c.v || c.weak) { has_flux = true; break; }
  }
  uint32_t rnd() {
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return rng;
  }
  int64_t next_transition() {
    if (!has_flux) {  // unformatted/blank: no flux at all, only keep time going
      real = false;   // a time marker for the DPLL, never a transition
      if (tr.cells.empty()) {
        t72 += 16 * 9;
        return t72 / 9;
      }
      for (size_t n = 0; n < tr.cells.size(); n++) {
        if ((int)pos == tr.index_cell) index_times.push_back(t72);
        t72 += tr.cells[pos].dur;
        pos = (pos + 1) % tr.cells.size();
      }
      return t72 / 9;
    }
    for (;;) {
      if ((int)pos == tr.index_cell) index_times.push_back(t72);
      const Cell &c = tr.cells[pos];
      t72 += c.dur;
      pos = (pos + 1) % tr.cells.size();
      int v = c.weak ? (rnd() & 1) : c.v;
      if (v) return t72 / 9;
    }
  }
};

// ---------------------------------------------------------------- WD1772 DPLL
//
// Port of Steem SSE's TWD1772Dpll::GetNextBit (patent US 4808884, via MAME).
// One slot per 8 MHz cycle, an 11-bit counter, 2048/128 = 16 cycles = 2 us
// per cell at the nominal increment. This is directly synthesisable.

struct Dpll {
  int64_t latest_transition = 0, ctime = 0;
  bool latest_real = false;  // latest_transition is a real flux transition
  uint16_t counter = 0, increment = 128, transition_time = 0xffff;
  uint8_t history = 0x80, slot = 0;
  uint8_t pa = 0, ps = 0, fa = 0, fs = 0;
  int min_inc = 128, max_inc = 128;

  void reset(int64_t when) {
    counter = 0; increment = 128; transition_time = 0xffff; history = 0x80;
    slot = 0; latest_transition = ctime = when; latest_real = false; pa = ps = fa = fs = 0;
  }

  int next_bit(FluxSource &src, int64_t &tm) {
    while (ctime - latest_transition >= 0) {
      latest_transition = src.next_transition();
      latest_real = src.real;
    }
    int64_t when = latest_transition;
    for (;;) {
      int64_t etime = ctime + slot + 1;
      if (transition_time == 0xffff && latest_real && etime - when >= 0) transition_time = counter;
      if (slot < 8) {
        uint8_t m = (uint8_t)(1 << slot);
        if (pa & m) counter += 226;
        else if (ps & m) counter += 30;
        else counter += increment;
        if ((fa & m) && increment < 140) increment++;
        else if ((fs & m) && increment > 117) increment--;
      } else
        counter += increment;
      slot++;
      tm = etime;
      if (counter & 0x800) break;
    }
    int bit = transition_time != 0xffff;
    if (bit) {
      static const uint8_t pha[8] = {0xf, 0x7, 0x3, 0x1, 0, 0, 0, 0};
      static const uint8_t phs[8] = {0, 0, 0, 0, 0x1, 0x3, 0x7, 0xf};
      static const uint8_t freqa[4][8] = {{0xf, 0x7, 0x3, 0x1, 0, 0, 0, 0},
                                          {0x7, 0x3, 0x1, 0, 0, 0, 0, 0},
                                          {0x7, 0x3, 0x1, 0, 0, 0, 0, 0},
                                          {0, 0, 0, 0, 0, 0, 0, 0}};
      static const uint8_t freqs[4][8] = {{0, 0, 0, 0, 0, 0, 0, 0},
                                          {0, 0, 0, 0, 0, 0x1, 0x3, 0x7},
                                          {0, 0, 0, 0, 0, 0x1, 0x3, 0x7},
                                          {0, 0, 0, 0, 0x1, 0x3, 0x7, 0xf}};
      int cslot = transition_time >> 8;
      pa = pha[cslot];
      ps = phs[cslot];
      int way = transition_time & 0x400 ? 1 : 0;
      if (history & 0x80) history = way ? 0x80 : 0x83;
      else if (history & 0x40) history = way ? history & 2 : (history & 2) | 1;
      fa = freqa[history & 3][cslot];
      fs = freqs[history & 3][cslot];
      history = way ? (history >> 1) | 2 : history >> 1;
    } else
      pa = ps = fa = fs = 0;
    counter &= 0x7ff;
    ctime = tm;
    transition_time = 0xffff;
    slot = 0;
    if (increment < min_inc) min_inc = increment;
    if (increment > max_inc) max_inc = increment;
    return bit;
  }
};

// ---------------------------------------------------------------- CRC

static uint16_t crc16(uint16_t crc, uint8_t b) {
  crc ^= (uint16_t)b << 8;
  for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)(crc << 1 ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}

// ---------------------------------------------------------------- decode
//
// Recover N revolutions of cells through the DPLL, then look for marks and
// fields in the cell stream, the way a WD1772 would see them.

struct RecBits {
  std::vector<uint8_t> bit;
  std::vector<int64_t> time;  // 8 MHz cycles at the end of each cell
  std::vector<int64_t> index; // 8 MHz cycles of each index pulse
  int min_inc, max_inc;
};

static RecBits recover(const Track &tr, int revs, uint32_t seed) {
  RecBits r;
  FluxSource src(tr, seed);
  Dpll dpll;
  dpll.reset(0);
  int64_t limit = (tr.rev_time ? tr.rev_time : 200 * 72000LL) * revs / 9;  // cycles
  int64_t tm = 0;
  while (tm < limit) {
    int b = dpll.next_bit(src, tm);
    r.bit.push_back((uint8_t)b);
    r.time.push_back(tm);
  }
  for (int64_t t : src.index_times) r.index.push_back(t / 9);
  r.min_inc = dpll.min_inc;
  r.max_inc = dpll.max_inc;
  return r;
}

struct Mark {
  size_t pos;  // index of the last cell of the mark
  int type;    // 1 = A1 (0x4489), 2 = C2 (0x5224)
};

static std::vector<Mark> find_marks(const RecBits &r) {
  std::vector<Mark> m;
  uint32_t sh = 0;
  size_t last_a1 = 0;
  bool have_a1 = false;
  for (size_t i = 0; i < r.bit.size(); i++) {
    sh = sh << 1 | r.bit[i];
    if ((sh & 0xffff) == 0x4489) {
      // an A1 can't overlap the previous A1 (16 cells), as in the WD1772
      if (!have_a1 || i - last_a1 >= 16) {
        m.push_back({i, 1});
        last_a1 = i;
        have_a1 = true;
      }
    } else if ((sh & 0xffff) == 0x5224)
      m.push_back({i, 2});
  }
  return m;
}

// decode the byte whose 16 cells end at cell index 'end' (inclusive)
static int byte_at(const RecBits &r, size_t end) {
  if (end >= r.bit.size() || end < 15) return -1;
  int v = 0;
  for (int k = 0; k < 8; k++) v = v << 1 | r.bit[end - 14 + 2 * k];  // data cells
  return v;
}

struct Field {
  size_t am_pos;     // cell index of the mark byte (FE / FB...) end
  int64_t t_us;      // microseconds from index
  uint8_t trk, side, sec, len;
  bool id_crc_ok;
  int dam = -1;      // FB / F8 ..., -1 = no data field found
  bool data_crc_ok = false;
  int64_t data_us = 0;  // time to read the data field
  std::vector<uint8_t> data;
  bool nested = false;  // this ID lies inside another sector's data field
};

static int64_t us_of(const RecBits &r, size_t pos, int64_t idx) {
  return (r.time[pos] - idx) / 8;
}

static std::vector<Field> decode_fields(const RecBits &r, const std::vector<Mark> &marks,
                                        int64_t from, int64_t to) {
  std::vector<Field> out;
  // find runs of three A1 marks 16 cells apart, then the mark byte
  for (size_t i = 0; i + 2 < marks.size(); i++) {
    if (marks[i].type != 1 || marks[i + 1].type != 1 || marks[i + 2].type != 1) continue;
    if (marks[i + 1].pos - marks[i].pos != 16 || marks[i + 2].pos - marks[i + 1].pos != 16) continue;
    size_t p = marks[i + 2].pos + 16;
    int am = byte_at(r, p);
    if (am != 0xFE) continue;
    if (r.time[p] < from || r.time[p] >= to) continue;
    Field f{};
    f.am_pos = p;
    f.t_us = us_of(r, p, from);
    uint16_t crc = 0xffff;
    crc = crc16(crc, 0xA1); crc = crc16(crc, 0xA1); crc = crc16(crc, 0xA1); crc = crc16(crc, 0xFE);
    int b[6];
    bool ok = true;
    for (int k = 0; k < 6; k++) {
      b[k] = byte_at(r, p + 16 * (k + 1));
      if (b[k] < 0) ok = false;
    }
    if (!ok) continue;
    for (int k = 0; k < 4; k++) crc = crc16(crc, (uint8_t)b[k]);
    f.trk = (uint8_t)b[0]; f.side = (uint8_t)b[1]; f.sec = (uint8_t)b[2]; f.len = (uint8_t)b[3];
    f.id_crc_ok = crc == (uint16_t)(b[4] << 8 | b[5]);
    // data mark must start within 43 bytes of the ID CRC (WD1772, MFM)
    size_t id_end = p + 16 * 6;
    for (size_t j = i + 3; j + 2 < marks.size(); j++) {
      if (marks[j].pos <= id_end) continue;
      if (marks[j].pos > id_end + 16 * (43 + 3)) break;
      if (marks[j].type != 1 || marks[j + 1].type != 1 || marks[j + 2].type != 1) continue;
      if (marks[j + 1].pos - marks[j].pos != 16 || marks[j + 2].pos - marks[j + 1].pos != 16) continue;
      size_t q = marks[j + 2].pos + 16;
      int dam = byte_at(r, q);
      if (dam < 0xF8 || dam > 0xFB) {
        if (dam == 0xFE) break;  // next ID: no data for this one
        continue;
      }
      f.dam = dam;
      int size = 128 << (f.len & 3);
      uint16_t c2 = 0xffff;
      c2 = crc16(c2, 0xA1); c2 = crc16(c2, 0xA1); c2 = crc16(c2, 0xA1); c2 = crc16(c2, (uint8_t)dam);
      bool dok = true;
      for (int k = 0; k < size; k++) {
        int v = byte_at(r, q + 16 * (k + 1));
        if (v < 0) { dok = false; break; }
        f.data.push_back((uint8_t)v);
        c2 = crc16(c2, (uint8_t)v);
      }
      int c_hi = byte_at(r, q + 16 * (size + 1)), c_lo = byte_at(r, q + 16 * (size + 2));
      if (dok && c_hi >= 0 && c_lo >= 0) {
        f.data_crc_ok = c2 == (uint16_t)(c_hi << 8 | c_lo);
        f.data_us = (r.time[q + 16 * (size + 2)] - r.time[q]) / 8;
      }
      break;
    }
    out.push_back(f);
  }
  // IDs that sit inside another sector's data field
  for (auto &a : out) {
    if (a.dam < 0) continue;
    size_t size = 128u << (a.len & 3);
    for (auto &b : out)
      if (&a != &b && b.am_pos > a.am_pos + 16 * 7 && b.am_pos < a.am_pos + 16 * (50 + size + 3))
        b.nested = true;
  }
  return out;
}

// ---------------------------------------------------------------- read track
//
// What a WD1772 READ TRACK returns: one byte per 16 cells from index to
// index, re-framed whenever the AM detector sees an A1 (0x4489) or C2
// (0x5224) mark. Marks are flagged so a dump shows where re-framing happened.

// Each entry is a byte | mark << 8 (mark: 0 none, 1 A1, 2 C2) and the time of
// its last cell in 8 MHz cycles.
struct RtByte {
  int v;
  int64_t time;
};

static std::vector<RtByte> read_track(const RecBits &r, int64_t from, int64_t to) {
  std::vector<RtByte> out;
  uint32_t sh = 0;
  int cnt = 0;
  size_t last_a1 = 0;
  bool have_a1 = false;
  for (size_t i = 0; i < r.bit.size(); i++) {
    sh = sh << 1 | r.bit[i];
    if (r.time[i] < from) continue;
    if (r.time[i] >= to) break;
    int mark = 0;
    if ((sh & 0xffff) == 0x4489 && (!have_a1 || i - last_a1 >= 16)) { mark = 1; last_a1 = i; have_a1 = true; }
    else if ((sh & 0xffff) == 0x5224) mark = 2;
    if (mark || ++cnt == 16) {
      int v = 0;
      for (int k = 0; k < 8; k++) v = v << 1 | (int)((sh >> (14 - 2 * k)) & 1);
      out.push_back({v | mark << 8, r.time[i]});
      cnt = 0;
    }
  }
  return out;
}

static void dump_read_track(const RecBits &r, int64_t from, int64_t to) {
  int col = 0;
  for (const RtByte &b : read_track(r, from, to)) {
    if (col == 0) printf("%6lld: ", (long long)((b.time - from) / 8));
    int mark = b.v >> 8;
    printf("%02X%c", b.v & 0xff, mark == 1 ? '\'' : mark == 2 ? '"' : ' ');
    if (++col == 32) { printf("\n"); col = 0; }
  }
  if (col) printf("\n");
}

// The -s listing of one side: every ID/data field whose ID mark is in the
// second revolution, in a fixed format, sorted. Weak bits use 'seed'.
static std::vector<std::string> sector_lines(const Hfe &h, int t, int s, uint32_t seed) {
  Track tr = play(h, t, s);
  RecBits r = recover(tr, 3, seed);
  std::vector<Mark> m = find_marks(r);
  int64_t a = r.index.size() > 1 ? r.index[1] : 0, b = r.index.size() > 2 ? r.index[2] : INT64_MAX;
  std::vector<std::string> lines;
  for (auto &f : decode_fields(r, m, a, b)) {
    unsigned hsh = 2166136261u;
    for (uint8_t v : f.data) { hsh ^= v; hsh *= 16777619u; }
    char ln[160];
    snprintf(ln, sizeof ln, "T%02d.%d id %d.%d.%d.%d %s dam %02X %s %08x", t, s, f.trk, f.side, f.sec,
             f.len, f.id_crc_ok ? "ok" : "BAD", f.dam & 0xff,
             f.dam < 0 ? "-" : (f.data_crc_ok ? "ok" : "BAD"), f.dam < 0 ? 0 : hsh);
    lines.push_back(ln);
  }
  std::sort(lines.begin(), lines.end());
  lines.erase(std::unique(lines.begin(), lines.end()), lines.end());
  return lines;
}

// ---------------------------------------------------------------- report

struct DiskStats {
  int max_stream = 0, max_cells = 0;
  int64_t max_rev = 0, min_rev = INT64_MAX;
  int min_x = 999, max_x = 0;
  int max_opcodes = 0;
  long weak_cells = 0;
  int sides_density = 0, sides_weak = 0, sides_long = 0, sides_blank = 0;
  int bad_id_crc = 0, bad_data_crc = 0, deleted = 0, no_data = 0, odd_size = 0,
      odd_track = 0, odd_sector = 0, dup = 0, nested = 0, weak_sectors = 0, c2 = 0;
  int min_dpll_inc = 128, max_dpll_inc = 128;
  int formatted_beyond_79 = 0;
};

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr,
            "usage: %s [-v] [-t track] [-r track side] image.hfe ...\n"
            "  Decodes every track side through the HFE player + WD1772 DPLL model\n"
            "  and reports the protection features the FPGA controller must handle.\n"
            "  -v  list every side and every ID field\n"
            "  -t  only this track\n"
            "  -s  list every ID/data field in a fixed format (for diffing decoders)\n"
            "  -r  dump what READ TRACK returns for one side (' = A1 mark, \" = C2 mark,\n"
            "      left column = microseconds from index)\n",
            argv[0]);
    return 1;
  }
  bool verbose = false, sector_dump = false;
  int only_track = -1, rt_track = -1, rt_side = 0;
  int argi = 1;
  for (; argi < argc && argv[argi][0] == '-'; argi++) {
    if (!strcmp(argv[argi], "-v")) verbose = true;
    else if (!strcmp(argv[argi], "-s")) sector_dump = true;
    else if (!strcmp(argv[argi], "-t") && argi + 1 < argc) only_track = atoi(argv[++argi]);
    else if (!strcmp(argv[argi], "-r") && argi + 2 < argc) {
      rt_track = atoi(argv[++argi]);
      rt_side = atoi(argv[++argi]);
    }
  }
  if (sector_dump) {
    // machine-readable list of every field, for diffing against other decoders
    for (; argi < argc; argi++) {
      Hfe h;
      if (!h.load(argv[argi])) continue;
      for (int t = 0; t < h.ntracks; t++)
        for (int s = 0; s < h.nsides; s++)
          for (auto &l : sector_lines(h, t, s, 0x1234567u + t * 7 + s)) puts(l.c_str());
    }
    return 0;
  }
  if (rt_track >= 0) {
    for (; argi < argc; argi++) {
      Hfe h;
      if (!h.load(argv[argi]) || rt_track >= h.ntracks || rt_side >= h.nsides) {
        printf("%s: not an HFE file or no such track\n", argv[argi]);
        continue;
      }
      Track tr = play(h, rt_track, rt_side);
      RecBits r = recover(tr, 3, 0x1234567u);
      int64_t a = r.index.size() > 1 ? r.index[1] : 0, b = r.index.size() > 2 ? r.index[2] : INT64_MAX;
      printf("==== %s T%02d.%d read track\n", argv[argi], rt_track, rt_side);
      dump_read_track(r, a, b);
    }
    return 0;
  }
  for (; argi < argc; argi++) {
    Hfe h;
    if (!h.load(argv[argi])) {
      printf("%s: not an HFE file\n", argv[argi]);
      continue;
    }
    const char *name = strrchr(argv[argi], '/');
    name = name ? name + 1 : argv[argi];
    printf("==== %s\n", name);
    printf("HFE %s, %d tracks, %d sides, %d kbps, encoding %d, interface %d\n",
           h.v3 ? "v3" : "v1", h.ntracks, h.nsides, h.bitrate, h.encoding, h.interface);
    printf("Sides listed only when unusual (-v lists all). Sector flags: /size, @T.S ID differs from\n"
           "the physical track, !id ID CRC bad, !dat data CRC bad, - no data field, del deleted mark,\n"
           "W weak (reads differently each pass), ~pct data rate vs 250kbps, ^ ID inside another\n"
           "sector's data, * duplicate number.\n\n");
    DiskStats ds;
    for (int t = 0; t < h.ntracks; t++) {
      if (only_track >= 0 && t != only_track) continue;
      for (int s = 0; s < h.nsides; s++) {
        Track tr = play(h, t, s);
        ds.max_stream = std::max(ds.max_stream, tr.stream_bytes);
        ds.max_cells = std::max(ds.max_cells, (int)tr.cells.size());
        ds.max_opcodes = std::max(ds.max_opcodes, tr.opcodes);
        ds.weak_cells += tr.weak_cells;
        if (tr.weak_cells) ds.sides_weak++;
        if (!tr.cells.empty()) {
          ds.max_rev = std::max(ds.max_rev, tr.rev_time);
          ds.min_rev = std::min(ds.min_rev, tr.rev_time);
          ds.min_x = std::min(ds.min_x, tr.min_x);
          ds.max_x = std::max(ds.max_x, tr.max_x);
        }
        if (tr.max_x - tr.min_x > 2) ds.sides_density++;

        // two independent decodes: weak bits get different random values
        RecBits r1 = recover(tr, 3, 0x1234567u + t * 7 + s);
        RecBits r2 = recover(tr, 3, 0x89abcdeu + t * 13 + s);
        ds.min_dpll_inc = std::min(ds.min_dpll_inc, r1.min_inc);
        ds.max_dpll_inc = std::max(ds.max_dpll_inc, r1.max_inc);
        std::vector<Mark> m1 = find_marks(r1), m2 = find_marks(r2);
        int64_t i1a = r1.index.size() > 1 ? r1.index[1] : 0, i1b = r1.index.size() > 2 ? r1.index[2] : INT64_MAX;
        int64_t i2a = r2.index.size() > 1 ? r2.index[1] : 0, i2b = r2.index.size() > 2 ? r2.index[2] : INT64_MAX;
        std::vector<Field> f1 = decode_fields(r1, m1, i1a, i1b);
        std::vector<Field> f2 = decode_fields(r2, m2, i2a, i2b);
        // index address marks: C2 C2 C2 FC (a lone 0x5224 also shows up
        // misaligned inside ordinary data, so only count real marks)
        int c2 = 0;
        for (size_t k = 0; k + 2 < m1.size(); k++)
          if (m1[k].type == 2 && m1[k + 1].type == 2 && m1[k + 2].type == 2 &&
              m1[k + 1].pos - m1[k].pos == 16 && m1[k + 2].pos - m1[k + 1].pos == 16 &&
              byte_at(r1, m1[k + 2].pos + 16) == 0xFC && r1.time[m1[k].pos] >= i1a && r1.time[m1[k].pos] < i1b)
            c2++;
        if (c2) ds.c2++;

        double rev_ms = tr.rev_time / 72000.0;
        bool is_long = rev_ms > 202.0;
        if (is_long) ds.sides_long++;
        if (f1.empty()) ds.sides_blank++;
        if (t >= 80 && !f1.empty()) ds.formatted_beyond_79++;

        // one token per sector, in disk order, with flags for anything unusual:
        //   <sec>[/size if not 512][@T.S if ID differs][!id][!dat][-][del][W][~pct][^][*]
        std::string toks;
        std::set<int> seen;
        bool odd = false;
        for (auto &f : f1) {
          char b[96];
          std::string tk;
          snprintf(b, sizeof b, "%d", f.sec);
          tk = b;
          if ((f.len & 3) != 2) { ds.odd_size++; snprintf(b, sizeof b, "/%d", 128 << (f.len & 3)); tk += b; odd = true; }
          if (f.trk != t || f.side != s) { ds.odd_track++; snprintf(b, sizeof b, "@%d.%d", f.trk, f.side); tk += b; odd = true; }
          if (f.sec < 1 || f.sec > 11) { ds.odd_sector++; odd = true; }
          if (!f.id_crc_ok) { ds.bad_id_crc++; tk += "!id"; odd = true; }
          if (f.dam < 0) { ds.no_data++; tk += "-"; odd = true; }
          else if (!f.data_crc_ok) { ds.bad_data_crc++; tk += "!dat"; odd = true; }
          if (f.dam == 0xF8) { ds.deleted++; tk += "del"; odd = true; }
          for (auto &g : f2)  // weak: reads differently on another pass
            if (g.sec == f.sec && g.trk == f.trk && std::llabs(g.t_us - f.t_us) < 2000 && g.dam >= 0 && f.dam >= 0) {
              if (g.data != f.data || g.data_crc_ok != f.data_crc_ok) { ds.weak_sectors++; tk += "W"; odd = true; }
              break;
            }
          if (f.dam >= 0 && f.data_us) {  // a 512 byte sector takes 16.5 ms at 2 us cells
            double ratio = f.data_us / ((128.0 * (1 << (f.len & 3)) + 3) * 32.0);
            if (ratio < 0.97 || ratio > 1.03) { snprintf(b, sizeof b, "~%.0f%%", 100.0 / ratio); tk += b; odd = true; }
          }
          if (f.nested) { ds.nested++; tk += "^"; odd = true; }
          if (seen.count(f.sec)) { ds.dup++; tk += "*"; odd = true; }
          seen.insert(f.sec);
          if (!toks.empty()) toks += " ";
          toks += tk;
        }
        std::string extra;
        if (c2) { char b[64]; snprintf(b, sizeof b, "  idxmark %d", c2); extra += b; }
        if (tr.weak_cells) { char b[64]; snprintf(b, sizeof b, "  weakcells %d", tr.weak_cells); extra += b; }
        if (is_long) extra += "  LONG";
        if (f1.empty()) extra += "  (no sectors)";

        bool interesting = odd || c2 || tr.weak_cells || is_long || tr.max_x - tr.min_x > 2;
        if (verbose || interesting) {
          printf("T%02d.%d %6.2fms %6d cells x%d-%d %5dB ops%5d | %s%s\n", t, s, rev_ms, (int)tr.cells.size(),
                 tr.min_x, tr.max_x, tr.stream_bytes, tr.opcodes, toks.c_str(), extra.c_str());
          if (verbose)
            for (auto &f : f1)
              printf("        id T%d.%d s%d len%d @%6lldus crc %s  dam %02X data %s %lldus\n", f.trk, f.side, f.sec,
                     f.len, (long long)f.t_us, f.id_crc_ok ? "ok" : "BAD", f.dam & 0xff,
                     f.dam < 0 ? "-" : (f.data_crc_ok ? "ok" : "BAD"), (long long)f.data_us);
        }
      }
    }
    printf("\nSummary for %s\n", name);
    printf("  buffer: largest side stream %d bytes, largest revolution %d cells\n", ds.max_stream, ds.max_cells);
    printf("  rotation: %.2f - %.2f ms per revolution (200 ms nominal), %d long sides\n",
           ds.min_rev / 72000.0, ds.max_rev / 72000.0, ds.sides_long);
    printf("  cell size: x=%d - %d (72 = 2.00us), %d sides with density changes, max %d opcodes/side\n",
           ds.min_x, ds.max_x, ds.sides_density, ds.max_opcodes);
    printf("  DPLL increment range: %d - %d (128 nominal, limits 117-140)\n", ds.min_dpll_inc, ds.max_dpll_inc);
    printf("  weak bits: %ld cells on %d sides, %d sectors read differently each time\n", ds.weak_cells,
           ds.sides_weak, ds.weak_sectors);
    printf("  sectors: %d ID CRC errors, %d data CRC errors, %d without data, %d deleted, %d odd size,\n"
           "           %d wrong track/side in ID, %d odd sector numbers, %d duplicates, %d nested IDs\n",
           ds.bad_id_crc, ds.bad_data_crc, ds.no_data, ds.deleted, ds.odd_size, ds.odd_track, ds.odd_sector,
           ds.dup, ds.nested);
    printf("  other: %d sides with index marks, %d unformatted sides, %d formatted sides beyond track 79\n\n",
           ds.c2, ds.sides_blank, ds.formatted_beyond_79);
  }
  return 0;
}
