// tb_floppy.cpp - Verilator test of the whole HFE floppy path: image header
// and track table from the SD card, the track loader, heads, rotation and
// side switching (hfe_floppy.v), through the DPLL, mark detector and
// WD1772 controller, against tools/hfev3/hfe_model.cpp
//
//   ./tb_floppy [-t track] [-q] image.hfe [other.hfe] [not_hfe.st]
//
// The testbench is the CPU and the SD card (serving blocks of the image
// files with the core's handshake). The first image goes into drive A:
//   - mount: the header and track table are read, the drive becomes valid
//   - every track: SEEK with verify (seek error only where the model has
//     no good ID for that track on side 0), then on each side READ ADDRESS
//     (an ID of the model's) and READ SECTOR of every track/sector pair
//     with data (the model's data, CRC error and deleted bits)
//   - index pulses: every revolution, including those across seeks and
//     loads, within 5% of the track's own revolution
//   - side switch in the middle of a READ ADDRESS
// A second HFE goes into drive B (reads from it, then from A again); a
// non-HFE image must not become an HFE drive.

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <memory>
#include "ref.h"
#include "verilated.h"
#include "Vfloppy_tb_top.h"

static bool quiet = false;
static const char *swap_to = nullptr;   // -c: disk change in drive A to this image

static const char *base(const char *p) {
  const char *s = strrchr(p, '/');
  return s ? s + 1 : p;
}
static unsigned fnv(const std::vector<uint8_t> &d) {
  unsigned h = 2166136261u;
  for (uint8_t v : d) { h ^= v; h *= 16777619u; }
  return h;
}
static bool load_file(const char *p, std::vector<uint8_t> &d) {
  FILE *f = fopen(p, "rb");
  if (!f) return false;
  fseek(f, 0, SEEK_END);
  d.resize(ftell(f));
  fseek(f, 0, SEEK_SET);
  bool ok = fread(d.data(), 1, d.size(), f) == d.size();
  fclose(f);
  return ok;
}

// ------------------------------------------------------------------ model

struct SideModel {
  std::vector<Field> fl;
  std::vector<bool> weak;
  int64_t rev_cycles = 0;
};

static SideModel model_side(const Hfe &h, int t, int s) {
  SideModel m;
  Track tr = play(h, t, s);
  m.rev_cycles = tr.rev_time / 9;
  if (tr.cells.empty()) return m;
  uint32_t seed = 0x1234567u + t * 7 + s;
  RecBits rb = recover(tr, 3, seed);
  std::vector<Mark> mk = find_marks(rb);
  m.fl = decode_fields(rb, mk, rb.index.size() > 1 ? rb.index[1] : 0, rb.index.size() > 2 ? rb.index[2] : INT64_MAX);
  RecBits rb2 = recover(tr, 3, seed ^ 0x5a5a5a5au);
  std::vector<Mark> mk2 = find_marks(rb2);
  std::vector<Field> fl2 = decode_fields(rb2, mk2, rb2.index.size() > 1 ? rb2.index[1] : 0,
                                         rb2.index.size() > 2 ? rb2.index[2] : INT64_MAX);
  m.weak.assign(m.fl.size(), false);
  for (size_t i = 0; i < m.fl.size(); i++)
    for (auto &g : fl2)
      if (g.sec == m.fl[i].sec && g.trk == m.fl[i].trk && std::llabs(g.t_us - m.fl[i].t_us) < 2000 && g.dam >= 0 &&
          m.fl[i].dam >= 0) {
        if (g.data != m.fl[i].data || g.data_crc_ok != m.fl[i].data_crc_ok) m.weak[i] = true;
        break;
      }
  return m;
}

// ------------------------------------------------------------------ sim

struct Sim {
  Vfloppy_tb_top *top;
  std::vector<uint8_t> file[2];
  int64_t clocks = 0;

  // SD card: a request is taken on the rising edge of sd_rd, answered after
  // ~100 us, then 512 bytes at one per 8 clocks (~4 MB/s)
  int sd_state = 0, sd_drive = 0, sd_byte = 0;
  uint32_t sd_lba = 0;
  int64_t sd_t = 0;
  int prev_rd = 0;
  long blocks = 0, blocks_b = 0;   // blocks read, those for drive B

  std::vector<int64_t> index;     // clock of each index pulse
  int last_head = -1;
  std::vector<std::pair<int64_t, int>> head_at;  // clock, head

  explicit Sim(Vfloppy_tb_top *t) : top(t) {}

  void tick() {
    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
    clocks++;
    if (top->index) index.push_back(clocks);
    if (top->head_track != last_head) {
      last_head = top->head_track;
      head_at.push_back({clocks, last_head});
    }
    // SD card
    top->sd_dout_strobe = 0;
    int rd = top->sd_rd;
    switch (sd_state) {
      case 0:
        if ((rd & ~prev_rd) != 0) {
          sd_drive = (rd & 1) ? 0 : 1;
          sd_lba = top->sd_lba;
          sd_t = clocks + 3200;
          sd_state = 1;
        }
        break;
      case 1:
        if (clocks >= sd_t) {
          top->sd_ack = 1;
          sd_byte = 0;
          sd_t = clocks + 8;
          sd_state = 2;
        }
        break;
      case 2:
        if (clocks >= sd_t) {
          size_t off = (size_t)sd_lba * 512 + sd_byte;
          top->sd_buff_addr = sd_byte;
          top->sd_dout = off < file[sd_drive].size() ? file[sd_drive][off] : 0;
          top->sd_dout_strobe = 1;
          sd_t = clocks + 8;
          if (++sd_byte == 512) sd_state = 3;
        }
        break;
      case 3:
        if (clocks >= sd_t) {
          top->sd_ack = 0;
          blocks++;
          if (sd_drive == 1) blocks_b++;
          sd_state = 0;
        }
        break;
    }
    prev_rd = rd;
  }
  void ticks(int64_t k) { for (int64_t i = 0; i < k; i++) tick(); }

  void write(int reg, int v) {
    top->cpu_addr = reg; top->cpu_din = v; top->cpu_rw = 0; top->cpu_sel = 1;
    ticks(4);
    top->cpu_sel = 0; top->cpu_rw = 1;
    ticks(4);
  }
  int read(int reg) {
    top->cpu_addr = reg; top->cpu_rw = 1; top->cpu_sel = 1;
    top->eval();
    int v = top->cpu_dout;
    ticks(4);
    top->cpu_sel = 0;
    ticks(4);
    return v;
  }

  struct Run {
    bool timeout = false;
    int status = 0;
    std::vector<uint8_t> data;
  };
  // run a command until IRQ, taking DRQ bytes; 'during' is called every clock
  template <class F>
  Run command(int cmd, int64_t max_clocks, F during) {
    Run r;
    write(0, cmd);
    int64_t t0 = clocks, serve_at = -1;
    bool drq_prev = false;
    while (!top->irq) {
      if (clocks - t0 > max_clocks) { r.timeout = true; break; }
      tick();
      during(clocks - t0);
      if (top->drq && !drq_prev) serve_at = clocks + 16;
      drq_prev = top->drq;
      if (serve_at >= 0 && clocks >= serve_at) {
        serve_at = -1;
        r.data.push_back((uint8_t)read(3));
        drq_prev = top->drq;
      }
    }
    if (!r.timeout && top->drq) {
      ticks(16);
      r.data.push_back((uint8_t)read(3));
    }
    r.status = read(0);
    return r;
  }
  Run command(int cmd, int64_t max_clocks = 32000000LL * 3) {
    return command(cmd, max_clocks, [](int64_t) {});
  }

  bool mount(int drive, int64_t max_clocks = 32000 * 200) {
    top->img_size = (uint32_t)file[drive].size();
    top->img_mounted = 1 << drive;
    tick();
    top->img_mounted = 0;
    int64_t t0 = clocks;
    while (clocks - t0 < max_clocks) {
      tick();
      if (top->hfe_valid & (1 << drive)) return true;
    }
    return false;
  }
};

// ------------------------------------------------------------------ checks

struct Result {
  long tracks = 0, sides = 0, fail = 0, sectors = 0, rnf_skipped = 0, seek_err = 0;
  double min_rev = 1e9, max_rev = 0;
};

static bool fails_any = false;
#define FAILF(...) do { printf("FAIL %s T%02d.%d: ", name, t, s); printf(__VA_ARGS__); printf("\n"); ok = false; fails_any = true; } while (0)

// does the RTL's sector match a model field of this track/sector?
static bool match_sector(const SideModel &m, int trk, int sec, const Sim::Run &r) {
  unsigned hr = fnv(r.data);
  bool crc_bad = r.status & 0x08, deleted = r.status & 0x20;
  for (size_t i = 0; i < m.fl.size(); i++) {
    const Field &f = m.fl[i];
    if (!f.id_crc_ok || f.trk != trk || f.sec != sec || f.dam < 0) continue;
    bool data_ok = (m.weak[i] && !f.data_crc_ok) ? crc_bad : (fnv(f.data) == hr && crc_bad == !f.data_crc_ok);
    if (data_ok && r.data.size() == f.data.size() && deleted == (f.dam <= 0xF9)) return true;
  }
  return false;
}

static bool test_side(Sim &sim, const Hfe &h, int t, int s, const char *name, Result &res) {
  bool ok = true;
  SideModel m = model_side(h, t, s);
  sim.top->side = s;
  sim.ticks(64);
  res.sides++;

  std::set<std::string> ids;
  for (auto &f : m.fl) {
    char b[64];
    snprintf(b, sizeof b, "%d.%d.%d.%d %s", f.trk, f.side, f.sec, f.len, f.id_crc_ok ? "ok" : "BAD");
    ids.insert(b);
  }
  // READ ADDRESS
  Sim::Run r = sim.command(0xC8);
  if (r.timeout) FAILF("read address timeout");
  else if (ids.empty()) {
    if (!(r.status & 0x10)) FAILF("read address on a blank side: status %02X", r.status);
  } else {
    char b[64];
    snprintf(b, sizeof b, "%d.%d.%d.%d %s", r.data.size() > 3 ? r.data[0] : -1, r.data.size() > 3 ? r.data[1] : -1,
             r.data.size() > 3 ? r.data[2] : -1, r.data.size() > 3 ? r.data[3] : -1, (r.status & 0x08) ? "BAD" : "ok");
    if ((r.status & 0x14) || r.data.size() != 6 || !ids.count(b)) FAILF("read address %s status %02X", b, r.status);
  }

  // READ SECTOR of every pair with data
  std::set<int> done;
  for (auto &f : m.fl) {
    if (!f.id_crc_ok || done.count(f.trk << 8 | f.sec)) continue;
    done.insert(f.trk << 8 | f.sec);
    bool has_data = false;
    for (auto &g : m.fl) has_data |= g.id_crc_ok && g.trk == f.trk && g.sec == f.sec && g.dam >= 0;
    if (!has_data) { res.rnf_skipped++; continue; }
    sim.write(1, f.trk);
    sim.write(2, f.sec);
    r = sim.command(0x88);
    res.sectors++;
    if (r.timeout) { FAILF("sector %d.%d timeout", f.trk, f.sec); continue; }
    if ((r.status & 0x14) || !match_sector(m, f.trk, f.sec, r))
      FAILF("sector %d.%d status %02X, %zu bytes %08x", f.trk, f.sec, r.status, r.data.size(), fnv(r.data));
  }
  return ok;
}

static void run_image(Sim &sim, const Hfe &h, const char *name, int only_t, Result &res) {
  bool ok;
  int t = 0, s = 0;
  sim.top->floppy_drive = 2;   // drive A
  sim.top->side = 0;
  sim.write(0, 0xD0);
  sim.ticks(64);
  Sim::Run r = sim.command(0x08);   // restore, h, no verify
  ok = true;
  if (r.timeout || sim.top->head_track != 0) FAILF("restore: status %02X head %d", r.status, sim.top->head_track);

  for (t = 0; t < h.ntracks; t++) {
    if (only_t >= 0 && t != only_t) continue;
    s = 0;
    ok = true;
    // SEEK with verify on side 0
    SideModel m0 = model_side(h, t, 0);
    bool expect_ok = false;
    for (auto &f : m0.fl) expect_ok |= f.id_crc_ok && f.trk == t;
    sim.top->side = 0;
    sim.write(3, t);
    size_t idx0 = sim.index.size();
    r = sim.command(0x1C);
    res.tracks++;
    if (r.timeout) FAILF("seek timeout");
    else if (sim.top->head_track != t || sim.read(1) != t) FAILF("seek: head %d track register %d", sim.top->head_track, sim.read(1));
    else if (expect_ok == !!(r.status & 0x10)) FAILF("seek verify: status %02X, model %s", r.status, expect_ok ? "has IDs" : "none");
    if (r.status & 0x10) res.seek_err++;
    // revolutions since the seek, against this track's own
    sim.ticks(32000 * 10);
    (void)idx0;
    for (s = 0; s < h.nsides; s++)
      if (!test_side(sim, h, t, s, name, res)) ok = false;
    s = 0;
    if (!ok) res.fail++;
    else if (!quiet) printf("ok   %s T%02d  blocks so far %ld\n", name, t, sim.blocks);
    fflush(stdout);
  }
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  int only_t = -1, argi = 1;
  for (; argi < argc && argv[argi][0] == '-'; argi++) {
    if (!strcmp(argv[argi], "-t") && argi + 1 < argc) only_t = atoi(argv[++argi]);
    else if (!strcmp(argv[argi], "-q")) quiet = true;
    else if (!strcmp(argv[argi], "-c") && argi + 1 < argc) swap_to = argv[++argi];
  }
  if (argi >= argc) {
    fprintf(stderr, "usage: %s [-t track] [-q] [-c disk2.hfe] image.hfe [other.hfe] [not_hfe.st]\n", argv[0]);
    return 1;
  }
  auto top = std::make_unique<Vfloppy_tb_top>();
  Sim sim(top.get());
  top->floppy_drive = 3;
  top->cpu_rw = 1;
  top->reset = 1;
  sim.ticks(8);
  top->reset = 0;

  const char *name = base(argv[argi]);
  Hfe h;
  if (!h.load(argv[argi]) || !load_file(argv[argi], sim.file[0])) {
    printf("%s: not an HFE file\n", argv[argi]);
    return 1;
  }
  bool ok = true;
  int t = 0, s = 0;
  if (!sim.mount(0)) FAILF("drive A did not become an HFE drive after mounting");
  long hdr_blocks = sim.blocks;

  Result res;
  int64_t c0 = sim.clocks;
  run_image(sim, h, name, only_t, res);
  double sim_s = (sim.clocks - c0) / 32e6;

  // index pulses: every revolution against the track the head was on
  {
    size_t hi = 0;
    int bad = 0;
    for (size_t i = 1; i < sim.index.size(); i++) {
      int64_t a = sim.index[i - 1], b = sim.index[i];
      while (hi + 1 < sim.head_at.size() && sim.head_at[hi + 1].first <= b) hi++;
      double ms = (b - a) / 32000.0;
      res.min_rev = std::min(res.min_rev, ms);
      res.max_rev = std::max(res.max_rev, ms);
      int ht = sim.head_at.empty() ? 0 : sim.head_at[hi].second;
      double nominal = ht < h.ntracks ? play(h, ht, 0).rev_time / 72000.0 : 200.0;
      if (nominal < 1) nominal = 200.0;
      if (std::fabs(ms / nominal - 1.0) > 0.05) {
        if (bad++ < 5) printf("FAIL %s index: revolution of %.2f ms at %.3f s (head %d, track %.2f ms)\n", name, ms,
                              b / 32e6, ht, nominal);
        fails_any = true;
      }
    }
    if (bad > 5) printf("     ... %d revolutions out of range in all\n", bad);
  }

  // side switch in the middle of a READ ADDRESS: the ID comes from side 1
  if (h.nsides == 2 && only_t < 0) {
    t = 2; s = 1;
    sim.top->side = 0;
    sim.write(3, 2);
    sim.command(0x18);
    SideModel m1 = model_side(h, 2, 1);
    if (!m1.fl.empty()) {
      Sim::Run r = sim.command(0xC8, 32000000LL * 3, [&](int64_t dt) { if (dt == 200) sim.top->side = 1; });
      bool found = false;
      for (auto &f : m1.fl)
        found |= r.data.size() == 6 && r.data[0] == f.trk && r.data[1] == f.side && r.data[2] == f.sec && r.data[3] == f.len;
      if (r.timeout || !found) FAILF("side switch during read address: status %02X", r.status);
      else if (!quiet) printf("ok   side switch during READ ADDRESS gives a side 1 ID\n");
    }
  }

  // drive B: another HFE, then back to A
  if (argi + 1 < argc && only_t < 0) {
    const char *nb = base(argv[argi + 1]);
    Hfe hb;
    std::vector<uint8_t> raw;
    if (hb.load(argv[argi + 1]) && load_file(argv[argi + 1], sim.file[1])) {
      t = 0; s = 0;
      if (!sim.mount(1)) FAILF("drive B (%s) did not become an HFE drive", nb);
      sim.top->floppy_drive = 1;   // drive B
      sim.top->side = 0;
      sim.command(0x08);
      SideModel mb = model_side(hb, 0, 0), ma = model_side(h, 0, 0);
      Sim::Run r;
      bool rb_ok = false, ra_ok = false;
      for (auto &f : mb.fl)
        if (f.id_crc_ok && f.dam >= 0) {
          sim.write(1, f.trk); sim.write(2, f.sec);
          r = sim.command(0x88);
          rb_ok = !r.timeout && match_sector(mb, f.trk, f.sec, r);
          break;
        }
      sim.top->floppy_drive = 2;   // back to A: its head is still where it was,
      sim.command(0x08);            // so restore, as software would
      for (auto &f : ma.fl)
        if (f.id_crc_ok && f.dam >= 0) {
          sim.write(1, f.trk); sim.write(2, f.sec);
          r = sim.command(0x88);
          ra_ok = !r.timeout && match_sector(ma, f.trk, f.sec, r);
          break;
        }
      if (!rb_ok || !ra_ok) FAILF("two drives: read from B %s, then from A %s", rb_ok ? "ok" : "FAILED", ra_ok ? "ok" : "FAILED");
      else if (!quiet) printf("ok   drive B (%s) read, then drive A again\n", nb);
    }
    // a non-HFE image does not become an HFE drive
    if (argi + 2 < argc && load_file(argv[argi + 2], sim.file[1])) {
      if (sim.mount(1, 32000 * 50)) FAILF("%s became an HFE drive", base(argv[argi + 2]));
      else if (!quiet) printf("ok   %s is not taken as HFE\n", base(argv[argi + 2]));
    }
  }

  // disk change in drive A (-c): the new image is read, with the head left
  // where it was (as a game waiting for "insert disk 2" leaves it), both
  // when the change comes while idle and in the middle of a READ ADDRESS
  if (swap_to) {
    Hfe h2;
    std::vector<uint8_t> f2;
    if (!h2.load(swap_to) || !load_file(swap_to, f2)) {
      printf("%s: not an HFE file\n", swap_to);
      return 1;
    }
    const char *n2 = base(swap_to);
    std::vector<uint8_t> f1 = sim.file[0];
    for (int pass = 0; pass < 4; pass++) {
      bool mid = pass & 1;                       // change during a READ ADDRESS
      bool to2 = !(pass & 2);                    // 1 -> 2, then back 2 -> 1
      const Hfe &hn = to2 ? h2 : h;
      const char *nn = to2 ? n2 : name;
      int trk = 1 + pass;                        // a track with data on both
      t = trk; s = 0;
      sim.top->floppy_drive = 2;
      sim.top->side = 0;
      sim.write(3, trk);
      sim.command(0x18);                         // seek, the head stays here
      sim.ticks(32000 * 50);
      sim.file[0] = to2 ? f2 : f1;
      if (mid) {
        sim.command(0xC0, 32000000LL * 3, [&](int64_t dt) {
          if (dt == 32000 * 3) { sim.top->img_size = (uint32_t)sim.file[0].size(); sim.top->img_mounted = 1; }
          else if (dt == 32000 * 3 + 1) sim.top->img_mounted = 0;
        });
        int64_t t0 = sim.clocks;
        while (!(sim.top->hfe_valid & 1) && sim.clocks - t0 < 32000 * 200) sim.tick();
        if (!(sim.top->hfe_valid & 1)) FAILF("disk change to %s (during READ ADDRESS): drive not valid again", nn);
      } else if (!sim.mount(0)) FAILF("disk change to %s: drive not valid again", nn);
      // the new disk's IDs and data on the track the head is on
      SideModel m = model_side(hn, trk, 0);
      Sim::Run ra = sim.command(0xC0);
      bool id_ok = false;
      for (auto &f : m.fl)
        id_ok |= ra.data.size() == 6 && ra.data[0] == f.trk && ra.data[2] == f.sec && ra.data[3] == f.len;
      bool rd_ok = m.fl.empty();
      for (auto &f : m.fl)
        if (f.id_crc_ok && f.dam >= 0) {
          sim.write(1, f.trk); sim.write(2, f.sec);
          Sim::Run r = sim.command(0x80);
          rd_ok = !r.timeout && match_sector(m, f.trk, f.sec, r);
          break;
        }
      if (!m.fl.empty() && (!id_ok || !rd_ok))
        FAILF("disk change to %s%s: track %d reads %s", nn, mid ? " during READ ADDRESS" : "", trk,
              !id_ok ? "a wrong ID" : "wrong data");
      else if (!quiet) printf("ok   disk change to %s%s: track %d reads the new disk\n", nn,
                              mid ? " during READ ADDRESS" : "", trk);
    }
  }

  printf("%-44s %3ld tracks, %3ld failed | %ld sides, %ld sectors, %ld RNF skipped, %ld seek errors expected | "
         "header+table %ld blocks, %ld blocks in all | revolutions %.2f-%.2f ms | %.1f s simulated\n",
         name, res.tracks, res.fail, res.sides, res.sectors, res.rnf_skipped, res.seek_err, hdr_blocks, sim.blocks,
         res.min_rev, res.max_rev, sim_s);
  return fails_any ? 1 : 0;
}
