// tb_fdc.cpp - Verilator test of the WD1772 MFM controller (wd1772_mfm.v)
// against tools/hfev3/hfe_model.cpp
//
//   ./tb_fdc [-t track] [-s side] [-q] image.hfe ...   every side
//   ./tb_fdc -1 image.hfe                              type I / misc tests
//
// The testbench is the CPU (register accesses, DRQ served ~0.5 us after
// it rises) and the drive mechanics (head moves on step pulses, the track
// is reloaded). For every side it checks, against the model:
//
//   READ ADDRESS  every ID of the revolution (track, side, sector, length,
//                 CRC status) comes back, and nothing else
//   READ SECTOR   for every track/sector pair in the IDs: the data (FNV
//                 hash), CRC error, record type (deleted) and lost data
//                 bits, or record not found when no such ID has data;
//                 and the data rate, byte to byte, within 1% of the model
//                 (the slow sector 6 protection)
//   READ TRACK    every byte and the 8 MHz cycle it arrives in, from index
//                 to index, as read_track()

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <memory>
#include "ref.h"
#include "verilated.h"
#include "Vfdc_tb_top.h"

static bool quiet = false;

static const char *base(const char *p) {
  const char *s = strrchr(p, '/');
  return s ? s + 1 : p;
}

static unsigned fnv(const std::vector<uint8_t> &d) {
  unsigned h = 2166136261u;
  for (uint8_t v : d) { h ^= v; h *= 16777619u; }
  return h;
}

static int nominal_x(const Hfe &h) {
  int nominal = 72;
  if (h.bitrate >= 50 && h.bitrate <= 2000) nominal = (18000 + h.bitrate / 2) / h.bitrate;
  return nominal;
}
static uint32_t seed_of(int t, int s) { return 0x1234567u + t * 7 + s; }

// ------------------------------------------------------------------ sim

struct Sim {
  Vfdc_tb_top *top;
  const Hfe *h = nullptr;
  std::vector<uint8_t> st, rawt;
  uint32_t addr_q = 0;
  int head = 0, side = 0;
  bool loaded = false;      // a track is playing
  int64_t n = -1;           // 32 MHz clocks since the player's time 0
  int64_t clocks = 0;       // all clocks
  std::vector<int64_t> index;  // 8 MHz cycle of each index pulse since time 0
  bool reload = false;

  explicit Sim(Vfdc_tb_top *t) : top(t) {}

  void load(int t, int s) {
    head = t; side = s;
    index.clear();
    n = -1;
    top->drive_reset = 1;
    loaded = false;
    if (h && t < h->ntracks && s < h->nsides) {
      st = h->side_stream(t, s);
      Track tr = play(*h, t, s);
      if (!st.empty() && !tr.cells.empty()) {
        top->v3 = h->v3;
        top->nominal_x = nominal_x(*h);
        top->stream_len = (uint16_t)st.size();
        top->idx_at_start = tr.op_index == 0;
        top->seed = seed_of(t, s);
        top->side = s;
        // the RTL reads the raw track (both sides, as in the file)
        rawt.assign(h->d.begin() + std::min(h->d.size(), (size_t)h->trk_off(t)),
                    h->d.begin() + std::min(h->d.size(), (size_t)h->trk_off(t) + (h->trk_len(t) + 511) / 512 * 512));  // whole blocks
        loaded = true;
      }
    }
    top->dr_track0 = head == 0;
    for (int i = 0; i < 4; i++) tick();
    if (loaded) top->drive_reset = 0;
  }

  void tick() {
    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
    clocks++;
    top->ram_data = addr_q < rawt.size() ? rawt[addr_q] : 0;
    addr_q = top->ram_addr;
    if (top->start) n = 0;
    else if (n >= 0) n++;
    if (top->index && n >= 0) index.push_back(n >> 2);
    if (top->step_pulse) {
      head += top->step_out ? -1 : 1;
      if (head < 0) head = 0;
      if (head > 85) head = 85;
      reload = true;
    }
    if (reload && !top->step_pulse) {  // the drive needs the new track
      reload = false;
      load(head, side);
    }
  }
  void ticks(int k) { for (int i = 0; i < k; i++) tick(); }

  // ---- CPU bus (an access lasts ~4 clocks, as a 68000 bus cycle)
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

  // Run a command until IRQ; DRQ bytes are collected with their 8 MHz cycle.
  struct Run {
    bool timeout = false;
    int status = 0;
    std::vector<uint8_t> data;
    std::vector<int64_t> when;   // 8 MHz cycle of the byte (from the DRQ edge)
    int64_t clocks = 0;
  };
  Run command(int cmd, int64_t max_clocks = 32000000LL * 3) {
    Run r;
    write(0, cmd);
    int64_t t0 = clocks;
    bool drq_prev = false;
    int64_t serve_at = -1;
    while (!top->irq) {
      if (clocks - t0 > max_clocks) { r.timeout = true; break; }
      tick();
      if (top->drq && !drq_prev) {
        // drq rose: the byte reached the controller the clock before
        r.when.push_back(n >= 0 ? (n - 6) >> 2 : -1);
        serve_at = clocks + 16;
      }
      drq_prev = top->drq;
      if (serve_at >= 0 && clocks >= serve_at) {
        serve_at = -1;
        r.data.push_back((uint8_t)read(3));
        drq_prev = top->drq;
      }
    }
    // a byte whose DRQ is still up at the IRQ (READ ADDRESS ends with its
    // sixth byte) is still taken, as the DMA would
    if (!r.timeout && top->drq) {
      if (!drq_prev) r.when.push_back(n >= 0 ? (n - 6) >> 2 : -1);
      ticks(serve_at >= 0 ? (int)std::max<int64_t>(0, serve_at - clocks) : 16);
      r.data.push_back((uint8_t)read(3));
    }
    r.clocks = clocks - t0;
    r.status = read(0);
    return r;
  }
};

// ------------------------------------------------------------------ results

struct Result {
  long sides = 0, fail = 0, skipped = 0;
  long ra = 0, sectors = 0, rnf = 0, rnf_skipped = 0, slow = 0, rt_bytes = 0;
  void add(const Result &o) {
    sides += o.sides; fail += o.fail; skipped += o.skipped; ra += o.ra; sectors += o.sectors;
    rnf += o.rnf; rnf_skipped += o.rnf_skipped; slow += o.slow; rt_bytes += o.rt_bytes;
  }
};

#define FAILF(...) do { if (ok) printf("FAIL %s T%02d.%d:", name, t, s); printf(" " __VA_ARGS__); ok = false; } while (0)

static bool test_side(Sim &sim, const Hfe &h, int t, int s, const char *name, Result &res) {
  Track tr = play(h, t, s);
  if (h.side_stream(t, s).empty() || tr.cells.empty()) {
    res.skipped++;
    return true;
  }
  bool ok = true;
  uint32_t seed = seed_of(t, s);

  // ---- model: the fields of the second revolution, and a second decode
  // with other weak bits to tell which sectors are weak
  RecBits rb = recover(tr, 3, seed);
  std::vector<Mark> mk = find_marks(rb);
  int64_t ia = rb.index.size() > 1 ? rb.index[1] : 0, ib = rb.index.size() > 2 ? rb.index[2] : INT64_MAX;
  std::vector<Field> fl = decode_fields(rb, mk, ia, ib);
  RecBits rb2 = recover(tr, 3, seed ^ 0x5a5a5a5au);
  std::vector<Mark> mk2 = find_marks(rb2);
  std::vector<Field> fl2 = decode_fields(rb2, mk2, rb2.index.size() > 1 ? rb2.index[1] : 0,
                                         rb2.index.size() > 2 ? rb2.index[2] : INT64_MAX);
  std::vector<bool> weak(fl.size(), false);
  for (size_t i = 0; i < fl.size(); i++)
    for (auto &g : fl2)
      if (g.sec == fl[i].sec && g.trk == fl[i].trk && std::llabs(g.t_us - fl[i].t_us) < 2000 && g.dam >= 0 &&
          fl[i].dam >= 0) {
        if (g.data != fl[i].data || g.data_crc_ok != fl[i].data_crc_ok) weak[i] = true;
        break;
      }

  // ---- the RTL: fresh disk, head on this track, motor running
  sim.load(t, s);
  sim.write(0, 0xD0);   // force interrupt: idle, motor on
  sim.ticks(64);

  // READ ADDRESS: every ID of a revolution
  std::set<std::string> m_ids, r_ids;
  for (auto &f : fl) {
    char b[64];
    snprintf(b, sizeof b, "%d.%d.%d.%d %s", f.trk, f.side, f.sec, f.len, f.id_crc_ok ? "ok" : "BAD");
    m_ids.insert(b);
  }
  int n_ra = m_ids.empty() ? 1 : (int)fl.size() + 1;
  for (int i = 0; i < n_ra; i++) {
    Sim::Run r = sim.command(0xC8);
    res.ra++;
    if (r.timeout) { FAILF("read address timeout"); break; }
    if (m_ids.empty()) {
      if (!(r.status & 0x10)) FAILF("read address on a blank side: status %02X, no RNF", r.status);
      break;
    }
    if (r.status & 0x14 || r.data.size() != 6) {
      FAILF("read address status %02X, %zu bytes", r.status, r.data.size());
      break;
    }
    char b[64];
    snprintf(b, sizeof b, "%d.%d.%d.%d %s", r.data[0], r.data[1], r.data[2], r.data[3],
             (r.status & 0x08) ? "BAD" : "ok");
    r_ids.insert(b);
    if (!m_ids.count(b)) FAILF("read address gave %s, not in the model", b);
    if (sim.read(2) != r.data[0]) FAILF("read address: sector register %d, not the ID track %d", sim.read(2), r.data[0]);
  }
  if (!m_ids.empty() && r_ids != m_ids) FAILF("read address saw %zu of %zu IDs", r_ids.size(), m_ids.size());

  // READ SECTOR for every track/sector pair, in rotation order
  std::vector<std::pair<int64_t, int>> pairs;  // t_us, trk << 8 | sec
  {
    std::set<int> seen;
    for (auto &f : fl)
      if (f.id_crc_ok && !seen.count(f.trk << 8 | f.sec)) {
        seen.insert(f.trk << 8 | f.sec);
        pairs.push_back({f.t_us, f.trk << 8 | f.sec});
      }
    std::sort(pairs.begin(), pairs.end());
  }
  bool rnf_done = false;
  for (auto &p : pairs) {
    int trk = p.second >> 8, sec = p.second & 0xff;
    std::vector<size_t> cand;
    for (size_t i = 0; i < fl.size(); i++)
      if (fl[i].id_crc_ok && fl[i].trk == trk && fl[i].sec == sec && fl[i].dam >= 0) cand.push_back(i);
    if (cand.empty() && rnf_done) {  // each RNF costs 5 revolutions: one per side is enough
      res.rnf_skipped++;
      continue;
    }
    sim.write(1, trk);
    sim.write(2, sec);
    Sim::Run r = sim.command(0x88);
    if (r.timeout) { FAILF("read sector %d.%d timeout", trk, sec); break; }
    if (cand.empty()) {
      rnf_done = true;
      res.rnf++;
      if (!(r.status & 0x10)) FAILF("sector %d.%d has no data in the model, status %02X", trk, sec, r.status);
      continue;
    }
    res.sectors++;
    if (r.status & 0x14) { FAILF("sector %d.%d status %02X", trk, sec, r.status); continue; }
    unsigned hr = fnv(r.data);
    bool crc_bad = r.status & 0x08, deleted = r.status & 0x20;
    // which model field did it read?
    long match = -1;
    for (size_t i : cand) {
      const Field &f = fl[i];
      bool data_ok = (weak[i] && !f.data_crc_ok) ? crc_bad : fnv(f.data) == hr && crc_bad == !f.data_crc_ok;
      if (data_ok && r.data.size() == f.data.size() && deleted == (f.dam <= 0xF9)) { match = (long)i; break; }
    }
    if (match < 0) {
      const Field &f = fl[cand[0]];
      FAILF("sector %d.%d: rtl %zu bytes %08x crc %s %s, model %zu bytes %08x crc %s dam %02X%s", trk, sec,
            r.data.size(), hr, crc_bad ? "BAD" : "ok", deleted ? "deleted" : "", f.data.size(), fnv(f.data),
            f.data_crc_ok ? "ok" : "BAD", f.dam, weak[cand[0]] ? " weak" : "");
      continue;
    }
    // data rate: microseconds per byte, rtl (first to last data byte) vs model (DAM to CRC)
    const Field &f = fl[match];
    if (r.when.size() == f.data.size() && f.data_us && f.data.size() > 1) {
      double us_r = (r.when.back() - r.when.front()) / 8.0 / (double)(f.data.size() - 1);
      double us_m = f.data_us / (double)(f.data.size() + 2);
      if (std::fabs(us_r / us_m - 1.0) > 0.01) FAILF("sector %d.%d: %.2f us/byte, model %.2f", trk, sec, us_r, us_m);
      if (us_r > 33.0) res.slow++;
    } else
      FAILF("sector %d.%d: %zu DRQs for %zu bytes", trk, sec, r.when.size(), f.data.size());
  }

  // READ TRACK, compared with read_track() for the revolution it read
  {
    size_t before = sim.index.size();
    Sim::Run r = sim.command(0xE8, 32000000LL * 2);
    if (r.timeout) FAILF("read track timeout");
    else if (r.status & 0x14) FAILF("read track status %02X", r.status);
    else {
      size_t k = before;  // the index the transfer started at
      while (k < sim.index.size() && !r.when.empty() && sim.index[k] > r.when.front()) k--;
      while (k + 1 < sim.index.size() && !r.when.empty() && sim.index[k + 1] <= r.when.front()) k++;
      RecBits rr = recover(tr, (int)k + 3, seed);
      if (k + 1 >= rr.index.size() || rr.index[k] != sim.index[k]) FAILF("read track: index %zu differs", k);
      else {
        std::vector<RtByte> m = read_track(rr, rr.index[k], rr.index[k + 1]);
        size_t nb = std::min(m.size(), r.data.size()), d = nb;
        for (size_t i = 0; i < nb; i++)
          if ((m[i].v & 0xff) != r.data[i] || m[i].time != r.when[i]) { d = i; break; }
        if (d < nb || m.size() != r.data.size()) {
          if (d < nb)
            FAILF("read track byte #%zu rtl %02X@%lld model %02X@%lld (%zu/%zu)", d, r.data[d], (long long)r.when[d],
                  m[d].v & 0xff, (long long)m[d].time, r.data.size(), m.size());
          else
            FAILF("read track %zu bytes, model %zu", r.data.size(), m.size());
        }
        res.rt_bytes += (long)r.data.size();
      }
    }
  }

  res.sides++;
  if (!ok) {
    res.fail++;
    printf("\n");
  } else if (!quiet)
    printf("ok   %s T%02d.%d  %zu IDs, %zu sectors\n", name, t, s, m_ids.size(), pairs.size());
  fflush(stdout);
  return ok;
}

// ------------------------------------------------------------------ type I

static int type1_tests(Sim &sim, const Hfe &h, const char *name) {
  int fails = 0;
  // ms values are printed in decimal in the status column
  auto check = [&](bool c, const char *what, int status) {
    printf("%s %-60s %s %02X head %d track %d\n", c ? "ok  " : "FAIL", what, "status", status, sim.head, sim.read(1));
    if (!c) fails++;
  };
  printf("==== type I and misc tests on %s\n", name);
  sim.load(10, 0);
  sim.write(1, 10);

  // RESTORE with verify, motor off: spin up (6 index pulses) first
  int64_t c0 = sim.clocks;
  Sim::Run r = sim.command(0x04);
  double ms = (sim.clocks - c0) / 32000.0;
  check(!r.timeout && sim.head == 0 && sim.read(1) == 0 && !(r.status & 0x18) && (r.status & 0x04) && (r.status & 0x20),
        "restore + verify from track 10, with spin up", r.status);
  printf("     spin up + restore took %.0f ms\n", ms);
  check(ms > 5 * 190.0, "  spin up waited for the index pulses", 0);

  sim.write(3, 40);   // seek destination
  r = sim.command(0x1C);
  check(!r.timeout && sim.head == 40 && sim.read(1) == 40 && !(r.status & 0x18), "seek 40 + verify", r.status);

  r = sim.command(0x58);  // step in, update
  check(!r.timeout && sim.head == 41 && sim.read(1) == 41, "step in + update", r.status);
  r = sim.command(0x7C);  // step out, update, verify
  check(!r.timeout && sim.head == 40 && sim.read(1) == 40 && !(r.status & 0x10), "step out + update + verify", r.status);
  r = sim.command(0x48);  // step in, no update
  check(!r.timeout && sim.head == 41 && sim.read(1) == 40, "step in, track register kept", r.status);
  r = sim.command(0x3C);  // step (same direction: in), update, verify
  check(!r.timeout && sim.head == 42 && sim.read(1) == 41 && (r.status & 0x10), "step + verify with a wrong track register: seek error",
        r.status);

  sim.write(1, 42);
  sim.write(3, 2);
  r = sim.command(0x1B);  // seek to 2, h, no verify, 3 ms steps
  check(!r.timeout && sim.head == 2 && sim.read(1) == 2, "seek 2, no verify", r.status);

  // type II on this track: a write is refused, read works
  sim.write(2, 1);
  r = sim.command(0xA8);
  check(!r.timeout && (r.status & 0x40) && !(r.status & 0x01), "write sector: write protected", r.status);
  r = sim.command(0x88);
  check(!r.timeout && r.data.size() == 512 && !(r.status & 0x1C), "read sector 2.1", r.status);
  // multi-sector read of sectors 1.. until RNF
  sim.write(2, 1);
  r = sim.command(0x98, 32000000LL * 4);
  check(!r.timeout && (r.status & 0x10) && r.data.size() % 512 == 0 && r.data.size() >= 512 * 9 && sim.read(2) > 9,
        "multi-sector read ends with RNF after the last sector", r.status);
  printf("     (%zu bytes, sector register %d)\n", r.data.size(), sim.read(2));

  // force interrupt while busy: D0 stops a read, no IRQ
  sim.write(2, 99);  // a sector that is not there: the read would run for 5 revolutions
  sim.write(0, 0x88);
  sim.ticks(32000 * 50);
  sim.write(0, 0xD0);
  sim.ticks(64);
  int st = sim.read(0);
  check(!(st & 0x01) && !sim.top->irq, "force interrupt D0 stops a read without IRQ", st);
  sim.write(0, 0xD8);
  sim.ticks(16);
  bool irq_now = sim.top->irq;
  check(irq_now, "force interrupt D8 interrupts at once", sim.read(0));
  sim.write(0, 0xD4);
  sim.ticks(16);
  st = sim.read(0);  // clears the irq
  int64_t t0 = sim.clocks;
  while (!sim.top->irq && sim.clocks - t0 < 32000000LL / 2) sim.tick();
  bool first = sim.top->irq;
  sim.read(0);
  t0 = sim.clocks;
  while (!sim.top->irq && sim.clocks - t0 < 32000000LL / 2) sim.tick();
  check(first && sim.top->irq, "force interrupt D4 interrupts at every index pulse", st);
  sim.write(0, 0xD0);
  sim.read(0);

  // motor off after 10 idle index pulses
  t0 = sim.clocks;
  while (sim.top->motor_on && sim.clocks - t0 < 32000000LL * 3) sim.tick();
  ms = (sim.clocks - t0) / 32000.0;
  printf("     motor went off after %.0f ms idle\n", ms);
  check(!sim.top->motor_on && ms > 9 * 190.0 && ms < 11 * 210.0, "motor off after 10 idle revolutions", 0);

  // no disk: type II ends with RNF
  sim.top->dr_present = 0;
  r = sim.command(0x88);
  check(!r.timeout && (r.status & 0x10), "no disk: read sector gives RNF", r.status);
  sim.top->dr_present = 1;
  return fails;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  int only_t = -1, only_s = -1, argi = 1;
  bool t1 = false;
  for (; argi < argc && argv[argi][0] == '-'; argi++) {
    if (!strcmp(argv[argi], "-t") && argi + 1 < argc) only_t = atoi(argv[++argi]);
    else if (!strcmp(argv[argi], "-s") && argi + 1 < argc) only_s = atoi(argv[++argi]);
    else if (!strcmp(argv[argi], "-q")) quiet = true;
    else if (!strcmp(argv[argi], "-1")) t1 = true;
  }
  auto top = std::make_unique<Vfdc_tb_top>();
  Sim sim(top.get());
  top->dr_present = 1;
  top->dr_wp = 1;
  top->cpu_rw = 1;
  top->fdc_reset_n = 0;
  sim.ticks(8);
  top->fdc_reset_n = 1;

  Result total;
  int fails = 0;
  for (; argi < argc; argi++) {
    Hfe h;
    if (!h.load(argv[argi])) {
      printf("%s: not an HFE file\n", argv[argi]);
      fails++;
      continue;
    }
    sim.h = &h;
    if (t1) {
      fails += type1_tests(sim, h, base(argv[argi]));
      continue;
    }
    Result r;
    for (int t = 0; t < h.ntracks; t++)
      for (int s = 0; s < h.nsides; s++) {
        if ((only_t >= 0 && t != only_t) || (only_s >= 0 && s != only_s)) continue;
        test_side(sim, h, t, s, base(argv[argi]), r);
      }
    printf("%-44s %4ld sides, %3ld failed, %ld skipped | %ld read address, %ld sectors (%ld slow), %ld RNF (%ld more "
           "skipped), %ld read track bytes\n",
           base(argv[argi]), r.sides, r.fail, r.skipped, r.ra, r.sectors, r.slow, r.rnf, r.rnf_skipped, r.rt_bytes);
    fflush(stdout);
    total.add(r);
  }
  if (!t1) printf("TOTAL %ld sides, %ld failed\n", total.sides, total.fail);
  return (total.fail || fails) ? 1 : 0;
}
