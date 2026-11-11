// tb_hfe.cpp - Verilator unit tests for the HFE floppy blocks, against
// tools/hfev3/hfe_model.cpp
//
//   ./tb_hfe [-t track] [-s side] [-q] image.hfe ...
//
// For every side of every image the RTL plays three revolutions, the same
// span the model's -s mode decodes, and each block's output is compared
// with the model:
//   player : the 8 MHz cycle of every flux transition and index pulse,
//            weak bits included (same seed as hfe_model -s)
//   dpll   : every recovered cell (value and 8 MHz cycle), and the range
//            of the DPLL increment
//   amdet  : READ TRACK of the second revolution (every byte, its marks and
//            its 8 MHz cycle, as read_track()), and the ID/data fields read
//            with sync_en/CRC driven like the FDC will, as hfe_model -s

#include <cstdio>
#include <cstdlib>
#include <memory>
#include "ref.h"
#include "verilated.h"
#include "Vhfe_tb_top.h"

struct Result {
  long sides = 0, fail = 0, skipped = 0;
  long flux = 0, index = 0, cells = 0, rtbytes = 0, fields = 0;
  int min_inc = 128, max_inc = 128;
};

static bool quiet = false;
static int debug_n = 0;

static const char *base(const char *p) {
  const char *s = strrchr(p, '/');
  return s ? s + 1 : p;
}

// first difference of two sequences, -1 if equal
template <class T>
static long first_diff(const std::vector<T> &a, const std::vector<T> &b) {
  size_t n = std::min(a.size(), b.size());
  for (size_t i = 0; i < n; i++)
    if (a[i] != b[i]) return (long)i;
  return a.size() == b.size() ? -1 : (long)n;
}

static bool test_side(Vhfe_tb_top *top, const Hfe &h, int t, int s, const char *name, Result &res) {
  Track tr = play(h, t, s);
  std::vector<uint8_t> st = h.side_stream(t, s);
  if (st.empty() || tr.cells.empty()) {
    res.skipped++;
    return true;
  }
  int nominal = 72;
  if (h.bitrate >= 50 && h.bitrate <= 2000) nominal = (18000 + h.bitrate / 2) / h.bitrate;
  uint32_t seed = 0x1234567u + t * 7 + s;
  int64_t limit = tr.rev_time * 3 / 9;  // three revolutions, 8 MHz cycles

  // ---- model
  std::vector<int64_t> m_flux, m_index;
  {
    FluxSource src(tr, seed);
    for (;;) {
      int64_t tt = src.next_transition();
      if (tt > limit) break;
      if (src.has_flux) m_flux.push_back(tt);
    }
    for (int64_t i : src.index_times)
      if (i / 9 <= limit) m_index.push_back(i / 9);
  }
  // cells through the DPLL, encoded as cycle * 2 + value
  std::vector<int64_t> m_cells;
  int m_min_inc, m_max_inc;
  {
    RecBits rb = recover(tr, 3, seed);
    for (size_t i = 0; i < rb.bit.size(); i++)
      if (rb.time[i] < limit) m_cells.push_back(rb.time[i] * 2 + rb.bit[i]);
    m_min_inc = rb.min_inc;
    m_max_inc = rb.max_inc;
  }
  // READ TRACK of the second revolution: byte | mark << 8, and its cycle
  std::vector<int64_t> m_rt;
  {
    RecBits rb = recover(tr, 3, seed);
    int64_t a = rb.index.size() > 1 ? rb.index[1] : 0, b = rb.index.size() > 2 ? rb.index[2] : INT64_MAX;
    for (const RtByte &x : read_track(rb, a, b)) m_rt.push_back(x.time << 12 | x.v);
  }
  std::vector<std::string> m_fields = sector_lines(h, t, s, seed);

  // ---- RTL
  top->v3 = h.v3;
  top->nominal_x = nominal;
  top->stream_len = (uint16_t)st.size();
  top->idx_at_start = tr.op_index == 0;
  top->seed = seed;
  top->side = s;
  // the RTL reads the raw track (both sides, as in the file)
  std::vector<uint8_t> rawt(h.d.begin() + std::min(h.d.size(), (size_t)h.trk_off(t)),
                            h.d.begin() + std::min(h.d.size(), (size_t)h.trk_off(t) + (h.trk_len(t) + 511) / 512 * 512));  // whole blocks
  top->reset = 1;
  for (int i = 0; i < 4; i++) {
    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
  }
  top->reset = 0;

  std::vector<int64_t> r_flux, r_index, r_cells;
  int r_min_inc = 128, r_max_inc = 128;
  std::vector<std::pair<int64_t, int>> r_rt;  // cycle, byte | mark << 8

  // field reader standing in for the FDC: looks for A1 A1 A1 FE with sync
  // on, reads the 6 ID bytes with sync off, then looks for A1 A1 A1 F8-FB
  // whose first A1 ends within 43+3 bytes of the ID, and reads the data.
  enum { SEARCH_ID, READ_ID, SEARCH_DAM, READ_DATA } fst = SEARCH_ID;
  struct Fld {
    int64_t fe_time;
    uint8_t id[4];
    bool id_ok;
    int dam = -1;
    bool data_ok = false;
    std::vector<uint8_t> data;
  };
  std::vector<Fld> r_fld;
  int fcount = 0;
  int64_t cellidx = 0, dam_deadline = 0;
  uint8_t idb[6];
  top->f_sync = 1;

  uint32_t addr_q = 0;   // address the RAM samples on the next edge
  int64_t n = -1;        // 32 MHz clock since time 0
  bool under = false;
  for (long guard = 0; guard < 200000000L; guard++) {
    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
    // synchronous RAM: data out after this edge is what it sampled at it
    top->ram_data = addr_q < rawt.size() ? rawt[addr_q] : 0;
    addr_q = top->ram_addr;

    if (top->start) n = 0;
    else if (n >= 0) n++;
    if (n < 0) continue;
    int64_t cyc = n >> 2;
    if (cyc > limit) break;
    if (top->flux) r_flux.push_back(cyc);
    if (top->index) r_index.push_back(cyc);
    if (top->underrun) under = true;
    // DPLL outputs are registered: this edge reports the ce8 step of clock n-1
    if (top->bit_valid) {
      int64_t bc = (n - 1) >> 2;
      if (bc < limit) r_cells.push_back(bc * 2 + top->bit_val);
      if (top->increment < r_min_inc) r_min_inc = top->increment;
      if (top->increment > r_max_inc) r_max_inc = top->increment;
      cellidx++;
    }
    // mark detector outputs: the byte whose last cell was reported last clock
    if (top->rt_valid) r_rt.push_back({(n - 5) >> 2, top->rt_byte | (top->rt_a1 ? 1 : top->rt_c2 ? 2 : 0) << 8});
    if (top->f_valid) {
      int64_t bt = (n - 5) >> 2, pos = cellidx - 1;
      int v = top->f_byte;
      bool cand = top->f_a1_run == 3 && !top->f_a1 && !top->f_c2;  // byte after three A1 marks
      bool used = false;  // the byte was a mark byte that started a field
      if (fst == SEARCH_DAM) {
        if (pos > dam_deadline + 48) fst = SEARCH_ID;  // no triple can start in time any more
        else if (cand && pos - 48 <= dam_deadline) {
          if (v >= 0xF8 && v <= 0xFB) {
            r_fld.back().dam = v;
            used = true;
            fst = READ_DATA;
            fcount = 0;
            top->f_sync = 0;
          } else if (v == 0xFE)
            fst = SEARCH_ID;  // the next ID: this one has no data
        }
      }
      if (used) {
      } else if (fst == SEARCH_ID) {
        if (cand && v == 0xFE) {
          Fld f;
          f.fe_time = bt;
          r_fld.push_back(f);
          fst = READ_ID;
          fcount = 0;
          top->f_sync = 0;
        }
      } else if (fst == READ_ID) {
        idb[fcount++] = (uint8_t)v;
        if (fcount == 6) {
          Fld &f = r_fld.back();
          memcpy(f.id, idb, 4);
          f.id_ok = top->f_crc == 0;
          fst = SEARCH_DAM;
          dam_deadline = pos + 16 * 43;  // first A1 must end by 43+3 bytes after the ID CRC
          top->f_sync = 1;
        }
      } else if (fst == READ_DATA) {
        Fld &f = r_fld.back();
        int size = 128 << (f.id[3] & 3);
        if (fcount < size) f.data.push_back((uint8_t)v);
        if (++fcount == size + 2) {
          f.data_ok = top->f_crc == 0;
          fst = SEARCH_ID;
          top->f_sync = 1;
        }
      }
    }
  }

  // READ TRACK window and field lines, as the model reports them
  std::vector<int64_t> r_rtw;
  int64_t wa = r_index.size() > 1 ? r_index[1] : 0, wb = r_index.size() > 2 ? r_index[2] : INT64_MAX;
  for (auto &x : r_rt)
    if (x.first >= wa && x.first < wb) r_rtw.push_back(x.first << 12 | x.second);
  std::vector<std::string> r_fields;
  for (auto &f : r_fld) {
    if (f.fe_time < wa || f.fe_time >= wb) continue;
    unsigned hsh = 2166136261u;
    for (uint8_t v : f.data) { hsh ^= v; hsh *= 16777619u; }
    bool dok = f.dam >= 0 && f.data.size() == (size_t)(128 << (f.id[3] & 3)) && f.data_ok;
    char ln[160];
    snprintf(ln, sizeof ln, "T%02d.%d id %d.%d.%d.%d %s dam %02X %s %08x", t, s, f.id[0], f.id[1], f.id[2], f.id[3],
             f.id_ok ? "ok" : "BAD", f.dam & 0xff, f.dam < 0 ? "-" : (dok ? "ok" : "BAD"), f.dam < 0 ? 0 : hsh);
    r_fields.push_back(ln);
  }
  std::sort(r_fields.begin(), r_fields.end());
  r_fields.erase(std::unique(r_fields.begin(), r_fields.end()), r_fields.end());

  res.sides++;
  res.flux += (long)r_flux.size();
  res.index += (long)r_index.size();
  for (int i = 0; i < debug_n; i++)
    printf("  %3d  flux r %6lld m %6lld | cell r %6lld:%lld m %6lld:%lld\n", i,
           i < (int)r_flux.size() ? (long long)r_flux[i] : -1LL, i < (int)m_flux.size() ? (long long)m_flux[i] : -1LL,
           i < (int)r_cells.size() ? (long long)(r_cells[i] >> 1) : -1LL, i < (int)r_cells.size() ? (long long)(r_cells[i] & 1) : -1LL,
           i < (int)m_cells.size() ? (long long)(m_cells[i] >> 1) : -1LL, i < (int)m_cells.size() ? (long long)(m_cells[i] & 1) : -1LL);
  res.cells += (long)r_cells.size();
  res.min_inc = std::min(res.min_inc, r_min_inc);
  res.max_inc = std::max(res.max_inc, r_max_inc);
  long df = first_diff(r_flux, m_flux), di = first_diff(r_index, m_index);
  long dc = first_diff(r_cells, m_cells);
  bool inc_ok = r_min_inc == m_min_inc && r_max_inc == m_max_inc;
  long dr = first_diff(r_rtw, m_rt), dfl = first_diff(r_fields, m_fields);
  res.rtbytes += (long)r_rtw.size();
  res.fields += (long)r_fields.size();
  bool ok = df < 0 && di < 0 && dc < 0 && inc_ok && !under && dr < 0 && dfl < 0;
  if (!ok) {
    res.fail++;
    printf("FAIL %s T%02d.%d:", name, t, s);
    if (under) printf(" underrun");
    if (df >= 0)
      printf(" flux #%ld rtl %lld model %lld (%zu/%zu)", df,
             df < (long)r_flux.size() ? (long long)r_flux[df] : -1LL,
             df < (long)m_flux.size() ? (long long)m_flux[df] : -1LL, r_flux.size(), m_flux.size());
    if (di >= 0)
      printf(" index #%ld rtl %lld model %lld (%zu/%zu)", di,
             di < (long)r_index.size() ? (long long)r_index[di] : -1LL,
             di < (long)m_index.size() ? (long long)m_index[di] : -1LL, r_index.size(), m_index.size());
    if (dc >= 0) {
      auto show = [](const std::vector<int64_t> &v, long i) {
        if (i >= (long)v.size()) return std::string("end");
        char b[48];
        snprintf(b, sizeof b, "%lld:%d", (long long)(v[i] >> 1), (int)(v[i] & 1));
        return std::string(b);
      };
      printf(" cell #%ld rtl %s model %s (%zu/%zu)", dc, show(r_cells, dc).c_str(), show(m_cells, dc).c_str(),
             r_cells.size(), m_cells.size());
    }
    if (!inc_ok) printf(" increment rtl %d-%d model %d-%d", r_min_inc, r_max_inc, m_min_inc, m_max_inc);
    if (dr >= 0) {
      auto show = [](const std::vector<int64_t> &v, long i) {
        if (i >= (long)v.size()) return std::string("end");
        char b[48];
        snprintf(b, sizeof b, "%lld:%02X/%d", (long long)(v[i] >> 12), (int)(v[i] & 0xff), (int)(v[i] >> 8 & 3));
        return std::string(b);
      };
      printf(" readtrack #%ld rtl %s model %s (%zu/%zu)", dr, show(r_rtw, dr).c_str(), show(m_rt, dr).c_str(),
             r_rtw.size(), m_rt.size());
    }
    printf("\n");
    if (dfl >= 0) {
      printf("  fields differ at #%ld (%zu rtl, %zu model)\n", dfl, r_fields.size(), m_fields.size());
      for (long i = dfl; i < dfl + 3; i++) {
        if (i < (long)r_fields.size()) printf("    rtl   %s\n", r_fields[i].c_str());
        if (i < (long)m_fields.size()) printf("    model %s\n", m_fields[i].c_str());
      }
    }
  } else if (!quiet)
    printf("ok   %s T%02d.%d  %zu flux, %zu index, %zu cells, inc %d-%d, %zu read track bytes, %zu fields\n", name, t,
           s, r_flux.size(), r_index.size(), r_cells.size(), r_min_inc, r_max_inc, r_rtw.size(), r_fields.size());
  fflush(stdout);
  return ok;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  int only_t = -1, only_s = -1, argi = 1;
  for (; argi < argc && argv[argi][0] == '-'; argi++) {
    if (!strcmp(argv[argi], "-t") && argi + 1 < argc) only_t = atoi(argv[++argi]);
    else if (!strcmp(argv[argi], "-s") && argi + 1 < argc) only_s = atoi(argv[++argi]);
    else if (!strcmp(argv[argi], "-q")) quiet = true;
    else if (!strcmp(argv[argi], "-d") && argi + 1 < argc) debug_n = atoi(argv[++argi]);
  }
  auto top = std::make_unique<Vhfe_tb_top>();
  Result total;
  for (; argi < argc; argi++) {
    Hfe h;
    if (!h.load(argv[argi])) {
      printf("%s: not an HFE file\n", argv[argi]);
      total.fail++;
      continue;
    }
    Result r;
    for (int t = 0; t < h.ntracks; t++)
      for (int s = 0; s < h.nsides; s++) {
        if ((only_t >= 0 && t != only_t) || (only_s >= 0 && s != only_s)) continue;
        test_side(top.get(), h, t, s, base(argv[argi]), r);
      }
    printf("%-44s %4ld sides, %3ld failed, %ld skipped | %ld flux, %ld index, %ld cells, inc %d-%d, %ld RT bytes, %ld fields\n",
           base(argv[argi]), r.sides, r.fail, r.skipped, r.flux, r.index, r.cells, r.min_inc, r.max_inc, r.rtbytes,
           r.fields);
    total.sides += r.sides; total.fail += r.fail; total.skipped += r.skipped;
  }
  printf("TOTAL %ld sides, %ld failed\n", total.sides, total.fail);
  return total.fail ? 1 : 0;
}
