//
// wd1772_dpll.v - WD1772 digital data separator (DPLL)
//
// Copyright (c) 2026 kodak80 and contributors
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// Port of Steem SSE's TWD1772Dpll (patent US 4808884, via MAME's wd_fdc),
// as modelled by Dpll::next_bit in tools/hfev3/hfe_model.cpp.
//
// An 11 bit counter is stepped once per 8 MHz cycle by 'increment'
// (nominally 128, so a cell is 2048/128 = 16 cycles = 2 us). A cell ends
// when the counter overflows; its value is 1 if a flux transition arrived
// during the cell. Where in the cell the transition arrived (the counter
// value then) sets phase corrections (pa/ps: big/small counter steps in the
// first 8 cycles of the next cell) and, with a 2 bit history of early/late
// transitions, frequency corrections (fa/fs: increment +-1, limited to
// 117..140).
//
// Only the first transition in a cell counts, as in the model.
//
// Timing: ce8 is the 8 MHz enable, high in the last 32 MHz clock of each
// 8 MHz cycle. A flux pulse in any clock of that cycle (the ce8 clock
// included) belongs to it. The first ce8 after reset is cycle 0 and is not
// stepped (the model starts at time 0 and steps to cycle 1 first); a
// transition in cycle 0 is ignored, again like the model.

module wd1772_dpll (
	input             clk,         // 32 MHz
	input             reset,
	input             ce8,         // 8 MHz enable, end of each 8 MHz cycle
	input             flux,        // flux transition pulse (one clk)

	output reg        bit_valid,   // a cell ended in this ce8 step
	output reg        bit_val,     // its value
	output reg  [7:0] increment
);

reg        first;
reg        fpend;           // a transition arrived earlier in this 8 MHz cycle
reg [10:0] counter;
reg [10:0] tt;              // counter value when the transition arrived
reg        tt_valid;
reg  [7:0] history;
reg  [4:0] slot;
reg  [7:0] pa, ps, fa, fs;  // correction masks for slots 0..7

wire        fl     = fpend | flux;
wire        catch  = !tt_valid && fl;
wire [10:0] tt_eff = tt_valid ? tt : counter;
wire        have   = tt_valid | fl;

wire        early  = slot < 5'd8;
wire  [2:0] s3     = slot[2:0];
wire  [7:0] add    = !early   ? increment :
                     pa[s3]   ? 8'd226 :
                     ps[s3]   ? 8'd30  : increment;
wire [11:0] cnt_n  = { 1'b0, counter } + { 4'd0, add };

// correction tables (pha, phs, freqa, freqs in the model)
wire [2:0] cslot = tt_eff[10:8];
wire       way   = tt_eff[10];

// Each table is 8 entries of 4 bits (slot masks), indexed by cslot. Kept
// as flat constants (nested function calls for these tables gave wrong
// masks in simulation with version 5.020 of the simulator).
localparam [31:0] T_PHA  = 32'h0000_137F;  // cslot 7..0: 0 0 0 0 1 3 7 F
localparam [31:0] T_PHS  = 32'hF731_0000;  //            F 7 3 1 0 0 0 0
localparam [31:0] T_F12A = 32'h0000_0137;  // freqa, history 1 or 2
localparam [31:0] T_F12S = 32'h7310_0000;  // freqs, history 1 or 2

wire [3:0] m_pha  = T_PHA [cslot*4 +: 4];
wire [3:0] m_phs  = T_PHS [cslot*4 +: 4];
wire [3:0] m_f12a = T_F12A[cslot*4 +: 4];
wire [3:0] m_f12s = T_F12S[cslot*4 +: 4];

// history update
wire [7:0] h_mid = history[7] ? (way ? 8'h80 : 8'h83) :
                   history[6] ? (way ? (history & 8'h02) : ((history & 8'h02) | 8'h01)) :
                   history;
wire [7:0] h_new = way ? ((h_mid >> 1) | 8'h02) : (h_mid >> 1);

// frequency masks: freqa/freqs[history & 3][cslot]
wire [3:0] m_fa = (h_mid[1:0] == 2'd0) ? m_pha  :
                  (h_mid[1:0] == 2'd3) ? 4'h0   : m_f12a;
wire [3:0] m_fs = (h_mid[1:0] == 2'd0) ? 4'h0   :
                  (h_mid[1:0] == 2'd3) ? m_phs  : m_f12s;

always @(posedge clk) begin
	bit_valid <= 1'b0;

	if (reset) begin
		first     <= 1'b1;
		fpend     <= 1'b0;
		counter   <= 11'd0;
		increment <= 8'd128;
		tt_valid  <= 1'b0;
		history   <= 8'h80;
		slot      <= 5'd0;
		pa <= 8'd0;  ps <= 8'd0;  fa <= 8'd0;  fs <= 8'd0;
	end else if (ce8) begin
		fpend <= 1'b0;
		if (first)
			first <= 1'b0;
		else begin
			// frequency correction in the first 8 slots of a cell
			if (early && fa[s3] && increment < 8'd140)
				increment <= increment + 8'd1;
			else if (early && fs[s3] && increment > 8'd117)
				increment <= increment - 8'd1;

			if (cnt_n[11]) begin
				// end of cell
				bit_valid <= 1'b1;
				bit_val   <= have;
				counter   <= cnt_n[10:0];
				slot      <= 5'd0;
				tt_valid  <= 1'b0;
				if (have) begin
					pa      <= { 4'h0, m_pha };
					ps      <= { 4'h0, m_phs };
					fa      <= { 4'h0, m_fa };
					fs      <= { 4'h0, m_fs };
					history <= h_new;
				end else begin
					pa <= 8'd0;  ps <= 8'd0;  fa <= 8'd0;  fs <= 8'd0;
				end
			end else begin
				counter <= cnt_n[10:0];
				slot    <= slot + 5'd1;
				if (catch) begin
					tt       <= counter;
					tt_valid <= 1'b1;
				end
			end
		end
	end else if (flux)
		fpend <= 1'b1;
end

endmodule
