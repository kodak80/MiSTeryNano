//
// hfe_player.v - plays one side of an HFE (v1/v3) track as flux transitions
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
// The player reads one side of a track from a buffer holding the raw HFE
// track as stored in the file (synchronous RAM, one clock latency): 512
// byte blocks, the first 256 bytes of each for side 0, the rest for
// side 1. The de-interleave is done in the address. It plays the side
// round and round like a spinning disk. It is the hardware version of play() + FluxSource
// in tools/hfev3/hfe_model.cpp and matches it cell for cell:
//
// - HFEv3 opcodes (bit-reversed in the file) as in Steem SSE with HFEv3:
//   BITRATE (x = rev8(param) if non-zero, resets skip), SKIPBITS
//   (skip = rev8(param) & 7 for the next byte), INDEX (first one in the
//   stream marks the index, resets skip), RAND (the byte's cells are cur_weak),
//   NOP (0x0F) and unknown opcodes are ignored (reset skip). A v1 image has none.
// - Cell timer: clk is 32 MHz, time is counted in 1/288 us. Each clock adds
//   9, a cell lasts 8*x (x/36 us), so every cell length is exact.
// - Weak cells get a fresh random value every time they are played, from
//   a 32 bit xorshift (a linear shift register), one step per weak cell.
//   It is the model's generator, so cur_weak tracks compare bit for bit.
// - x, skip and the INDEX state restart at the start of every revolution.
//
// A decoder runs ahead of the cell timer and keeps one decoded byte ready,
// so opcodes cost no disk time. 'underrun' flags if it ever falls behind
// (it never should: a byte lasts at least one cell, ~44 clocks).
//
// Time origin: 'start' goes high for one clock when playing begins. That
// clock is time 0 (the start of the first cell) and its 8 MHz cycle is 0.
//
// After a reset (a new track) playing starts at start_pos: the side's
// INDEX opcode, or 0 (the stream start is the index for v1 and for a v3
// side without one). The loader starts the player at the index time, so
// the disk keeps its rotation across track changes. The bytes before
// start_pos come after the wrap, as on every later revolution.
//
// ff (fast forward) makes the cell timer count 16 clocks in each clock, so
// the side plays 16 times as fast with every opcode, bitrate and skip as
// usual. hfe_floppy uses it to bring a newly loaded track round to where
// the disk is (the time since the index) and hides what it plays meanwhile.
// The cells stay longer than a clock (the shortest, x = 36, lasts 2) and
// the decoder needs 3 to 5 clocks a byte against at least 16.
//
// side: switching heads keeps the byte position (as FlashFloppy does). The
// byte being played finishes; the next one comes from the other side, with
// the bitrate back at nominal until the next BITRATE opcode. If the cell
// timer needs it before it is fetched, one blank (no flux) byte is played,
// much like a real drive switching heads.

module hfe_player (
	input             clk,          // 32 MHz
	input             reset,        // restart from the beginning of the stream

	input             v3,           // image is HFEv3: decode opcodes
	input       [7:0] nominal_x,    // cell length for the image bitrate (72 = 250 kbps)
	input      [14:0] stream_len,   // bytes in the side stream
	input             idx_at_start, // index at the stream start (v1, or v3 without INDEX)
	input      [14:0] start_pos,    // first byte played after a reset (0 in the model tests)
	input             ff,           // fast forward: time runs 16 times as fast (0 in the model tests)
	input      [31:0] seed,         // weak bit generator seed (must not be 0)
	input             side,

	// raw track buffer, one clock read latency
	output reg [15:0] ram_addr,
	input       [7:0] ram_data,

	output reg        start,        // first clock of playing (time 0)
	output reg        flux,         // flux transition at the end of a cell
	output reg        index,        // index pulse (start of the index cell)
	output reg        cell_end,     // a cell ends in this clock
	output reg        cell_val,     // its value
	output reg        cell_weak,    // it was a weak cell
	output reg        underrun      // decoder fell behind (sticky)
);

localparam [7:0] OP_INDEX = 8'h8F, OP_BITRATE = 8'h4F,
                 OP_SKIPBITS = 8'hCF, OP_RAND = 8'h2F;

function [7:0] rev8(input [7:0] v);
	rev8 = { v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7] };
endfunction

// ------------------------------------------------------------ decoder
// fetch-side state (in stream order, ahead of the cell timer)

localparam F_ADDR = 3'd0, F_WAIT = 3'd1, F_BYTE = 3'd2, F_PWAIT = 3'd3, F_PARAM = 3'd4;

reg  [2:0] fstate;
reg [14:0] fa;           // address of the byte being decoded
reg  [7:0] fx;           // current cell length
reg  [2:0] fskip;        // cells to skip in the next byte
reg        fidx_seen;    // an INDEX opcode was seen in this revolution
reg        fidx_pend;    // the next played byte starts with the index
reg        fop_skip;     // F_PARAM: 1 = SKIPBITS, 0 = BITRATE
reg        side_q;
reg        flushed;      // the prefetch was dropped by a side change

// decoded byte waiting to be played
reg        nb_valid;
reg  [7:0] nb_raw;
reg        nb_weak;
reg  [7:0] nb_x;
reg  [2:0] nb_skip;
reg        nb_idx;

wire [14:0] fa1 = fa + 15'd1;
wire        fa1_end = fa1 >= stream_len;
wire  [7:0] rd_rev  = rev8(ram_data);  // opcode parameter

// stream byte -> raw buffer address
function [15:0] raw_addr(input [14:0] a, input sd);
	raw_addr = { a[14:8], sd, a[7:0] };
endfunction

// ------------------------------------------------------------ cell timer

reg        running;
reg [11:0] acc;          // time into the current cell, 1/288 us
reg  [7:0] raw;          // byte being played
reg        cur_weak;
reg  [7:0] x;
reg  [2:0] k;            // cell (bit) within the byte, bit 0 first
reg [31:0] rng;

wire [11:0] acc_i = ff ? 12'd144 : 12'd9;   // 1/288 us per clock (16 clocks at once)
wire [11:0] acc_n = acc + acc_i;
wire [11:0] len8  = { 1'b0, x, 3'b000 };
wire [12:0] acc_d = { 1'b0, acc_n } - { 1'b0, len8 };   // its borrow is the compare
wire        ends  = running && !acc_d[12] && acc_d[11:0] != 12'd0;

// xorshift32, one step
wire [31:0] r1 = rng ^ (rng << 13);
wire [31:0] r2 = r1 ^ (r1 >> 17);
wire [31:0] r3 = r2 ^ (r2 << 5);

wire        cur_val = cur_weak ? r3[0] : raw[k];
wire        take    = !running ? nb_valid : (ends && k == 3'd7);


always @(posedge clk) begin
	start    <= 1'b0;
	flux     <= 1'b0;
	index    <= 1'b0;
	cell_end <= 1'b0;

	if (reset) begin
		fstate    <= F_ADDR;
		fa        <= start_pos;
		fx        <= nominal_x;
		fskip     <= 3'd0;
		fidx_seen <= 1'b0;
		fidx_pend <= idx_at_start;
		nb_valid  <= 1'b0;
		side_q    <= side;
		flushed   <= 1'b0;
		running   <= 1'b0;
		acc       <= 12'd0;
		rng       <= seed;
		underrun  <= 1'b0;
	end else begin
		// -------------------------------------------- cell timer
		if (ends) begin
			cell_end  <= 1'b1;
			cell_val  <= cur_val;
			cell_weak <= cur_weak;
			flux      <= cur_val;
			if (cur_weak) rng <= r3;
			acc <= acc_d[11:0];
			if (k != 3'd7) k <= k + 3'd1;
		end else if (running)
			acc <= acc_n;

		if (take) begin
			if (nb_valid) begin
				raw      <= nb_raw;
				cur_weak <= nb_weak;
				x        <= nb_x;
				k        <= nb_skip;
				index    <= nb_idx;   // the new cell starts in this clock
			end else begin
				// nothing fetched yet: a blank byte
				if (!flushed) underrun <= 1'b1;
				raw      <= 8'h00;
				cur_weak <= 1'b0;
				x        <= nominal_x;
				k        <= 3'd0;
			end
			if (!running) begin
				running <= 1'b1;
				start   <= 1'b1;
				acc     <= acc_i;   // time 0 is the start of this clock
			end
		end

		// -------------------------------------------- decoder
		if (take) nb_valid <= 1'b0;

		case (fstate)
			F_ADDR:
				if (!nb_valid || take) begin
					ram_addr <= raw_addr(fa, side);
					fstate   <= F_WAIT;
				end

			F_WAIT:  fstate <= F_BYTE;   // RAM samples ram_addr
			F_PWAIT: fstate <= F_PARAM;

			F_BYTE: begin
				// ram_data holds the byte at fa
				if (v3 && ram_data[3:0] == 4'hF && ram_data != OP_RAND) begin
					case (ram_data)
						OP_BITRATE, OP_SKIPBITS: begin
							if (!fa1_end) begin
								fop_skip <= (ram_data == OP_SKIPBITS);
								ram_addr <= raw_addr(fa1, side);
								fa       <= fa1;      // fa is now the parameter
								fstate   <= F_PWAIT;
							end else begin
								// opcode without its parameter at the end of the stream
								fa <= 15'd0;  fx <= nominal_x;  fskip <= 3'd0;
								fidx_seen <= 1'b0;  fidx_pend <= idx_at_start;
								fstate <= F_ADDR;
							end
						end
						OP_INDEX: begin
							if (!fidx_seen) begin
								fidx_seen <= 1'b1;
								fidx_pend <= 1'b1;
							end
							fskip <= 3'd0;
							if (fa1_end) begin
								fa <= 15'd0;  fx <= nominal_x;
								fidx_seen <= 1'b0;  fidx_pend <= idx_at_start;
							end else
								fa <= fa1;
							fstate <= F_ADDR;
						end
						default: begin  // NOP and unknown opcodes
							fskip <= 3'd0;
							if (fa1_end) begin
								fa <= 15'd0;  fx <= nominal_x;
								fidx_seen <= 1'b0;  fidx_pend <= idx_at_start;
							end else
								fa <= fa1;
							fstate <= F_ADDR;
						end
					endcase
				end else begin
					// a data byte, or a RAND (cur_weak) byte
					nb_valid <= 1'b1;
					flushed  <= 1'b0;
					nb_raw   <= ram_data;
					nb_weak  <= v3 && ram_data == OP_RAND;
					nb_x     <= fx;
					nb_skip  <= fskip;
					nb_idx   <= fidx_pend;
					fidx_pend <= 1'b0;
					fskip    <= 3'd0;
					if (fa1_end) begin
						fa <= 15'd0;  fx <= nominal_x;
						fidx_seen <= 1'b0;  fidx_pend <= idx_at_start;
					end else
						fa <= fa1;
					fstate <= F_ADDR;
				end
			end

			F_PARAM: begin
				if (fop_skip)
					fskip <= rd_rev[2:0];
				else begin
					if (rd_rev != 8'd0) fx <= rd_rev;
					fskip <= 3'd0;
				end
				if (fa1_end) begin
					fa <= 15'd0;  fx <= nominal_x;  fskip <= 3'd0;
					fidx_seen <= 1'b0;  fidx_pend <= idx_at_start;
				end else
					fa <= fa1;
				fstate <= F_ADDR;
			end

			default: fstate <= F_ADDR;
		endcase

		// -------------------------------------------- head switch
		side_q <= side;
		if (side != side_q) begin
			fstate   <= F_ADDR;   // drop any fetch in flight
			nb_valid <= 1'b0;
			fx       <= nominal_x;
			fskip    <= 3'd0;
			flushed  <= 1'b1;
		end
	end
end

endmodule
