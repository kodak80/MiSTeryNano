//
// mfm_amdetect.v - WD1772 MFM address mark detector, byte framing and CRC
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
// Takes the cells recovered by the DPLL and frames them into bytes, one
// byte per 16 cells (clock + data), the data cells being the byte.
//
// Marks: A1 with a missing clock (cells 0x4489) and C2 with a missing clock
// (0x5224). An A1 cannot overlap the previous A1, so a new one is only
// accepted 16 or more cells after the last, as in the WD1772. While
// sync_en is high (the controller is looking for an address mark, or doing
// READ TRACK) a mark re-frames: it is output as a byte at once and the next
// byte starts after it. While sync_en is low (reading the bytes of an ID
// or data field) the framing is left alone and mark patterns are plain data.
//
// CRC-CCITT (0x1021): preset on the third consecutive A1 mark to the CRC of
// A1 A1 A1 from 0xFFFF, then updated with every following byte, so after
// an ID or data field and its two CRC bytes, crc is 0 if the field is good.
//
// restart: framing and the A1 history start afresh with the next cell (the
// cell shift register keeps running), e.g. at the index for READ TRACK.
// If a cell arrives in the same clock, it is still counted before the
// restart.
//
// This is the hardware for read_track() and the field parsing in
// tools/hfev3/hfe_model.cpp.

module mfm_amdetect (
	input             clk,
	input             reset,

	input             bit_valid,   // a cell from the DPLL
	input             bit_val,

	input             sync_en,     // marks re-frame (searching / READ TRACK)
	input             restart,

	output reg        byte_valid,  // one clock per byte
	output reg  [7:0] byte_val,
	output reg        mark_a1,     // the byte is an A1 mark (only while sync_en)
	output reg        mark_c2,     // the byte is a C2 mark (only while sync_en)
	output reg  [1:0] a1_run,      // A1 marks right before this byte (saturates at 3)
	output reg [15:0] crc          // CRC after this byte
);

localparam [15:0] CRC_A1A1A1 = 16'hCDB4;  // CRC of A1 A1 A1 from 0xFFFF

function [15:0] crc_byte(input [15:0] c, input [7:0] d);
	integer i;
	reg [15:0] r;
	begin
		r = c ^ { d, 8'h00 };
		for (i = 0; i < 8; i = i + 1)
			r = r[15] ? ((r << 1) ^ 16'h1021) : (r << 1);
		crc_byte = r;
	end
endfunction

reg [15:0] sh;           // last 16 cells
reg  [3:0] cnt;          // cells in the current byte
reg  [4:0] since_a1;     // cells since the last accepted A1 (saturates)
reg        have_a1;
reg  [1:0] run;          // consecutive A1 mark bytes so far

wire [15:0] sh_n = { sh[14:0], bit_val };
wire  [4:0] a1_dist = (since_a1 == 5'd31) ? 5'd31 : since_a1 + 5'd1;
wire        is_a1 = (sh_n == 16'h4489) && (!have_a1 || a1_dist >= 5'd16);
wire        is_c2 = (sh_n == 16'h5224);
wire        mk    = sync_en && (is_a1 || is_c2);
wire        emit  = mk || cnt == 4'd15;
wire  [7:0] dbyte = { sh_n[14], sh_n[12], sh_n[10], sh_n[8],
                      sh_n[6],  sh_n[4],  sh_n[2],  sh_n[0] };
wire  [1:0] run_n = (run == 2'd3) ? 2'd3 : run + 2'd1;
wire [15:0] crc_n = crc_byte(crc, dbyte);

always @(posedge clk) begin
	byte_valid <= 1'b0;

	if (reset) begin
		sh       <= 16'd0;
		cnt      <= 4'd0;
		since_a1 <= 5'd0;
		have_a1  <= 1'b0;
		run      <= 2'd0;
		crc      <= 16'hFFFF;
	end else begin
		if (bit_valid) begin
			sh <= sh_n;

			// A1 spacing is tracked in every mode
			if (is_a1) begin
				since_a1 <= 5'd0;
				have_a1  <= 1'b1;
			end else
				since_a1 <= a1_dist;

			cnt <= emit ? 4'd0 : cnt + 4'd1;

			if (emit) begin
				byte_valid <= 1'b1;
				byte_val   <= dbyte;
				mark_a1    <= sync_en && is_a1;
				mark_c2    <= sync_en && is_c2;
				a1_run     <= run;
				if (sync_en && is_a1) begin
					run <= run_n;
					if (run_n == 2'd3) crc <= CRC_A1A1A1;
				end else begin
					run <= 2'd0;
					crc <= crc_n;
				end
			end
		end

		if (restart) begin
			cnt     <= 4'd0;
			have_a1 <= 1'b0;
			run     <= 2'd0;
		end
	end
end

endmodule
