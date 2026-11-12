//
// hfe_floppy.v - HFE (v1/v3) floppy drives: image header, track loader,
// head positions, rotation, and the track player
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
// The disk side of the HFE floppy path, for wd1772_mfm via wd1772_dpll and
// mfm_amdetect:
//
// - Mount: when img_mounted pulses for a drive, block 0 of the image is
//   read. The HXCPICFE (v1) or HXCHFEV3 (v3) signature makes it an HFE
//   drive (hfe_valid), anything else (an .st) leaves it to fdc1772.v. The
//   track table (4 bytes per track: block offset, byte length) is kept per
//   drive, and the nominal cell length is worked out from the bitrate
//   (x = 18000 / kbps, 72 for 250 kbps).
// - One raw track buffer (BUF_BYTES, both sides as stored in the file) for
//   the selected drive. A track is loaded block by block over the SD
//   interface of fdc1772.v whenever the head of the selected drive moves,
//   another drive is selected, or an image is mounted. Both sides are in
//   the buffer, so changing sides is immediate. While the blocks stream
//   in, each side's opcodes are scanned for an INDEX opcode.
// - Heads: one per drive, moved by the controller's step pulses (0..85).
// - Rotation: the time since the last index is always kept, as is the
//   length of a revolution. While nothing plays (loading, an empty or
//   missing track, no disk) the index pulses are made from that, so they
//   never stop. A track that is ready starts playing at once from the
//   side's index (its INDEX opcode, or the stream start), fast forwarded
//   (16 times as fast, nothing given to the controller) until it has
//   played for as long as it has been since the index; from then on it
//   plays at the disk's speed, where the disk is. So the index keeps its
//   period across seeks (HFE tracks start at the index, so the disk keeps
//   its rotation), and the drive reads within a few ms of the track being
//   in, as a real drive does once the head has settled: game loaders time
//   a SEEK with verify for an ID field to come along within a sector, not
//   after up to a revolution. If the index comes first (fast forward takes
//   at most 1/15 of the time since the index) the track starts there, and
//   if the side's stream comes round to its index first (a revolution
//   longer than the stream) that index is given and it plays on from there.
//   Until it plays the drive reads nothing, as if the head were still
//   settling.
//
// Weak bits are seeded from the rotation time when a load ends, so they
// differ from load to load.

module hfe_floppy #(
	parameter FD_NUM    = 2,
	parameter BUF_BYTES = 40960      // raw track buffer (largest track seen: 40,298);
	                                 // 32 KB + a power of two (32K + 8K, 16K, ...)
) (
	input                   clk,           // 32 MHz
	input                   clk8m_en,
	input                   reset,

	input      [FD_NUM-1:0] floppy_drive,  // drive select, active low (as fdc1772.v)
	input                   side,          // head: 0 = side 0, 1 = side 1

	input      [FD_NUM-1:0] img_mounted,   // pulse: an image was (un)mounted
	input            [31:0] img_size,
	output reg [FD_NUM-1:0] hfe_valid,     // the drive holds an HFE image

	// SD card, as fdc1772.v (sd_lba is relative to the image)
	output           [31:0] sd_lba,
	output reg [FD_NUM-1:0] sd_rd,
	input                   sd_ack,
	input             [8:0] sd_buff_addr,
	input             [7:0] sd_dout,
	input                   sd_dout_strobe,
	input                   sd_hold,       // the card is in use by someone else: don't start
	output                  sd_busy,       // our request or transfer is under way

	// head stepping from the controller
	input                   step_pulse,
	input                   step_out,      // 1 = towards track 0

	// to the controller / DPLL
	output                  dr_present,    // the selected drive holds an HFE image
	output                  dr_track0,
	output                  dr_index,      // one clock
	output                  flux,          // one clock

	// status
	output                  loading,
	output            [6:0] head_track
);

localparam WIDX = (FD_NUM > 1) ? $clog2(FD_NUM) : 1;

// HFE offsets are 16 bit block numbers
reg [15:0] sd_lba16;
assign sd_lba = { 16'd0, sd_lba16 };
localparam [7:0] OP_INDEX = 8'h8F, OP_BITRATE = 8'h4F, OP_SKIPBITS = 8'hCF;
localparam [23:0] REV_DEFAULT = 24'd1600000;     // 200 ms in 8 MHz cycles

// --------------------------------------------------------------- selection

reg [WIDX-1:0] sel;
reg            sel_any;
always @(*) begin : select
	integer i;
	sel = 0;
	sel_any = 1'b0;
	for (i = FD_NUM - 1; i >= 0; i = i - 1)
		if (!floppy_drive[i]) begin
			sel = i[WIDX-1:0];
			sel_any = 1'b1;
		end
end

// --------------------------------------------------------------- per drive

// Kept as packed registers, not arrays, so synthesis never makes RAMs of
// these few multi-ported values.
reg [FD_NUM*7-1:0] head_r;     // head position per drive
reg [FD_NUM-1:0]   hv_v3;      // HFE v3 image
reg [FD_NUM*8-1:0] hv_trks_r;  // tracks in the image
reg [FD_NUM-1:0]   hv_ds;      // double sided image
reg [FD_NUM*8-1:0] hv_x_r;     // nominal cell length

wire [6:0] head_sel = head_r[sel*7 +: 7];
wire [7:0] trks_sel = hv_trks_r[sel*8 +: 8];

assign head_track = head_sel;
assign dr_track0  = head_sel == 7'd0;
assign dr_present = hfe_valid[sel];

always @(posedge clk) begin : heads
	if (reset) begin
		head_r <= {FD_NUM*7{1'b0}};
	end else if (step_pulse && sel_any) begin
		if (step_out && head_sel != 7'd0)        head_r[sel*7 +: 7] <= head_sel - 7'd1;
		else if (!step_out && head_sel != 7'd85) head_r[sel*7 +: 7] <= head_sel + 7'd1;
	end
end

// --------------------------------------------------------------- memories

// track table: 1 KB per drive (256 tracks x offset16, length16)
// (the drive bit of the address is there only with more than one drive)
localparam LUTW = (FD_NUM > 1) ? WIDX + 10 : 10;
reg  [7:0] lut [0:FD_NUM*1024-1];
reg [WIDX+9:0] lut_waddr, lut_raddr;
reg        lut_we;
reg  [7:0] lut_wdata, lut_q;
always @(posedge clk) begin
	if (lut_we) lut[lut_waddr[LUTW-1:0]] <= lut_wdata;
	lut_q <= lut[lut_raddr[LUTW-1:0]];
end

// raw track buffer: a 32 KB RAM plus one for the rest (BUF_BYTES - 32 KB,
// a power of two), so each is a plain power-of-two memory for synthesis.
// The player stays within the loaded track.
localparam integer BUF_HI   = BUF_BYTES - 32768;
localparam integer BUF_HI_W = (BUF_HI > 1) ? $clog2(BUF_HI) : 1;

reg  [7:0] tbuf_lo [0:32767];
reg  [7:0] tbuf_hi [0:BUF_HI-1];
reg [15:0] buf_waddr;
reg        buf_we;
reg  [7:0] buf_wdata;
wire [15:0] buf_raddr;
reg  [7:0] buf_q_lo, buf_q_hi;
reg        buf_rhi;
wire [7:0] buf_q = buf_rhi ? buf_q_hi : buf_q_lo;

always @(posedge clk) begin
	if (buf_we && !buf_waddr[15]) tbuf_lo[buf_waddr[14:0]] <= buf_wdata;
	buf_q_lo <= tbuf_lo[buf_raddr[14:0]];
end
always @(posedge clk) begin
	if (buf_we && buf_waddr[15]) tbuf_hi[buf_waddr[BUF_HI_W-1:0]] <= buf_wdata;
	buf_q_hi <= tbuf_hi[buf_raddr[BUF_HI_W-1:0]];
	buf_rhi  <= buf_raddr[15];
end

// --------------------------------------------------------------- rotation

reg  [23:0] rot_t;         // 8 MHz cycles since the last index
reg  [23:0] rot_len;       // length of the last revolution
wire        p_index;
reg         playing;
wire [23:0] rot_t1  = rot_t + 24'd1;
// the index is due: a revolution of the last length (or 262 ms at most)
wire        v_due   = clk8m_en && (rot_t1 >= rot_len || rot_t[23:21] != 3'd0);
// the virtual index, while nothing plays; not when a track starts with it
// (the player gives that index)
wire        v_start;
wire        v_index = !playing && v_due && !v_start;
// fast forward: the index the player starts with is not given (the disk
// passed it rot_t ago), a second one is (and ends the fast forward)
reg         ff, ff_idx, p_restart;
reg  [25:0] ff_t;          // time played since that index, 32 MHz clocks
assign      dr_index = playing ? p_index && (!ff || ff_idx) : v_index;

always @(posedge clk) begin
	if (reset) begin
		rot_t   <= 24'd0;
		rot_len <= REV_DEFAULT;
	end else if (dr_index) begin
		// a revolution of a real track: take its length (within reason)
		// a revolution of a real track (65..262 ms) sets the length
		if (playing && rot_t[23:21] == 3'd0 && rot_t[20:19] != 2'd0) rot_len <= rot_t;
		rot_t <= 24'd0;
	end else if (clk8m_en && !rot_t[23])
		rot_t <= rot_t1;
end

// --------------------------------------------------------------- loader

localparam
	L_IDLE    = 4'd0,
	L_SDREQ   = 4'd1,   // request a block
	L_SDWAIT  = 4'd2,   // block streaming in
	L_HDR     = 4'd3,   // header block in
	L_DIV     = 4'd4,   // nominal x = (18000 + kbps/2) / kbps
	L_TABLE   = 4'd5,   // track table blocks
	L_ENTRY   = 4'd6,   // read a track table entry
	L_TRACK   = 4'd7,   // track blocks
	L_WAITIDX = 4'd8,   // track loaded: start it at the next index
	L_PLAY    = 4'd9;

localparam integer BUF_BLKS_I = BUF_BYTES / 512;
localparam integer BUF_SIDE_I = BUF_BYTES / 2;  // longest side stream that fits
localparam [7:0]  BUF_BLKS = BUF_BLKS_I[7:0];
localparam [14:0] BUF_SIDE = BUF_SIDE_I[14:0];

reg  [3:0] lstate, lret;
reg [FD_NUM-1:0] mount_pend;
reg [WIDX-1:0] ld_drive;    // drive being worked on
reg  [1:0] ld_kind;         // block being read: 0 header, 1 track table, 2 track
reg        ackD;

// header
reg        sig_v1, sig_v3;
reg  [7:0] h_trks, h_sides;
reg [15:0] h_kbps, h_lut;
reg [14:0] div_rem;
reg  [8:0] div_q;
reg  [1:0] tbl_blk;

// the track in the buffer
reg            tag_valid;
reg [WIDX-1:0] tag_drive;
reg  [6:0]     tag_track;
reg  [2:0]     ent_cnt;
reg [15:0]     t_off;
reg  [7:0]     t_len_lo;
reg  [7:0]     t_blk, t_nblk;
reg [14:0]     s_len;          // bytes per side stream
reg  [1:0]     has_index;      // per side: an INDEX opcode in the stream
reg [29:0]     idx_pos;        // per side: where its first INDEX opcode is
reg  [1:0]     want_param;     // per side: the next byte is an opcode parameter

reg [31:0]     p_seed;

assign loading = lstate != L_IDLE && lstate != L_PLAY;
// the player starts at the side's index (its INDEX opcode) and gives it
assign v_start = lstate == L_WAITIDX && s_len != 15'd0;
assign sd_busy = (|sd_rd) || lstate == L_SDWAIT;

// which track should be in the buffer
wire want_load = sel_any && hfe_valid[sel] &&
                 (!tag_valid || tag_drive != sel || tag_track != head_sel);

// the entry's length once its last byte is read, and what follows from it
wire [15:0] e_len  = { lut_q, t_len_lo };
wire  [7:0] e_nblk = e_len[15:9] + { 7'd0, e_len[8:0] != 9'd0 };
wire [14:0] e_slen = e_len[15:1];

// byte index within its side for the block being loaded
wire [14:0] side_idx = { t_blk[6:0], sd_buff_addr[7:0] };
wire        blk_done = lstate == L_SDWAIT && ackD && !sd_ack;

always @(posedge clk) begin : loader
	integer i;

	buf_we <= 1'b0;
	lut_we <= 1'b0;
	ackD   <= sd_ack;
	if (sd_ack) sd_rd <= 0;

	if (reset) begin
		lstate     <= L_IDLE;
		mount_pend <= 0;
		hfe_valid  <= 0;
		sd_rd      <= 0;
		tag_valid  <= 1'b0;
		playing    <= 1'b0;
		ff         <= 1'b0;
		p_restart  <= 1'b0;
	end else begin
		// ------------------------------------------- bytes of the block being read
		if (lstate == L_SDWAIT && sd_ack && sd_dout_strobe) begin
			case (ld_kind)
				2'd0: case (sd_buff_addr)  // header
					9'd0: begin sig_v1 <= sd_dout == "H"; sig_v3 <= sd_dout == "H"; end
					9'd1: begin sig_v1 <= sig_v1 && sd_dout == "X"; sig_v3 <= sig_v3 && sd_dout == "X"; end
					9'd2: begin sig_v1 <= sig_v1 && sd_dout == "C"; sig_v3 <= sig_v3 && sd_dout == "C"; end
					9'd3: begin sig_v1 <= sig_v1 && sd_dout == "P"; sig_v3 <= sig_v3 && sd_dout == "H"; end
					// "HXCP" (HXCPICFE, v1) or "HXCH" (HXCHFEV3, v3) is enough
					9'd9:  h_trks       <= sd_dout;
					9'd10: h_sides      <= sd_dout;
					9'd12: h_kbps[7:0]  <= sd_dout;
					9'd13: h_kbps[15:8] <= sd_dout;
					9'd18: h_lut[7:0]   <= sd_dout;
					9'd19: h_lut[15:8]  <= sd_dout;
					default: ;
				endcase
				2'd1: begin  // track table
					lut_we    <= 1'b1;
					lut_waddr <= { ld_drive, tbl_blk[0], sd_buff_addr };
					lut_wdata <= sd_dout;
				end
				default: begin  // track
					if (t_blk < BUF_BLKS) begin
						buf_we    <= 1'b1;
						buf_waddr <= { t_blk[6:0], sd_buff_addr };
						buf_wdata <= sd_dout;
					end
					// opcode scan of each side's stream
					if (hv_v3[ld_drive] && side_idx < s_len) begin
						if (want_param[sd_buff_addr[8]])
							want_param[sd_buff_addr[8]] <= 1'b0;
						else if (sd_dout == OP_INDEX) begin
							has_index[sd_buff_addr[8]] <= 1'b1;
							if (!has_index[sd_buff_addr[8]]) begin
								if (sd_buff_addr[8]) idx_pos[29:15] <= side_idx;
								else                 idx_pos[14:0]  <= side_idx;
							end
						end
						else if (sd_dout == OP_BITRATE || sd_dout == OP_SKIPBITS)
							want_param[sd_buff_addr[8]] <= 1'b1;
					end
				end
			endcase
		end
		if (blk_done && ld_kind == 2'd1) tbl_blk <= tbl_blk + 2'd1;
		if (blk_done && ld_kind == 2'd2) t_blk   <= t_blk + 8'd1;

		// ------------------------------------------- the loader
		case (lstate)
		L_IDLE, L_PLAY:
			if (|mount_pend) begin
				for (i = FD_NUM - 1; i >= 0; i = i - 1)
					if (mount_pend[i]) ld_drive <= i[WIDX-1:0];
				playing <= 1'b0;
				ld_kind <= 2'd0;
				sd_lba16 <= 16'd0;
				lret    <= L_HDR;
				lstate  <= L_SDREQ;
			end else if (want_load) begin
				playing   <= 1'b0;
				ld_drive  <= sel;
				tag_drive <= sel;
				tag_track <= head_sel;
				if ({ 1'b0, head_sel } >= trks_sel) begin
					// beyond the end of the image: an empty track
					tag_valid <= 1'b1;
					lstate    <= L_IDLE;
				end else begin
					ent_cnt   <= 3'd0;
					lut_raddr <= { sel, 1'b0, head_sel, 2'd0 };
					lstate    <= L_ENTRY;
				end
			end

		L_SDREQ:
			if (!sd_ack && !ackD && !sd_hold) begin
				sd_rd[ld_drive] <= 1'b1;
				lstate <= L_SDWAIT;
			end

		L_SDWAIT:
			if (blk_done) lstate <= lret;

		L_HDR:
			if ((sig_v1 || sig_v3) && h_trks != 8'd0 && h_sides != 8'd0 && h_sides <= 8'd2) begin
				hv_v3[ld_drive]   <= sig_v3;
				hv_trks_r[ld_drive*8 +: 8] <= h_trks;
				hv_ds[ld_drive]   <= h_sides == 8'd2;
				hv_x_r[ld_drive*8 +: 8] <= 8'd72;
				div_rem <= 15'd18000 + { 5'd0, h_kbps[10:1] };
				div_q   <= 9'd0;
				tbl_blk <= 2'd0;
				lstate  <= (h_kbps >= 16'd50 && h_kbps <= 16'd2000) ? L_DIV : L_TABLE;
			end else begin
				mount_pend[ld_drive] <= 1'b0;   // not an HFE image
				lstate <= L_IDLE;
			end

		L_DIV:
			if (div_rem >= { 4'd0, h_kbps[10:0] }) begin
				div_rem <= div_rem - { 4'd0, h_kbps[10:0] };
				div_q   <= div_q + 9'd1;
			end else begin
				hv_x_r[ld_drive*8 +: 8] <= div_q[7:0];
				lstate <= L_TABLE;
			end

		L_TABLE:
			// 4 bytes per track: one block up to 128 tracks, two above
			if (tbl_blk == 2'd2 || (tbl_blk == 2'd1 && h_trks <= 8'd128)) begin
				hfe_valid[ld_drive]  <= 1'b1;
				mount_pend[ld_drive] <= 1'b0;
				if (tag_drive == ld_drive) tag_valid <= 1'b0;
				lstate <= L_IDLE;
			end else begin
				ld_kind <= 2'd1;
				sd_lba16 <= h_lut + { 14'd0, tbl_blk };
				lret    <= L_TABLE;
				lstate  <= L_SDREQ;
			end

		L_ENTRY: begin
			// the table RAM answers one clock after its address
			ent_cnt   <= ent_cnt + 3'd1;
			lut_raddr <= lut_raddr + 1'd1;
			case (ent_cnt)
				3'd1: t_off[7:0]  <= lut_q;
				3'd2: t_off[15:8] <= lut_q;
				3'd3: t_len_lo    <= lut_q;
				3'd4: begin
					t_nblk     <= (e_nblk > BUF_BLKS) ? BUF_BLKS : e_nblk;
					s_len      <= (e_slen > BUF_SIDE) ? BUF_SIDE : e_slen;
					t_blk      <= 8'd0;
					has_index  <= 2'b00;
					idx_pos    <= 30'd0;
					want_param <= 2'b00;
					lstate     <= L_TRACK;
				end
				default: ;
			endcase
		end

		L_TRACK:
			if (want_load && (tag_drive != sel || tag_track != head_sel)) begin
				// the head moved on (or another drive): load that track instead
				lstate <= L_IDLE;
			end else if (t_blk == t_nblk) begin
				// complete: it plays from the next index
				tag_valid <= 1'b1;
				p_seed    <= { rot_t, rot_len[7:0] } | 32'd1;   // when the load ends
				lstate    <= L_WAITIDX;
			end else begin
				ld_kind <= 2'd2;
				sd_lba16 <= t_off + { 8'd0, t_blk };
				lret    <= L_TRACK;
				lstate  <= L_SDREQ;
			end

		L_WAITIDX:
			if (|mount_pend || want_load || !tag_valid)
				lstate <= L_IDLE;                // something else to do first
			else begin
				// at the index: play from it; otherwise catch the disk up
				playing <= s_len != 15'd0;       // an empty side stays blank
				ff      <= s_len != 15'd0 && !v_due;
				ff_idx  <= 1'b0;
				ff_t    <= 26'd0;
				lstate  <= L_PLAY;
			end

		default: lstate <= L_IDLE;
		endcase

		// fast forward until the player has played for as long as it has been
		// since the index (rot_t, 8 MHz cycles)
		p_restart <= 1'b0;
		if (ff) begin
			if (!playing)
				ff <= 1'b0;
			else if (v_due) begin
				ff        <= 1'b0;               // the index came first: start there
				p_restart <= 1'b1;
			end else if (p_index) begin
				if (ff_idx) ff <= 1'b0;          // the stream's next index: given
				ff_idx <= 1'b1;
				ff_t   <= 26'd0;
			end else begin
				ff_t <= ff_t + 26'd16;
				if (ff_idx && ff_t >= { rot_t, 2'b00 }) ff <= 1'b0;
			end
		end

		// a mount (or unmount) takes the drive out of service at once
		for (i = 0; i < FD_NUM; i = i + 1)
			if (img_mounted[i]) begin
				mount_pend[i] <= |img_size;
				hfe_valid[i]  <= 1'b0;
				if (tag_drive == i[WIDX-1:0]) begin
					tag_valid <= 1'b0;
					playing   <= 1'b0;
				end
			end
	end
end

// --------------------------------------------------------------- player

// side 1 of a single sided image is blank
wire p_flux;
assign flux = p_flux && !ff && (hv_ds[tag_drive] || !side);

hfe_player player (
	.clk          ( clk                          ),
	.reset        ( !playing || p_restart        ),
	.v3           ( hv_v3[tag_drive]             ),
	.nominal_x    ( hv_x_r[tag_drive*8 +: 8]     ),
	.stream_len   ( s_len                        ),
	.idx_at_start ( !hv_v3[tag_drive] || !has_index[side] ),
	.start_pos    ( side ? idx_pos[29:15] : idx_pos[14:0] ),
	.ff           ( ff                           ),
	.seed         ( p_seed                       ),
	.side         ( side & hv_ds[tag_drive]      ),
	.ram_addr     ( buf_raddr                    ),
	.ram_data     ( buf_q                        ),
	.start        (                              ),
	.flux         ( p_flux                       ),
	.index        ( p_index                      ),
	.cell_end     (                              ),
	.cell_val     (                              ),
	.cell_weak    (                              ),
	.underrun     (                              )
);

endmodule
