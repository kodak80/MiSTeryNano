//
// wd1772_mfm.v - WD1772 command state machine working on a real MFM stream
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
// The CPU side (registers, IRQ/DRQ handshake, step rates, status layout)
// follows src/fdc1772/fdc1772.v so the two can sit side by side. The disk
// side is the byte stream of mfm_amdetect (fed by hfe_player and
// wd1772_dpll), so everything a protection checks comes from the disk:
//
// - ID search: A1 A1 A1 FE with the detector re-framing on marks
//   (sync_en), then the six ID bytes with sync_en low. Track and sector
//   are compared with the registers; there is no side compare. An ID with
//   a bad CRC sets the CRC error bit and the search goes on.
// - Data: A1 A1 A1 F8-FB whose first A1 ends within 46 bytes of the ID
//   CRC (the reference model's window); an A1 A1 A1 FE there ends the wait
//   and is taken as the next ID. Data bytes reach the data register as
//   the disk delivers them (DRQ, lost data), then the data CRC. F8/F9 set
//   the record type (deleted) bit.
// - Record not found / seek error after 5 index pulses without success.
// - Type I verify reads the ID fields of the track it ends on.
// - READ ADDRESS returns the next ID (its track goes to the sector
//   register); READ TRACK returns every byte from index to index with the
//   detector re-framing on A1/C2 and restarted at the index.
// - This path is read only: WRITE SECTOR/TRACK end at once with the write
//   protect bit set.
// - FORCE INTERRUPT: D0 stops, D8 interrupts now, D4 at every index pulse.
//
// Timers use clk8m_en (8 MHz). Disk bytes and index pulses are taken in
// whatever 32 MHz clock they come.

module wd1772_mfm #(
	parameter CLK_EN = 16'd8000     // clk8m_en rate in kHz
) (
	input             clk,          // 32 MHz
	input             clk8m_en,
	input             floppy_reset, // active low

	// CPU interface, as fdc1772.v
	input       [1:0] cpu_addr,
	input             cpu_sel,
	input             cpu_rw,
	input       [7:0] cpu_din,
	output reg  [7:0] cpu_dout,
	output reg        irq,
	output reg        drq,

	// drive
	input             dr_present,   // a disk is inserted
	input             dr_wp,        // write protected
	input             dr_index,     // index pulse (one clock)
	input             dr_track0,    // head on track 0
	output reg        step_pulse,   // one clock per step
	output reg        step_out,     // direction of that step: 1 = towards track 0
	output reg        motor_on,

	// MFM byte stream from mfm_amdetect
	input             mb_valid,
	input       [7:0] mb_byte,
	input             mb_a1,
	input             mb_c2,
	input       [1:0] mb_a1_run,
	input      [15:0] mb_crc,
	output            sync_en,
	output            rt_restart
);

localparam REG_CMD = 2'd0, REG_TRACK = 2'd1, REG_SECTOR = 2'd2, REG_DATA = 2'd3;

localparam MOTOR_IDLE_INDEX = 4'd10;  // as fdc1772.v
localparam SPINUP_INDEX     = 3'd6;
localparam SEARCH_INDEX     = 3'd5;
localparam DAM_WINDOW       = 6'd49;  // DAM byte at most 46 + 3 bytes after the ID CRC

// ----------------------------------------------------------------- registers

reg  [7:0] cmd;
reg  [7:0] track_r;
reg  [7:0] sector_r;
reg  [7:0] data_r;
reg        busy;
reg        type1_status;    // status shows the type I layout
reg        st_seek_err;     // type I: seek error / type II/III: record not found
reg        st_crc_err;
reg        st_lost;
reg        st_rtype;        // record type (deleted data mark)
reg        st_wp;
reg        st_spunup;

wire cmd_t1 = !cmd[7];
wire cmd_rd_addr  = cmd[7:4] == 4'b1100;
wire cmd_rd_track = cmd[7:4] == 4'b1110;
wire cmd_wr_track = cmd[7:4] == 4'b1111;

// ------------------------------------------------------- index and motor

// the index status bit is a ~4 ms pulse, as from a real drive
// a 1 ms tick: every delay of the WD1772 is a whole number of ms, so the
// prescaler restarts whenever a delay is loaded and the delay counts ms
localparam [15:0] MS_LAST16 = CLK_EN - 16'd1;
localparam [12:0] MS_LAST   = MS_LAST16[12:0];   // CLK_EN up to 8192 kHz
reg [12:0] ms_pre;
wire       ms_tick = clk8m_en && ms_pre == MS_LAST;

localparam [2:0] INDEX_MS = 3'd4;
reg  [2:0] index_cnt;
wire       index_level = index_cnt != 3'd0;

reg  [3:0] motor_idle;
reg  [2:0] spinup_cnt;
reg        irq_on_index;

// ---------------------------------------------------------- CPU access

reg cpu_selD;
always @(posedge clk) cpu_selD <= cpu_sel;
wire cpu_acc = cpu_sel && !cpu_selD;            // first clock of an access
wire cpu_we  = cpu_acc && !cpu_rw;
// a command is taken when idle; FORCE INTERRUPT always
wire new_cmd = cpu_we && cpu_addr == REG_CMD && (!busy || cpu_din[7:4] == 4'b1101);

wire [7:0] status = type1_status ?
	{ motor_on, dr_wp, st_spunup, st_seek_err, st_crc_err, dr_track0, index_level, busy } :
	{ motor_on, st_wp, st_rtype,  st_seek_err, st_crc_err, st_lost,   drq,          busy };

always @(*) begin
	cpu_dout = 8'h00;
	if (cpu_sel && cpu_rw)
		case (cpu_addr)
			REG_CMD:    cpu_dout = status;
			REG_TRACK:  cpu_dout = track_r;
			REG_SECTOR: cpu_dout = sector_r;
			REG_DATA:   cpu_dout = data_r;
		endcase
end

// ------------------------------------------------------------- the FSM

localparam
	S_IDLE      = 5'd0,
	S_SPINUP    = 5'd1,
	S_T1        = 5'd2,
	S_T1_STEP   = 5'd3,
	S_T1_WAIT   = 5'd4,
	S_VERIFY    = 5'd5,
	S_T2        = 5'd6,
	S_SRCH_ID   = 5'd7,
	S_RD_ID     = 5'd8,
	S_SRCH_DAM  = 5'd9,
	S_RD_DATA   = 5'd10,
	S_RT_WAIT   = 5'd11,
	S_RT        = 5'd12,
	S_NODISK    = 5'd13,
	S_DONE      = 5'd14;

reg  [4:0] state;
reg  [4:0] after_spinup;
reg  [7:0] dest;            // seek destination (the data register at command time)
reg  [7:0] restore_steps;
reg  [4:0] delay;           // in ms
reg  [2:0] idx_seen;        // index pulses during the current search
reg  [2:0] nbytes;          // ID bytes read
reg  [7:0] id_trk, id_sec, id_len;
reg  [5:0] dam_cnt;         // bytes since the ID CRC
reg [10:0] dcnt;            // data field bytes read
reg  [1:0] dlen;            // its size code (128 << dlen bytes)
reg        step_dir_out;    // last step direction

wire seek_like = cmd[7:5] == 3'b000;   // restore / seek: step until done
wire t1_upd    = seek_like || cmd[4];  // track register follows the steps

// step rates, as fdc1772.v for the WD1772
wire [4:0] step_rate =
	(cmd[1:0] == 2'b00) ? 5'd6  :
	(cmd[1:0] == 2'b01) ? 5'd12 :
	(cmd[1:0] == 2'b10) ? 5'd2  :
	                      5'd3;

// data field position: still in the data (dcnt < size), both CRC bytes in
// (dcnt == size + 1); size is a power of two, so neither needs an adder
wire in_data = (dlen == 2'd0) ? dcnt[10:7] == 4'd0 :
               (dlen == 2'd1) ? dcnt[10:8] == 3'd0 :
               (dlen == 2'd2) ? dcnt[10:9] == 2'd0 : !dcnt[10];
wire crc_in  = dcnt == ((11'd128 << dlen) | 11'd1);

wire searching = state == S_SRCH_ID || state == S_SRCH_DAM || state == S_VERIFY;
assign sync_en    = searching || state == S_RT_WAIT || state == S_RT;
assign rt_restart = state == S_RT_WAIT && dr_index;

// a disk byte that goes to the CPU
wire deliver = mb_valid && ((state == S_RD_ID && cmd_rd_addr) ||
                            (state == S_RD_DATA && in_data) ||
                            (state == S_RT && !dr_index));

// "a byte after three A1 marks", the start of an ID or data field
wire am_byte = mb_valid && mb_a1_run == 2'd3 && !mb_a1 && !mb_c2;

always @(posedge clk) begin
	step_pulse <= 1'b0;

	if (!floppy_reset) begin
		cmd          <= 8'h00;
		track_r      <= 8'h00;
		sector_r     <= 8'h00;
		data_r       <= 8'h00;
		busy         <= 1'b0;
		type1_status <= 1'b1;
		st_seek_err  <= 1'b0;
		st_crc_err   <= 1'b0;
		st_lost      <= 1'b0;
		st_rtype     <= 1'b0;
		st_wp        <= 1'b0;
		st_spunup    <= 1'b0;
		irq          <= 1'b0;
		drq          <= 1'b0;
		motor_on     <= 1'b0;
		motor_idle   <= 4'd0;
		spinup_cnt   <= 3'd0;
		irq_on_index <= 1'b0;
		index_cnt    <= 3'd0;
		ms_pre       <= 13'd0;
		state        <= S_IDLE;
		delay        <= 5'd0;
		step_dir_out <= 1'b0;
	end else begin
		// ------------------------------------------------ index
		if (clk8m_en) ms_pre <= ms_tick ? 13'd0 : ms_pre + 13'd1;

		if (dr_index)
			index_cnt <= INDEX_MS;
		else if (ms_tick && index_cnt != 3'd0)
			index_cnt <= index_cnt - 3'd1;

		if (dr_index) begin
			if (irq_on_index) irq <= 1'b1;
			if (spinup_cnt != 3'd0) spinup_cnt <= spinup_cnt - 3'd1;
			if (!busy && motor_on) begin
				if (motor_idle == 4'd1) motor_on <= 1'b0;
				if (motor_idle != 4'd0) motor_idle <= motor_idle - 4'd1;
			end
		end

		if (ms_tick && delay != 5'd0) delay <= delay - 5'd1;

		// ------------------------------------------------ CPU registers
		if (cpu_acc && cpu_addr == REG_CMD && cpu_rw) irq <= 1'b0;   // status read
		if (cpu_acc && cpu_addr == REG_DATA) drq <= 1'b0;

		if (cpu_we && cpu_addr == REG_TRACK  && !busy) track_r  <= cpu_din;
		if (cpu_we && cpu_addr == REG_SECTOR && !busy) sector_r <= cpu_din;
		if (cpu_we && cpu_addr == REG_DATA)            data_r   <= cpu_din;

		if (new_cmd) begin
					if (cpu_din[7:4] == 4'b1101) begin
						// ---------------- FORCE INTERRUPT
						if (!busy) begin
							type1_status <= 1'b1;
							st_seek_err  <= 1'b0;
							st_crc_err   <= 1'b0;
							motor_on     <= 1'b1;   // as fdc1772.v / Hatari (STF)
							motor_idle   <= MOTOR_IDLE_INDEX;
						end
						busy  <= 1'b0;
						state <= S_IDLE;
						drq   <= 1'b0;
						irq_on_index <= cpu_din[2];
						if (cpu_din[3]) irq <= 1'b1;
						cmd   <= cpu_din;
					end else begin
						cmd          <= cpu_din;
						busy         <= 1'b1;
						irq          <= 1'b0;
						drq          <= 1'b0;
						irq_on_index <= 1'b0;
						type1_status <= !cpu_din[7];
						st_seek_err  <= 1'b0;
						st_crc_err   <= 1'b0;
						st_lost      <= 1'b0;
						st_rtype     <= 1'b0;
						st_wp        <= 1'b0;
						dest         <= data_r;
						motor_idle   <= 4'd0;
						// motor on; without the h flag wait for the spin up
						if (motor_on) spinup_cnt <= 3'd0;
						else begin
							st_spunup <= 1'b0;
							if (!cpu_din[3]) spinup_cnt <= SPINUP_INDEX;
							else begin
								spinup_cnt <= 3'd0;
								st_spunup  <= 1'b1;
							end
						end
						motor_on     <= 1'b1;
						after_spinup <= !cpu_din[7] ? S_T1 : S_T2;
						state        <= S_SPINUP;
					end
		end else begin

		// ------------------------------------------------ command execution
		case (state)
		S_IDLE: ;

		S_SPINUP:
			if (spinup_cnt == 3'd0) begin
				st_spunup <= 1'b1;
				if (!dr_present) begin
					// no disk: give up after 6 ms, as fdc1772.v
					delay  <= 5'd6;
					ms_pre <= 13'd0;
					state <= S_NODISK;
				end else begin
					restore_steps <= 8'd0;
					if (cmd[7:4] == 4'b0000) track_r <= 8'hFF;  // restore
					// E flag: 15 ms head settling for type II/III
					if (cmd[7] && cmd[2]) begin
						delay  <= 5'd15;
						ms_pre <= 13'd0;
					end
					state <= after_spinup;
				end
			end

		S_NODISK:
			if (delay == 5'd0) begin
				st_seek_err <= 1'b1;
				state <= S_DONE;
			end

		// ---------------------------------------------------- type I
		S_T1: begin
			if (cmd[7:5] == 3'b000) begin
				// restore (towards track 0) / seek (to dest)
				if (cmd[4] == 1'b0 && dr_track0) begin
					track_r <= 8'd0;
					state   <= S_VERIFY;
				end else if (cmd[4] == 1'b0 && restore_steps == 8'd255) begin
					st_seek_err <= 1'b1;
					state <= S_DONE;
				end else if (cmd[4] == 1'b1 && track_r == dest) begin
					state <= S_VERIFY;
				end else begin
					step_dir_out <= cmd[4] == 1'b0 || dest < track_r;
					state <= S_T1_STEP;
				end
			end else begin
				// step (same direction), step in, step out
				if (cmd[6:5] == 2'b10) step_dir_out <= 1'b0;
				if (cmd[6:5] == 2'b11) step_dir_out <= 1'b1;
				state <= S_T1_STEP;
			end
			idx_seen <= 3'd0;
		end

		S_T1_STEP:
			if (step_dir_out && dr_track0) begin
				// stepping out on track 0: nothing to do
				track_r <= 8'd0;
				state   <= S_VERIFY;
			end else begin
				step_pulse <= 1'b1;
				step_out   <= step_dir_out;
				if (t1_upd) track_r <= step_dir_out ? track_r - 8'd1 : track_r + 8'd1;
				restore_steps <= restore_steps + 8'd1;
				delay  <= step_rate;
				ms_pre <= 13'd0;
				state <= S_T1_WAIT;
			end

		S_T1_WAIT:
			if (delay == 5'd0) begin
				idx_seen <= 3'd0;
				state <= seek_like ? S_T1 : S_VERIFY;
			end

		// verify: an ID on this track with a good CRC within 5 revolutions
		S_VERIFY:
			if (!cmd[2])
				state <= S_DONE;
			else if (dr_index && idx_seen == SEARCH_INDEX - 3'd1) begin
				st_seek_err <= 1'b1;
				state <= S_DONE;
			end else begin
				if (dr_index) idx_seen <= idx_seen + 3'd1;
				if (am_byte && mb_byte == 8'hFE) begin
					nbytes <= 3'd0;
					state  <= S_RD_ID;
				end
			end

		// ---------------------------------------------------- type II / III
		S_T2:
			if (cmd[7:5] == 3'b101 || cmd_wr_track) begin
				// write sector / write track: this path is read only
				st_wp <= 1'b1;
				state <= S_DONE;
			end else if (delay == 5'd0) begin
				idx_seen <= 3'd0;
				state <= cmd_rd_track ? S_RT_WAIT : S_SRCH_ID;
			end

		S_SRCH_ID, S_SRCH_DAM:
			if (dr_index && idx_seen == SEARCH_INDEX - 3'd1) begin
				st_seek_err <= 1'b1;   // record not found
				state <= S_DONE;
			end else begin
				if (dr_index) idx_seen <= idx_seen + 3'd1;
				if (am_byte && mb_byte == 8'hFE) begin
					nbytes <= 3'd0;
					state  <= S_RD_ID;
				end else if (state == S_SRCH_DAM && mb_valid) begin
					if (am_byte && mb_byte[7:2] == 6'b111110) begin
						// F8-FB: the data field
						st_rtype <= !mb_byte[1];
						dcnt  <= 11'd0;
						state <= S_RD_DATA;
					end else if (dam_cnt == DAM_WINDOW)
						state <= S_SRCH_ID;
					else
						dam_cnt <= dam_cnt + 6'd1;
				end
			end

		S_RD_ID: begin
			if (dr_index && idx_seen != SEARCH_INDEX - 3'd1) idx_seen <= idx_seen + 3'd1;
			if (mb_valid) begin
				nbytes <= nbytes + 3'd1;
				case (nbytes)
					3'd0: id_trk <= mb_byte;
					3'd1: ;
					3'd2: id_sec <= mb_byte;
					3'd3: id_len <= mb_byte;
					default: ;
				endcase
				if (nbytes == 3'd5) begin
					// the ID and its CRC are in, mb_crc is 0 if it is good
					if (cmd_rd_addr) begin
						sector_r   <= id_trk;
						st_crc_err <= mb_crc != 16'd0;
						state      <= S_DONE;
					end else if (cmd_t1) begin
						if (mb_crc != 16'd0) begin
							st_crc_err <= 1'b1;
							state <= S_VERIFY;
						end else if (id_trk == track_r) begin
							st_crc_err <= 1'b0;
							state <= S_DONE;
						end else
							state <= S_VERIFY;
					end else begin
						// read sector
						if (id_trk == track_r && id_sec == sector_r) begin
							if (mb_crc != 16'd0) begin
								st_crc_err <= 1'b1;
								state <= S_SRCH_ID;
							end else begin
								st_crc_err <= 1'b0;
								dlen    <= id_len[1:0];
								dam_cnt <= 6'd0;
								state   <= S_SRCH_DAM;
							end
						end else
							state <= S_SRCH_ID;
					end
				end
			end
		end

		S_RD_DATA:
			if (mb_valid) begin
				dcnt <= dcnt + 11'd1;
				// a data byte goes to the CPU (deliver)
				if (!in_data && crc_in) begin
					// both CRC bytes are in
					if (mb_crc != 16'd0) begin
						st_crc_err <= 1'b1;
						state <= S_DONE;
					end else if (cmd[4]) begin
						// multiple sectors: on to the next one
						sector_r <= sector_r + 8'd1;
						idx_seen <= 3'd0;
						state    <= S_SRCH_ID;
					end else
						state <= S_DONE;
				end
			end

		S_RT_WAIT:
			if (dr_index) state <= S_RT;

		S_RT:   // the bytes go to the CPU (deliver) until the index
			if (dr_index)
				state <= S_DONE;

		S_DONE: begin
			busy  <= 1'b0;
			irq   <= 1'b1;
			motor_idle <= MOTOR_IDLE_INDEX;
			state <= S_IDLE;
		end

		default: state <= S_IDLE;
		endcase

		// a byte for the CPU: READ ADDRESS, READ SECTOR data, READ TRACK
		if (deliver) begin
			if (drq) st_lost <= 1'b1;   // the last one was not taken
			data_r <= mb_byte;
			drq    <= 1'b1;
		end
		end
	end
end

endmodule
