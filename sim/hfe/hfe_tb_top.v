// hfe_tb_top.v - unit test top for the HFE floppy blocks
//
// HFE track player -> WD1772 DPLL -> mark detector / CRC, with an 8 MHz clock
// enable whose cycle 0 is the player's time 0, so cycle numbers match
// tools/hfev3/hfe_model.cpp. The (raw HFE) track buffer lives in the C++
// testbench.

module hfe_tb_top (
	input             clk,
	input             reset,
	input             v3,
	input       [7:0] nominal_x,
	input      [14:0] stream_len,
	input             idx_at_start,
	input      [31:0] seed,
	input             side,
	output     [15:0] ram_addr,
	input       [7:0] ram_data,

	output            start,
	output            flux,
	output            index,
	output            cell_end,
	output            cell_val,
	output            underrun,

	output            ce8,
	output            bit_valid,
	output            bit_val,
	output      [7:0] increment,

	// READ TRACK framing: always re-frames, restarted at the index
	output            rt_valid,
	output      [7:0] rt_byte,
	output            rt_a1,
	output            rt_c2,

	// field reading: sync_en driven by the testbench like the FDC would
	input             f_sync,
	output            f_valid,
	output      [7:0] f_byte,
	output            f_a1,
	output            f_c2,
	output      [1:0] f_a1_run,
	output     [15:0] f_crc
);

// 8 MHz enable: last clock of each 8 MHz cycle, cycle 0 starts with the
// player's time 0. Outputs of the player are registered, so its clock n
// is seen here one edge later, together with phase == n % 4.
reg  [1:0] phase;
reg        run;
always @(posedge clk) begin
	if (reset) begin
		phase <= 2'd0;
		run   <= 1'b0;
	end else if (start || run) begin
		run   <= 1'b1;
		phase <= phase + 2'd1;
	end
end
assign ce8 = (start || run) && phase == 2'd3;

wd1772_dpll dpll (
	.clk       ( clk       ),
	.reset     ( reset     ),
	.ce8       ( ce8       ),
	.flux      ( flux      ),
	.bit_valid ( bit_valid ),
	.bit_val   ( bit_val   ),
	.increment ( increment )
);

mfm_amdetect amd_rt (
	.clk        ( clk       ),
	.reset      ( reset     ),
	.bit_valid  ( bit_valid ),
	.bit_val    ( bit_val   ),
	.sync_en    ( 1'b1      ),
	.restart    ( index     ),
	.byte_valid ( rt_valid  ),
	.byte_val   ( rt_byte   ),
	.mark_a1    ( rt_a1     ),
	.mark_c2    ( rt_c2     ),
	.a1_run     (           ),
	.crc        (           )
);

mfm_amdetect amd_f (
	.clk        ( clk       ),
	.reset      ( reset     ),
	.bit_valid  ( bit_valid ),
	.bit_val    ( bit_val   ),
	.sync_en    ( f_sync    ),
	.restart    ( 1'b0      ),
	.byte_valid ( f_valid   ),
	.byte_val   ( f_byte    ),
	.mark_a1    ( f_a1      ),
	.mark_c2    ( f_c2      ),
	.a1_run     ( f_a1_run  ),
	.crc        ( f_crc     )
);

hfe_player player (
	.clk          ( clk          ),
	.reset        ( reset        ),
	.v3           ( v3           ),
	.nominal_x    ( nominal_x    ),
	.stream_len   ( stream_len   ),
	.idx_at_start ( idx_at_start ),
	.start_pos    ( 15'd0        ),
	.ff           ( 1'b0         ),
	.seed         ( seed         ),
	.side         ( side         ),
	.ram_addr     ( ram_addr     ),
	.ram_data     ( ram_data     ),
	.start        ( start        ),
	.flux         ( flux         ),
	.index        ( index        ),
	.cell_end     ( cell_end     ),
	.cell_val     ( cell_val     ),
	.cell_weak    (              ),
	.underrun     ( underrun     )
);

endmodule
