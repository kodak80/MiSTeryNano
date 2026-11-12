// fdc_tb_top.v - test top for the WD1772 MFM controller
//
// hfe_player -> wd1772_dpll -> mfm_amdetect -> wd1772_mfm, the detector's
// sync_en/restart driven by the controller. The C++ testbench plays the
// CPU and the drive mechanics: it holds the track buffer, moves the head
// on step pulses and reloads the track (drive_reset restarts the disk
// chain; the controller keeps running).

module fdc_tb_top (
	input             clk,
	input             fdc_reset_n,
	input             drive_reset,

	// track buffer
	input             v3,
	input       [7:0] nominal_x,
	input      [14:0] stream_len,
	input             idx_at_start,
	input      [31:0] seed,
	input             side,
	output     [15:0] ram_addr,
	input       [7:0] ram_data,

	// drive
	input             dr_present,
	input             dr_wp,
	input             dr_track0,
	output            step_pulse,
	output            step_out,
	output            motor_on,

	// CPU
	input       [1:0] cpu_addr,
	input             cpu_sel,
	input             cpu_rw,
	input       [7:0] cpu_din,
	output      [7:0] cpu_dout,
	output            irq,
	output            drq,

	// observation
	output            start,
	output            index,
	output            underrun
);

wire flux;

hfe_player player (
	.clk          ( clk          ),
	.reset        ( drive_reset  ),
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
	.cell_end     (              ),
	.cell_val     (              ),
	.cell_weak    (              ),
	.underrun     ( underrun     )
);

// 8 MHz enable aligned with the player's time 0 (see hfe_tb_top.v), free
// running otherwise so the controller's timers keep going
reg  [1:0] phase;
reg        run;
always @(posedge clk) begin
	phase <= start ? 2'd1 : phase + 2'd1;
	if (drive_reset) run <= 1'b0;
	else if (start)  run <= 1'b1;
end
wire ce8      = phase == 2'd3 && !start;
wire dpll_ce8 = ce8 && run;      // the DPLL's cycle 0 is the player's time 0

wire bit_valid, bit_val;
wd1772_dpll dpll (
	.clk       ( clk         ),
	.reset     ( drive_reset ),
	.ce8       ( dpll_ce8    ),
	.flux      ( flux        ),
	.bit_valid ( bit_valid   ),
	.bit_val   ( bit_val     ),
	.increment (             )
);

wire        sync_en, rt_restart;
wire        mb_valid, mb_a1, mb_c2;
wire  [7:0] mb_byte;
wire  [1:0] mb_a1_run;
wire [15:0] mb_crc;

mfm_amdetect amd (
	.clk        ( clk         ),
	.reset      ( drive_reset ),
	.bit_valid  ( bit_valid   ),
	.bit_val    ( bit_val     ),
	.sync_en    ( sync_en     ),
	.restart    ( rt_restart  ),
	.byte_valid ( mb_valid    ),
	.byte_val   ( mb_byte     ),
	.mark_a1    ( mb_a1       ),
	.mark_c2    ( mb_c2       ),
	.a1_run     ( mb_a1_run   ),
	.crc        ( mb_crc      )
);

wd1772_mfm fdc (
	.clk          ( clk          ),
	.clk8m_en     ( ce8          ),
	.floppy_reset ( fdc_reset_n  ),
	.cpu_addr     ( cpu_addr     ),
	.cpu_sel      ( cpu_sel      ),
	.cpu_rw       ( cpu_rw       ),
	.cpu_din      ( cpu_din      ),
	.cpu_dout     ( cpu_dout     ),
	.irq          ( irq          ),
	.drq          ( drq          ),
	.dr_present   ( dr_present   ),
	.dr_wp        ( dr_wp        ),
	.dr_index     ( index        ),
	.dr_track0    ( dr_track0    ),
	.step_pulse   ( step_pulse   ),
	.step_out     ( step_out     ),
	.motor_on     ( motor_on     ),
	.mb_valid     ( mb_valid     ),
	.mb_byte      ( mb_byte      ),
	.mb_a1        ( mb_a1        ),
	.mb_c2        ( mb_c2        ),
	.mb_a1_run    ( mb_a1_run    ),
	.mb_crc       ( mb_crc       ),
	.sync_en      ( sync_en      ),
	.rt_restart   ( rt_restart   )
);

endmodule
