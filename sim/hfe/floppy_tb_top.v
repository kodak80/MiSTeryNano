// floppy_tb_top.v - test top for the whole HFE floppy path
//
// hfe_floppy (image header, track loader, heads, rotation, player)
// -> wd1772_dpll -> mfm_amdetect -> wd1772_mfm, with a free running 8 MHz
// enable as in the core. The C++ testbench is the CPU and the SD card.

module floppy_tb_top (
	input             clk,
	input             reset,

	input       [1:0] floppy_drive,   // active low
	input             side,
	input       [1:0] img_mounted,
	input      [31:0] img_size,
	output      [1:0] hfe_valid,

	output     [31:0] sd_lba,
	output      [1:0] sd_rd,
	input             sd_ack,
	input       [8:0] sd_buff_addr,
	input       [7:0] sd_dout,
	input             sd_dout_strobe,

	input       [1:0] cpu_addr,
	input             cpu_sel,
	input             cpu_rw,
	input       [7:0] cpu_din,
	output      [7:0] cpu_dout,
	output            irq,
	output            drq,

	output            index,
	output            loading,
	output      [6:0] head_track,
	output            motor_on
);

reg [1:0] phase = 2'd0;
always @(posedge clk) phase <= phase + 2'd1;
wire ce8 = phase == 2'd3;

wire step_pulse, step_out, dr_present, dr_track0, flux;

hfe_floppy #(.FD_NUM(2)) floppy (
	.clk            ( clk            ),
	.clk8m_en       ( ce8            ),
	.reset          ( reset          ),
	.floppy_drive   ( floppy_drive   ),
	.side           ( side           ),
	.img_mounted    ( img_mounted    ),
	.img_size       ( img_size       ),
	.hfe_valid      ( hfe_valid      ),
	.sd_lba         ( sd_lba         ),
	.sd_rd          ( sd_rd          ),
	.sd_ack         ( sd_ack         ),
	.sd_buff_addr   ( sd_buff_addr   ),
	.sd_dout        ( sd_dout        ),
	.sd_dout_strobe ( sd_dout_strobe ),
	.sd_hold        ( 1'b0           ),
	.sd_busy        (                ),
	.step_pulse     ( step_pulse     ),
	.step_out       ( step_out       ),
	.dr_present     ( dr_present     ),
	.dr_track0      ( dr_track0      ),
	.dr_index       ( index          ),
	.flux           ( flux           ),
	.loading        ( loading        ),
	.head_track     ( head_track     )
);

wire bit_valid, bit_val;
wd1772_dpll dpll (
	.clk       ( clk       ),
	.reset     ( reset     ),
	.ce8       ( ce8       ),
	.flux      ( flux      ),
	.bit_valid ( bit_valid ),
	.bit_val   ( bit_val   ),
	.increment (           )
);

wire        sync_en, rt_restart;
wire        mb_valid, mb_a1, mb_c2;
wire  [7:0] mb_byte;
wire  [1:0] mb_a1_run;
wire [15:0] mb_crc;

mfm_amdetect amd (
	.clk        ( clk        ),
	.reset      ( reset      ),
	.bit_valid  ( bit_valid  ),
	.bit_val    ( bit_val    ),
	.sync_en    ( sync_en    ),
	.restart    ( rt_restart ),
	.byte_valid ( mb_valid   ),
	.byte_val   ( mb_byte    ),
	.mark_a1    ( mb_a1      ),
	.mark_c2    ( mb_c2      ),
	.a1_run     ( mb_a1_run  ),
	.crc        ( mb_crc     )
);

wd1772_mfm fdc (
	.clk          ( clk          ),
	.clk8m_en     ( ce8          ),
	.floppy_reset ( !reset       ),
	.cpu_addr     ( cpu_addr     ),
	.cpu_sel      ( cpu_sel      ),
	.cpu_rw       ( cpu_rw       ),
	.cpu_din      ( cpu_din      ),
	.cpu_dout     ( cpu_dout     ),
	.irq          ( irq          ),
	.drq          ( drq          ),
	.dr_present   ( dr_present   ),
	.dr_wp        ( 1'b1         ),
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
