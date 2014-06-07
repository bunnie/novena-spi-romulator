//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2013, Andrew "bunnie" Huang
//
// See the NOTICE file distributed with this work for additional 
// information regarding copyright ownership.  The copyright holder 
// licenses this file to you under the Apache License, Version 2.0 
// (the "License"); you may not use this file except in compliance
// with the License.  You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing,
// code distributed under the License is distributed on an
// "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied.  See the License for the
// specific language governing permissions and limitations
// under the License.
//////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

//`define USE_ROMULATOR 1

module novena_fpga(
		   output wire       APOPTOSIS,
		   
		   input wire AUD6_TFS,
		   input wire AUD6_TXC,
		   input wire AUD6_TXD,
		   input wire AUD_MCLK,
		   input wire AUD_MIC_CLK,
		   input wire AUD_MIC_DAT,
		   
		   input wire BATT_NRST,
		   input wire BATT_REFLASH_ALRT,
		   
		   input wire CLK2_N,
		   input wire CLK2_P,
		   
		   input wire DDC_SCL,
		   input wire DDC_SDA,
		   
		   output wire ECSPI3_MISO,
		   input wire ECSPI3_MOSI,
		   input wire ECSPI3_RDY,
		   input wire ECSPI3_SCLK,
		   input wire ECSPI3_SS2,
		   
		   input wire EIM_BCLK,
		   input wire [1:0] EIM_CS,
		   inout wire [15:0] EIM_DA,
		   input wire [18:16] EIM_A,
		   input wire EIM_LBA,
		   input wire EIM_OE,
		   input wire EIM_RW,
		   input wire EIM_WAIT,
		   
		   output wire FPGA_LED2,
		   input wire FPGA_LSPI_CLK,
		   input wire FPGA_LSPI_CS,
		   input wire FPGA_LSPI_HOLD,
		   input wire FPGA_LSPI_MISO,
		   input wire FPGA_LSPI_MOSI,
		   input wire FPGA_LSPI_WP,
		   
		   input wire I2C3_SCL,
		   input wire I2C3_SDA,
		   
		   input wire SMB_SCL,
		   input wire SMB_SDA,
		   
		   input wire UART4_CTS,
		   input wire UART4_RTS,
		   input wire UART4_RXD,
		   input wire UART4_TXD,
		   
		   // input wire UIM_CLK,
		   // input wire UIM_DATA,
		   // input wire UIM_PWR,
		   // input wire UIM_PWRON,
		   // input wire UIM_RESET,

		   input wire  F_LVDS_CK_P0, // sck
		   input wire  F_LVDS_CK_N0, // si
		   input wire  F_DX18, // cs input
		   output wire F_LVDS_P11,  // so
		   output wire F_LVDS_P15,  // cs output

		   output wire F_LVDS_N7, // drivers
		   output wire F_LVDS_P7,
		   
		   input wire RESETBMCU
	 );

   wire [15:0] 		      eim_dout;
   wire [15:0] 		      eim_din;
   wire 		      clk;   // free-runs at 50 MHz, unbuffered
   wire 		      clk50; // zero-delay, DLL version of above. Use this.
   wire 		      clk100; // doubled-up version of the above. For time base applications.
   wire 		      bclk;  // NOTE: doesn't run until first CPU access to EIM; then free-runs at 133 MHz
   reg [23:0] 		      counter;
   
   wire 		      ddr3_dll_locked;
   wire 		      ddr3clk;
   
   
   wire 		      reset;
   reg 			      emulate_r;

   always @(posedge spiclk) begin
      emulate_r <= emulate;
   end

   assign F_LVDS_N7 = !emulate_r; // drives so when low -- set to 1 for bypass mode
   assign F_LVDS_P7 = 1'b0; // drives cs when low

   // P15 is cs output to SPINOR
   assign F_LVDS_P15 = emulate_r ? 1'b1 : F_DX18;
//   assign F_LVDS_P15 = F_DX18; // set this for bypass mode
//   assign F_LVDS_P15 = 1'b1; // set to 1 to disable SPINOR during emulation
//   assign F_LVDS_P11 = 1'bz;

   
   ////////////////////////////////////
   ///// MASTER RESET
   ////////////////////////////////////
   
   sync_reset master_res_sync( .glbl_reset(!RESETBMCU), .clk(clk), .reset(reset) );
     
   wire 	      bclk_dll, bclk_div2_dll, bclk_div4_dll, bclk_locked;
   wire 	      bclk_early;
   
   ////////////////////////////////////
   ///// BCLK DLL -- generate zero-delay clock plus slower versions for internal use
   ////////////////////////////////////
   wire 	      bclk_int_in, bclk_io_in;
   IBUFG   clkibufg (.I(EIM_BCLK), .O(bclk) );
   BUFG    bclk_dll_bufg(.I(bclk), .O(bclk_int_in) );
   
   bclk_dll bclk_dll_mod( .clk133in(bclk_int_in), .clk133(bclk_dll),
			  .RESET(reset), .LOCKED(bclk_locked));

   wire 	      i_reset, i_locked;
   wire 	      o_reset, o_locked;
   wire 	      bclk_i, bclk_o;
   wire 	      i_fbk_out, i_fbk_in;
   wire 	      o_fbk_out, o_fbk_in;
   
   dcm_delay bclk_i_dll( .clk133(bclk_int_in), .clk133out(bclk_i),
			  .CLKFB_IN(i_fbk_in), .CLKFB_OUT(i_fbk_out),
			  .RESET(i_reset), .LOCKED(i_locked));

   dcm_delay bclk_o_dll( .clk133(bclk_int_in), .clk133out(bclk_o),
			  .CLKFB_IN(o_fbk_in), .CLKFB_OUT(o_fbk_out),
			  .RESET(o_reset), .LOCKED(o_locked));
   
   // lock it to the input path
   BUFIO2FB bclk_o_fbk(.I(bclk_o), .O(o_fbk_in));
   // assign o_fbk_in = bclk_o;
//   BUFG bclk_io_fbk(.I(bclk_io), .O(io_fbk_in));
   
   assign i_fbk_in = bclk_i;

   ////////////////////////////////////
   ///// Register set -- area-inefficient, high fan-out/in registers for controlling/monitoring internal signals
   ///// All registers split into write or read only blanks
   ///// 0x40000 - 0x40FFF is reserved for w/o
   ///// 0x41000 - 0x41FFF is reserved for r/o
   /////   -> if you want to check a w/o value, loop it back to an r/o register
   ////////////////////////////////////
   
   reg 		      cs0_r, rw_r;
   reg [15:0] 	      din_r;
   reg [18:0] 	      bus_addr_r;
   reg 		      adv_r;

   reg 		      cs0_in, rw_in, adv_in;
   reg [15:0] 	      din_in;
   reg [2:0] 	      a_in;
   
   always @(posedge bclk_i) begin
      cs0_in <= EIM_CS[0];
      rw_in <= EIM_RW;
      din_in <= eim_din;
      adv_in <= !EIM_LBA; // latch address on LBA low
      a_in <= EIM_A[18:16];

      cs0_r <= cs0_in;
      rw_r <= rw_in;
      din_r <= din_in;
      adv_r <= adv_in;
   end
   
   always @(posedge bclk_i) begin 
      if( adv_in ) begin
	 bus_addr_r <= {a_in, din_in};
      end else begin
	 bus_addr_r <= bus_addr_r;
      end
   end

   wire [15:0] r40000wo;
   wire [15:0] r40002wo;

   wire [15:0] ro_d;

   //////// write-only registers
   reg_wo reg_wo_40000 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40000),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), .rbk_d(ro_d), 
			 .reg_d( r40000wo[15:0] ) );
   
   reg_wo reg_wo_40002 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40002),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(1'b0), .rbk_d(ro_d), // unreadable
			 .reg_d( r40002wo[15:0] ) );

   wire [15:0]        romulator_ctl;
   wire 	      emulate;
   wire 	      spi_uk_rd_en;
   wire 	      spi_uk_rst;
   wire 	      spi_out_rd_en;
   wire 	      spi_out_rst;
   wire 	      spi_adr_rd_en;
   wire 	      spi_adr_rst;
   assign emulate = romulator_ctl[0];       // 1
   assign spi_uk_rd_en = romulator_ctl[1];  // 2
   assign spi_uk_rst = romulator_ctl[2];    // 4
   assign spi_adr_rd_en = romulator_ctl[3]; // 8
   assign spi_adr_rst = romulator_ctl[4];   // 10
   assign spi_out_rd_en = romulator_ctl[5];  // 20
   assign spi_out_rst = romulator_ctl[6];    // 40
   
   reg_wo reg_wo_40010 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h40010),
			 .bus_d(din_r), .we(!cs0_r && !rw_r), .re(!cs0_r && rw_r), 
			 .rbk_d(ro_d), .reg_d( romulator_ctl[15:0] ) );


   //////// read-only registers
   // loopback readback
   reg_ro reg_ro_41000 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41000),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( r40000wo[15:0] ) );

   reg_ro reg_ro_41002 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41002),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( r40002wo[15:0] ) );


   wire [15:0] 	      romulator_stat;
   reg_ro reg_ro_41100 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41100),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( romulator_stat[15:0] ) );

   wire [15:0] 	      romulator_count;
   reg_ro reg_ro_41102 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41102),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( romulator_count[15:0] ) );

   wire [15:0] 	      romulator_adr_stat;
   reg_ro reg_ro_41104 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41104),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( romulator_adr_stat[15:0] ) );

   wire [15:0] 	      romulator_adr_count;
   reg_ro reg_ro_41106 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41106),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( romulator_adr_count[15:0] ) );

   wire [23:0] 	      romulator_adr_dout;
   reg_ro reg_ro_41108 ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41108),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( romulator_adr_dout[15:0] ) );

   reg_ro reg_ro_4110A ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4110A),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( {8'b0,romulator_adr_dout[23:16]} ) );

   wire [15:0] 	      romulator_out_stat;
   reg_ro reg_ro_4110C ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4110C),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( romulator_out_stat[15:0] ) );

   wire [15:0] 	      romulator_out_count;
   reg_ro reg_ro_4110E ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h4110E),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( romulator_out_count[15:0] ) );
   
   ///////////////////////
   ///////////////////////
   // CS1 bank registers: minimum size here is 64-bit, tuned for synchronous burst access only
   ///////////////////////

   wire [63:0] 	     rC04_0000wo;
   wire [63:0] 	     rC04_0008wo;
   wire [15:0] 	     ro_d_b;
   
   ///////// write registers
   // loopback test
   reg_wo_4burst reg_wo_4b_C04_0000( .clk(bclk_i), .bus_ad(eim_din), .my_a(19'h4_0000), 
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]), 
				     .reg_d( rC04_0000wo[63:0] ), .rbk_d(ro_d_b) );

   reg_wo_4burst reg_wo_4b_C04_0008( .clk(bclk_i), .bus_ad(eim_din), .my_a(19'h4_0008),
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
				     .reg_d( rC04_0008wo[63:0] ), .rbk_d(ro_d_b) );

   ///////// read registers
   // loopback test
   reg_ro_4burst reg_ro_4b_C04_1000( .clk(bclk_i), .bus_ad(eim_din), .my_a(19'h4_1000),
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
				     .reg_d( rC04_0000wo[63:0] ), .rbk_d(ro_d_b) );

   reg_ro_4burst reg_ro_4b_C04_1008( .clk(bclk_i), .bus_ad(eim_din), .my_a(19'h4_1008),
				     .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
				     .reg_d( rC04_0008wo[63:0] ), .rbk_d(ro_d_b) );

   // FPGA minor version code
   reg_ro reg_ro_41FFC ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41FFC),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( 16'h0001 ) ); // minor version

   // FPGA major version code
   reg_ro reg_ro_41FFE ( .clk(bclk_dll), .bus_a(bus_addr_r), .my_a(19'h41FFE),
			 .bus_d(ro_d), .re(!cs0_r && rw_r),
			 .reg_d( 16'h000A ) ); // 000A is for the SPI romulator

   ////////// VERSION LOG (major version 000A) /////////////
   //////
   // Minor version 0001, May 13 2014
   //   Initial cull to the SPI ROM feature set
   //
   
   // mux between block memory and register set based on high bits
   //   assign eim_dout = (bus_addr[18:16] != 3'b000) ? ro_d : bram_dout;
   // pipeline to improve timing
   reg [15:0]		     ro_d_r;
   reg [15:0] 		     ro_d_b_r;
   reg [1:0] 		     eim_rdcs;
   reg [15:0] 		     eim_dout_pipe;
   reg [15:0] 		     eim_dout_pipe2;

   always @(posedge bclk_i) begin
      ro_d_b_r <= ro_d_b;
   end
   
   always @(posedge bclk_dll) begin
      ro_d_r <= ro_d;
      eim_rdcs[1:0] <= EIM_CS[1:0];
      eim_dout_pipe <= (eim_rdcs[1:0] == 2'b10) ? ro_d_r : ro_d_b_r;
   end

   always @(posedge bclk_o) begin
      eim_dout_pipe2 <= eim_dout_pipe; // retime near the source to allow max time for wire delay
   end;

   wire [15:0] 	      spi_eeprom_rbk;
   wire [7:0] 	      spi_uk_cmd;
   wire 	      spi_uk_cmd_updated;

   wire 	      spiclk_i;
   wire 	      spiclk;
   
   IBUFG spi_clkibufg( .I(F_LVDS_CK_P0), .O(spiclk_i));
   BUFG spi_clkbufg( .I(spiclk_i), .O(spiclk) );

   wire [23:0] 	      spi_adr;
   wire 	      spi_adr_updated;

   wire [7:0] 	      spi_byte;
   wire 	      spi_byte_updated;
   
   wire [7:0] 	      spi_obyte;
   wire 	      spi_obyte_updated;
   
   spi_eeprom spi_eeprom(
			 .sdout(F_LVDS_P11),
			 .sdin(F_LVDS_CK_N0),
			 .scs(F_DX18),
			 .sclk(spiclk),
			 .swp(1'b1),
			 .shold(1'b1),

			 .eim_clk(bclk_i),
			 .bus_a(bus_addr_r),
			 .bus_d(din_r),
			 .we(!cs0_r && !rw_r),
			 .re(!cs0_r && rw_r),
			 .reg_d(spi_eeprom_rbk),
			 .rbk_d(ro_d),
		  
			 .spi_uk_cmd(spi_uk_cmd),  // bitbucket for unknown commands
			 .spi_uk_cmd_updated(spi_uk_cmd_updated),

			 .spi_byte(spi_byte),
			 .spi_byte_updated(spi_byte_updated),

			 .spi_obyte(spi_obyte),
			 .spi_obyte_updated(spi_obyte_updated),
			 
			 .spi_adr(spi_adr),
			 .spi_adr_updated(spi_adr_updated),
		  
			 .reset(reset)
			 );

   wire [11:0] 	      spi_uk_rd_data_count;
   wire 	      spi_uk_empty;
   wire 	      spi_uk_overflow;
   wire 	      spi_uk_full;
   wire [7:0] 	      spi_uk_dout;
   wire 	      spi_uk_rd_en_pulse;

   rising_edge spi_re( .clk(bclk_dll),
		       .level(spi_uk_rd_en),
		       .pulse(spi_uk_rd_en_pulse)
		       );

   assign romulator_stat[7:0] = spi_uk_dout[7:0];
   assign romulator_stat[8] = spi_uk_empty;
   assign romulator_stat[9] = spi_uk_full;
   assign romulator_stat[10] = spi_uk_overflow;
   assign romulator_count[11:0] = spi_uk_rd_data_count[11:0];
   
   uk_fifo spi_uk (
		   .rst(spi_uk_rst), // input rst
//		   .wr_clk(spiclk), // input wr_clk
		   .wr_clk(!spiclk), // input wr_clk  // invert for monitoring

//		   .din(spi_uk_cmd), // input [7 : 0] din
//		   .wr_en(spi_uk_cmd_updated), // input wr_en
		   .din(spi_byte),
		   .wr_en(spi_byte_updated),

		   .rd_clk(bclk_dll), // input rd_clk
		   .rd_en(spi_uk_rd_en_pulse), // input rd_en
		   .dout(spi_uk_dout), // output [7 : 0] dout
		   .full(spi_uk_full), // output full
		   .overflow(spi_uk_overflow), // output overflow
		   .empty(spi_uk_empty), // output empty
		   .rd_data_count(spi_uk_rd_data_count) // output [11 : 0] rd_data_count
		   );
   
   wire [11:0] 	      spi_out_rd_data_count;
   wire 	      spi_out_empty;
   wire 	      spi_out_overflow;
   wire 	      spi_out_full;
   wire [7:0] 	      spi_out_dout;
   wire 	      spi_out_rd_en_pulse;

   rising_edge spi_out_re( .clk(bclk_dll),
			   .level(spi_out_rd_en),
			   .pulse(spi_out_rd_en_pulse)
			   );

   assign romulator_out_stat[7:0] = spi_out_dout[7:0];
   assign romulator_out_stat[8] = spi_out_empty;
   assign romulator_out_stat[9] = spi_out_full;
   assign romulator_out_stat[10] = spi_out_overflow;
   assign romulator_out_count[11:0] = spi_out_rd_data_count[11:0];
   
   uk_fifo spi_out (
		   .rst(spi_out_rst), // input rst
		   .wr_clk(!spiclk), // input wr_clk
		    // invert from negedge sampling of sdout

		   .din(spi_obyte),
		   .wr_en(spi_obyte_updated),

		   .rd_clk(bclk_dll), // input rd_clk
		   .rd_en(spi_out_rd_en_pulse), // input rd_en
		   .dout(spi_out_dout), // output [7 : 0] dout
		   .full(spi_out_full), // output full
		   .overflow(spi_out_overflow), // output overflow
		   .empty(spi_out_empty), // output empty
		   .rd_data_count(spi_out_rd_data_count) // output [11 : 0] rd_data_count
		   );
   
   wire 	      spi_adr_rd_en_pulse;
   wire [23:0] 	      spi_adr_dout;
   wire 	      spi_adr_full;
   wire 	      spi_adr_overflow;
   wire 	      spi_adr_empty;
   wire [13:0] 	      spi_adr_rd_data_count;
   assign romulator_adr_count[13:0] = spi_adr_rd_data_count[13:0];
   assign romulator_adr_stat[8] = spi_adr_empty;
   assign romulator_adr_stat[9] = spi_adr_full;
   assign romulator_adr_stat[10] = spi_adr_overflow;

   assign romulator_adr_dout[23:0] = spi_adr_dout[23:0];
   
   rising_edge spi_adr_re( .clk(bclk_dll),
		       .level(spi_adr_rd_en),
		       .pulse(spi_adr_rd_en_pulse)
		       );
   nandadr_fifo spi_adr_fifo (
			 .rst(spi_adr_rst), // input rst
			 .wr_clk(spiclk), // input wr_clk
			 .rd_clk(bclk_dll), // input rd_clk
			 .din({6'b0,spi_adr[23:0]}), // input [29 : 0] din
			 .wr_en(spi_adr_updated), // input wr_en
			 .rd_en(spi_adr_rd_en_pulse), // input rd_en
			 .dout(spi_adr_dout[23:0]), // output [29 : 0] dout
			 .full(spi_adr_full), // output full
			 .overflow(spi_adr_overflow), // output overflow
			 .empty(spi_adr_empty), // output empty
			 .rd_data_count(spi_adr_rd_data_count) // output [13 : 0] rd_data_count
			 );
   
   //////////////
   // Output pipeline registers -- explicit instantiation as their LOCs are controlled in the UCF.
   //////////////
   FDSE oddr2_eim0( .D( eim_dout_pipe2[0] ),
		     .C( bclk_o ),
		     .Q( eim_dout[0] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim1( .D( eim_dout_pipe2[1] ),
		     .C( bclk_o ),
		     .Q( eim_dout[1] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim2( .D( eim_dout_pipe2[2] ),
		     .C( bclk_o ),
		     .Q( eim_dout[2] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim3( .D( eim_dout_pipe2[3] ),
		     .C( bclk_o ),
		     .Q( eim_dout[3] ),
		     .CE( 1'b1 ), .S(1'b0) );

   FDSE oddr2_eim4( .D( eim_dout_pipe2[4] ),
		     .C( bclk_o ),
		     .Q( eim_dout[4] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim5( .D( eim_dout_pipe2[5] ),
		     .C( bclk_o ),
		     .Q( eim_dout[5] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim6( .D( eim_dout_pipe2[6] ),
		     .C( bclk_o ),
		     .Q( eim_dout[6] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim7( .D( eim_dout_pipe2[7] ),
		     .C( bclk_o ),
		     .Q( eim_dout[7] ),
		     .CE( 1'b1 ), .S(1'b0) );

   FDSE oddr2_eim8( .D( eim_dout_pipe2[8] ),
		     .C( bclk_o ),
		     .Q( eim_dout[8] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eim9( .D( eim_dout_pipe2[9] ),
		     .C( bclk_o ),
		     .Q( eim_dout[9] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eimA( .D( eim_dout_pipe2[10] ),
		     .C( bclk_o ),
		     .Q( eim_dout[10] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eimB( .D( eim_dout_pipe2[11] ),
		     .C( bclk_o ),
		     .Q( eim_dout[11] ),
		     .CE( 1'b1 ), .S(1'b0) );

   FDSE oddr2_eimC( .D( eim_dout_pipe2[12] ),
		     .C( bclk_o ),
		     .Q( eim_dout[12] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eimD( .D( eim_dout_pipe2[13] ),
		     .C( bclk_o ),
		     .Q( eim_dout[13] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eimE( .D( eim_dout_pipe2[14] ),
		     .C( bclk_o ),
		     .Q( eim_dout[14] ),
		     .CE( 1'b1 ), .S(1'b0) );
   FDSE oddr2_eimF( .D( eim_dout_pipe2[15] ),
		     .C( bclk_o ),
		     .Q( eim_dout[15] ),
		     .CE( 1'b1 ), .S(1'b0) );
   

   //////////////
   /// "heartbeat" counter
   //////////////
   always @(posedge clk50) begin
      counter <= counter + 1;
   end

   assign FPGA_LED2 = counter[23];

   //////////////
   // IOBUFs as required by design
   //////////////
   IBUFGDS clkibufgds( .I(CLK2_P), .IB(CLK2_N), .O(clk) );

   reg [15:0]	      eim_d_t;
   reg 		      eim_lba_reg;
   reg 		      eim_oe_reg;

   always @(posedge bclk_i) begin
      eim_lba_reg <= EIM_LBA;
      eim_oe_reg <= EIM_OE;
   end
   
   always @(posedge bclk_o) begin
      eim_d_t[ 0] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 1] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 2] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 3] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 4] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 5] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 6] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 7] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 8] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[ 9] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[10] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[11] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[12] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[13] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[14] = eim_oe_reg | !eim_lba_reg;
      eim_d_t[15] = eim_oe_reg | !eim_lba_reg;
   end
   
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim0 (.IO(EIM_DA[ 0]), .I(eim_dout[ 0]), .T(eim_d_t), .O(eim_din[ 0]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim1 (.IO(EIM_DA[ 1]), .I(eim_dout[ 1]), .T(eim_d_t), .O(eim_din[ 1]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim2 (.IO(EIM_DA[ 2]), .I(eim_dout[ 2]), .T(eim_d_t), .O(eim_din[ 2]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim3 (.IO(EIM_DA[ 3]), .I(eim_dout[ 3]), .T(eim_d_t), .O(eim_din[ 3]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim4 (.IO(EIM_DA[ 4]), .I(eim_dout[ 4]), .T(eim_d_t), .O(eim_din[ 4]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim5 (.IO(EIM_DA[ 5]), .I(eim_dout[ 5]), .T(eim_d_t), .O(eim_din[ 5]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim6 (.IO(EIM_DA[ 6]), .I(eim_dout[ 6]), .T(eim_d_t), .O(eim_din[ 6]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim7 (.IO(EIM_DA[ 7]), .I(eim_dout[ 7]), .T(eim_d_t), .O(eim_din[ 7]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim8 (.IO(EIM_DA[ 8]), .I(eim_dout[ 8]), .T(eim_d_t), .O(eim_din[ 8]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim9 (.IO(EIM_DA[ 9]), .I(eim_dout[ 9]), .T(eim_d_t), .O(eim_din[ 9]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim10 (.IO(EIM_DA[10]), .I(eim_dout[10]), .T(eim_d_t), .O(eim_din[10]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim11 (.IO(EIM_DA[11]), .I(eim_dout[11]), .T(eim_d_t), .O(eim_din[11]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim12 (.IO(EIM_DA[12]), .I(eim_dout[12]), .T(eim_d_t), .O(eim_din[12]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim13 (.IO(EIM_DA[13]), .I(eim_dout[13]), .T(eim_d_t), .O(eim_din[13]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim14 (.IO(EIM_DA[14]), .I(eim_dout[14]), .T(eim_d_t), .O(eim_din[14]));
   IOBUF #(.DRIVE(12), .SLEW("FAST")) IOBUF_eim15 (.IO(EIM_DA[15]), .I(eim_dout[15]), .T(eim_d_t), .O(eim_din[15]));

   //////////////
   // DDR3 interface macro
   //////////////

   wire c1_clk0, c1_rst0;
   
   ddr3_clkgen ddr3_clkgen (
			    .clk50in(clk),
			    .clk50(clk50),
			    .clk400(ddr3clk),
			    .clk100(clk100),
			    .RESET(reset),
			    .LOCKED(ddr3_dll_locked)
			    );

   //////////////
   // tie downs (unused signals as of this rev of design)
   //////////////
   assign APOPTOSIS = 1'b0; // make apoptosis inactive, tigh high to force reboot on config
   assign ECSPI3_MISO = 1'b0;
   
endmodule
