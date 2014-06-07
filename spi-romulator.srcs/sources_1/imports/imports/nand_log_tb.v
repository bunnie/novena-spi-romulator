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

module nand_log_tb;
   reg bclk;  // 133 Mhz clock
   reg clk100;  // 100 Mhz clock

   reg nand_re;
   reg nand_we;
   reg nand_ale;
   reg nand_cle;
   reg nand_cs;
   reg nand_rb;
   reg [7:0] nand_din;
   reg [9:0] nand_uk;

   reg 	     log_reset;
   reg 	     log_run;
   wire      log_cmd_error; // stuck high if cmd fifo overflowed during logging
   wire      log_data_error; // stuck high if data fifo overflowed during logging
   wire [26:0] log_entries; // number of entries currently in the log
   
   wire [3:0]  ddr3_wr_mask;
   wire [31:0] ddr3_wr_data;
   wire        ddr3_wr_en;
   reg 	       ddr3_wr_full;
   reg [6:0]   ddr3_wr_count;
   wire        ddr3_cmd_clk;
   wire [2:0]  ddr3_cmd_instr;
   wire        ddr3_cmd_en;
   wire [5:0]  ddr3_cmd_burstlen;
   wire [29:0] ddr3_cmd_addr;
   reg 	       ddr3_cmd_full;

   wire [63:0] time_t_clk100; // note synched to clk100
   reg 	       reset;

nand_log uut(
		  bclk,  // 133 Mhz clock
		  clk100,  // 100 Mhz clock

		  nand_re,
		  nand_we,
		  nand_ale,
		  nand_cle,
		  nand_cs,
		  nand_rb,
		  nand_din,
		  nand_uk,

		  log_reset,
		  log_run,
		  log_cmd_error, // stuck high if cmd fifo overflowed during logging
		  log_data_error, // stuck high if data fifo overflowed during logging
		  log_entries, // number of entries currently in the log

		  ddr3_wr_mask,
		  ddr3_wr_data,
		  ddr3_wr_en,
		  ddr3_wr_full,
		  ddr3_wr_count,
		  ddr3_cmd_clk,
		  ddr3_cmd_instr,
		  ddr3_cmd_en,
		  ddr3_cmd_burstlen,
		  ddr3_cmd_addr,
		  ddr3_cmd_full,

		  time_t_clk100, // note synched to clk100
		  reset
		);

   parameter PERIOD_BCLK = 16'd8;   // 125 MHz (close to 133 MHz actual)
   always begin
      bclk = 1'b0;
      #(PERIOD_BCLK/2) bclk = 1'b1;
      #(PERIOD_BCLK/2);
   end

   parameter PERIOD_CLK100 = 16'd10;
   always begin
      clk100 = 1'b0;
      #(PERIOD_CLK100/2) clk100 = 1'b1;
      #(PERIOD_CLK100/2);
   end

   task nand_reset;
      begin
	 nand_we = 1'b1;
	 nand_re = 1'b1;
	 nand_ale = 1'b0;
	 nand_cle = 1'b0;
	 nand_cs = 1'b1;
	 nand_din = 8'hZZ;

	 log_reset = 1'b1;
	 log_run = 1'b0;
	 #1000;
	 log_reset = 1'b0;
	 log_run = 1'b1;
	 #1000;
      end
   endtask // nand_reset
   
   task nand_idle;
      begin
	 nand_we = 1'b1;
	 nand_re = 1'b1;
	 nand_ale = 1'b0;
	 nand_cle = 1'b0;
	 nand_cs = 1'b1;
	 nand_din = 8'hZZ;
      end
   endtask // nand_idle

   task nand_read_id;
      begin
	 nand_cs = 1'b0;
	 
	 nand_cle = 1'b1;
	 nand_we = 1'b0;
	 nand_din = 8'h90;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 nand_cle = 1'b0;
	 nand_din = 8'h01;
	 #20;

	 nand_ale = 1'b1;
	 #25;

	 nand_we = 1'b0;
	 nand_din = 8'h00;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 nand_din = 8'h23;
	 #20;

	 nand_ale = 1'b0;

	 #10;
	 nand_re = 1'b0;
	 nand_din = 8'h45;
	 #25;
	 nand_re = 1'b1;
	 #25;
	 nand_re = 1'b0;
	 nand_din = 8'h67;
	 #25;
	 nand_re = 1'b1;
	 #25;
	 nand_re = 1'b0;
	 nand_din = 8'h89;
	 #25;
	 nand_re = 1'b1;
	 #25;
	 nand_re = 1'b0;
	 nand_din = 8'hAB;
	 #25;
	 nand_re = 1'b1;
	 #25;
	 nand_re = 1'b0;
	 nand_din = 8'hCD;
	 #25;
	 nand_re = 1'b1;
	 #25;

	 nand_cs = 1'b1;
      end
   endtask; // nand_read_id

   initial begin
      nand_re = 1;
      nand_we = 1;
      nand_ale = 0;
      nand_cle = 0;
      nand_rb = 1;
      nand_din = 8'h00;
      nand_uk[9:0] = 10'h0;
      nand_cs = 1;

      log_reset = 1'b0;
      log_run = 1'b0;
      ddr3_wr_full = 1'b0;
      ddr3_wr_count = 7'b0;
      ddr3_cmd_full = 1'b0;
      
      reset = 1;
      #1000;
      reset = 0;
      nand_reset();
      #1000;
      
      nand_idle();
      #200;
      nand_read_id();

      $stop;

      #1000;
      $stop;
   end
   
endmodule // nand_log_tb


