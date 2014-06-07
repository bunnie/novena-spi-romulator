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

module nand_log(
		input wire bclk,  // 133 Mhz clock
		input wire clk100,  // 100 Mhz clock

		input wire nand_re,
		input wire nand_we,
		input wire nand_ale,
		input wire nand_cle,
		input wire nand_cs,
		input wire nand_rb,
		input wire [7:0] nand_din,
		input wire [9:0] nand_uk,

		input wire log_reset,
		input wire log_run,
		output reg log_cmd_error, // stuck high if cmd fifo overflowed during logging
		output reg log_data_error, // stuck high if data fifo overflowed during logging
		output reg [26:0] log_entries, // number of entries currently in the log

		output wire [3:0] ddr3_wr_mask,
		output wire [31:0] ddr3_wr_data,
		output wire ddr3_wr_en,
		input wire ddr3_wr_full,
		input wire ddr3_wr_empty,
		input wire [6:0] ddr3_wr_count,
		output wire ddr3_cmd_clk,
		output wire [2:0] ddr3_cmd_instr,
		output wire ddr3_cmd_en,
		output wire [5:0] ddr3_cmd_burstlen,
		output wire [29:0] ddr3_cmd_addr,
		input wire ddr3_cmd_full,
		input wire ddr3_cmd_empty,

		output wire [63:0] time_t_clk100, // note synched to clk100
		input wire reset
		);

   wire 		   bclk_reset, clk100_reset;
   sync_reset log_bclk_res_sync( .glbl_reset(reset), .clk(bclk), .reset(bclk_reset) );
   sync_reset log_clk100_res_sync( .glbl_reset(reset), .clk(clk100), .reset(clk100_reset) );
     
   reg [7:0] 		   cap_wr_din;
   reg [9:0] 		   cap_wr_uk;
   reg 			   cap_wr_ale;
   reg 			   cap_wr_cle;
   reg 			   cap_wr_cs;
   reg 			   cap_wr_rb;

   reg [7:0] 		   cap_rd_din;
   reg [9:0] 		   cap_rd_uk;
   reg 			   cap_rd_ale;
   reg 			   cap_rd_cle;
   reg 			   cap_rd_cs;
   reg 			   cap_rd_rb;

   reg [7:0] 		   log_din;
   reg [9:0] 		   log_uk;
   reg 			   log_ale;
   reg 			   log_cle;
   reg 			   log_cs;
   reg 			   log_rb;
   reg 			   log_we;
   reg 			   log_re;
   reg 			   log_capture_pulse; // whenever this goes high, grab the log_* data
   
   wire 		   cap_we;
   wire 		   cap_re;
   reg 			   cap_wed2, cap_wed1, cap_wedA;
   reg 			   cap_red2, cap_red1, cap_redA;
   wire 		   time_we;
   wire 		   time_re;

   ////////////
   // Sync the asynchronous NAND data into the bclk domain
   //
   // 1. capture data with the rising edges of the NAND we, re signals
   // 2. sync NAND we, re into bclk domain
   // 3. detect rising edges of we, re
   // 4. use rising edges to latch captured data
   // why this works:
   // the data capture stabilizes the data busses for long periods of time
   // as long as the pulse detection on the we, re signals is shorter than the
   // min cycle length of the NAND, we will always be guaranteed to grab "good"
   // data from the capture registers.
   // bclk runs at 133 MHz -> 7.5ns cycle length
   // pulse occurs max two cycles later, which is 15ns: shortest we, re cycle time is 25 ns.
   // so we should be 100% solid on the data captures.
   ////////////
   always @(posedge nand_we) begin
      cap_wr_din[7:0] <= nand_din[7:0];
      cap_wr_uk[9:0] <= nand_uk[9:0];
      
      cap_wr_ale <= nand_ale;
      cap_wr_cle <= nand_cle;
      cap_wr_cs <= nand_cs;
      cap_wr_rb <= nand_rb;
   end

   always @(posedge nand_re) begin
      cap_rd_din[7:0] <= nand_din[7:0];
      cap_rd_uk[9:0] <= nand_uk[9:0];
      
      cap_rd_ale <= nand_ale;
      cap_rd_cle <= nand_cle;
      cap_rd_cs <= nand_cs;
      cap_rd_rb <= nand_rb;
   end

   always @(posedge bclk) begin
//      cap_wedA <= nand_we;  // extra stage for synching into local clock domain
//      cap_redA <= nand_re;
      
//      cap_wed1 <= cap_wedA;
//      cap_red1 <= cap_redA;
      cap_wed1 <= nand_we;
      cap_red1 <= nand_re;
      cap_wed2 <= cap_wed1;
      cap_red2 <= cap_red1;
   end
   assign cap_we = !cap_wed2 & cap_wed1; // rising edge pulse gen
   assign cap_re = !cap_red2 & cap_red1;

   /////////
   // capture a single entry of log information on rising edges of we, re, synchronized to bclk
   /////////
   always @(posedge bclk) begin
      log_capture_pulse <= cap_we || cap_re;
      if( cap_we ) begin
	 log_din <= cap_wr_din;
	 log_uk <= cap_wr_uk;
	 log_ale <= cap_wr_ale;
	 log_cle <= cap_wr_cle;
	 log_cs <= cap_wr_cs;
	 log_rb <= cap_wr_rb;
	 log_we <= 1'b0;  // we is active low
	 log_re <= 1'b1;
      end else if( cap_re ) begin
	 log_din <= cap_rd_din;
	 log_uk <= cap_rd_uk;
	 log_ale <= cap_rd_ale;
	 log_cle <= cap_rd_cle;
	 log_cs <= cap_rd_cs;
	 log_rb <= cap_rd_rb;
	 log_we <= 1'b1;
	 log_re <= 1'b0;  // re is active low
      end else begin
	 log_din <= log_din;
	 log_uk <= log_uk;
	 log_ale <= log_ale;
	 log_cle <= log_cle;
	 log_cs <= log_cs;
	 log_rb <= log_rb;
	 log_we <= log_we;
	 log_re <= log_re;
      end // else: !if( cap_re )
   end // always @ (posedge bclk)

   /*
    * PACKET_NAND_CYCLE format (FPGA):
    * Offset | Size | Description
    * --------+------+-------------
    * 0 | 11 | Header
    * 11 | 1 | Data/Command pins
    * 12 | 1 | Bits [0..4] are ALE, CLE, WE, RE, and CS (in order)
    * 13 | 2 | Bits [0..9] are the unknown pins
    */   
   wire [22:0] ddr3_log_data;
   reg [40:0] ddr3_log_time; // this gives us up to 8.5 minutes of monotonic logging with ns-resolution
   wire [63:0] ddr3_log_entry;
   reg [63:0]  time_t_clk100_cap;

   ////// grabbing time_t in a synchronized fashion to bclk:
   // time_t_clk100 is frozen on the *falling* edge of we_re, yet
   // ddr3_log_time is captured on the *rising* edge of we, re; thereby avoiding
   // a synchronization problem trying to grab a fast-moving counter value
   always @(posedge bclk) begin
      if( cap_we || cap_re ) begin
	 ddr3_log_time[40:0] <= time_t_clk100_cap[40:0];
      end else begin
	 ddr3_log_time <= ddr3_log_time;
      end
   end

   assign ddr3_log_data[7:0] = log_din[7:0];
   assign ddr3_log_data[8] = log_ale;
   assign ddr3_log_data[9] = log_cle;
   assign ddr3_log_data[10] = log_we;
   assign ddr3_log_data[11] = log_re;
   assign ddr3_log_data[12] = log_cs;
   // note 3 bits are filled in here...cut them out at the final register output stage
   assign ddr3_log_data[22:13] = log_uk[9:0];

   assign ddr3_log_entry[63:0] = {ddr3_log_time, ddr3_log_data};

   /////////
   // now mux log time, data into the DDR3 memory
   /////////
   parameter LOG_DATA      = 4'b1 << 0;
   parameter LOG_TIME      = 4'b1 << 1;
   
   parameter LOG_nSTATES = 4;
   reg [(LOG_nSTATES - 1):0] cstate;
   reg [(LOG_nSTATES - 1):0] nstate;

   always @(posedge bclk or posedge bclk_reset) begin
      if( bclk_reset ) begin
	 cstate <= LOG_DATA;  // async reset
      end else if( log_reset ) begin
	 cstate <= LOG_DATA;  // sync reset
      end else begin
	 cstate <= nstate;
      end
   end

   always @(*) begin
      case (cstate)
	LOG_DATA: begin
	   if( log_capture_pulse ) begin
	      nstate <= LOG_TIME;
	   end else begin
	      nstate <= LOG_DATA;
	   end
	end
	LOG_TIME: begin
	   nstate <= LOG_DATA;
	end
      endcase // case (cstate)
   end

   /// this is a no-op right now, fortunately the synthesis tool doesn't care
   always @(posedge bclk) begin
      if( log_reset ) begin
	 // set things to zero
      end else begin
	 case (cstate)
	   LOG_DATA: begin
	      if( log_capture_pulse ) begin
		 //  hmmm...
	      end
	      // do stuff based upon the current state
	   end
	 endcase // case (cstate)
      end
   end
   
   reg [29:0] log_address;
   reg 	      cmd_delay;
   reg 	      cmd_flush; // used to advance the command queue to flush the DDR3 fifos
   
   assign ddr3_cmd_clk = bclk;
   assign ddr3_wr_data = (cstate == LOG_DATA) ? ddr3_log_entry[31:0] : ddr3_log_entry[63:32];
   assign ddr3_wr_mask = 4'b0; // never mask
   assign ddr3_wr_en = (((cstate == LOG_DATA) && log_capture_pulse) || (cstate == LOG_TIME)) & 
			  log_run;

   assign ddr3_cmd_instr = 3'b000; // hard-wired to a write command
   assign ddr3_cmd_burstlen = 6'h1; // write two words in a burst
   assign ddr3_cmd_en = cmd_delay & log_run | cmd_flush;
   assign ddr3_cmd_addr = log_address;

   reg 	      still_resetting;
   
   always @(posedge bclk) begin
      cmd_delay <= (cstate == LOG_TIME); // issue command enable one cycle after writes are done
      
      if( log_reset ) begin
	 log_address <= 30'h0F00_0000; // start log at high memory, 16 megs from top
	 log_entries <= 27'h0;
      end else if( cmd_delay ) begin
	 if( log_address < 30'h0FFF_FFF8 ) begin
	    log_address <= log_address + 30'h8; // 64 bits written = 8 bytes
	 end else begin
	    log_address <= log_address; // just keep owerwriting the last spot in case of overflow
	 end
	 log_entries <= log_entries + 27'h1; // if entries > 16 MB then we've overflowed
      end else begin
	 log_address <= log_address;
	 log_entries <= log_entries;
      end

      // catch if the command fifo ever overflows
      if( log_reset ) begin
	 log_cmd_error <= 1'b0;
      end else if( ddr3_cmd_full ) begin
	 log_cmd_error <= 1'b1;
      end else begin
	 log_cmd_error <= log_cmd_error;
      end

      // catch if the data fifo ever overflows
      // work around condition where fifo shows full during and slightly after reset
      if( log_reset ) begin
	 log_data_error <= 1'b0;
	 still_resetting <= 1'b1;
      end else if( still_resetting & ddr3_wr_full ) begin
	 still_resetting <= 1'b1;
	 log_data_error <= 1'b0;
      end else if( !still_resetting & ddr3_wr_full ) begin
	 log_data_error <= 1'b1;
	 still_resetting <= 1'b0;
      end else begin
	 log_data_error <= log_data_error;
	 still_resetting <= 1'b0;
      end
      
   end // always @ (posedge bclk)

   /////////
   // time_t generator: synchcronized to clk100
   //   also includes capture of time on falling-edge of we, re to aid synchronization to logs
   //   value is passed up to the register interface to the CPU, but the synchronization
   //   problem has to be solved there, too!
   /////////
   reg [31:0]  time_ns;
   reg [31:0]  time_s;
   reg 	       log_reset_clk100;
   reg 	       log_run_clk100;
   wire        time_we_clk100;
   wire        time_re_clk100;
   reg 	       clk100_wed1, clk100_wed2;
   reg 	       clk100_red1, clk100_red2;

   assign time_we_clk100 = clk100_wed2 & !clk100_wed1; // falling edge pulse gen
   assign time_re_clk100 = clk100_red2 & !clk100_red1;
   
   always @(posedge clk100) begin
      // local clock sync
      log_reset_clk100 <= log_reset;
      log_run_clk100 <= log_run;

      clk100_wed1 <= nand_we;
      clk100_red1 <= nand_re;
      clk100_wed2 <= clk100_wed1;
      clk100_red2 <= clk100_red1;
      
      // time_t_clk100_cap is updated only on the *falling* edge of we, re
      if( time_we_clk100 || time_re_clk100 ) begin
	 time_t_clk100_cap <= {time_s, time_ns};
      end else begin
	 time_t_clk100_cap <= time_t_clk100_cap;
      end
      
      if( log_reset_clk100 ) begin
	 time_ns <= 32'b0;
	 time_s <= 32'b0;
      end else begin
	 if( log_run_clk100 ) begin
	    if( time_ns < 32'd999_999_999 ) begin // count up to only a billion
	       time_ns <= time_ns + 32'd10; // increment by 10ns (100 MHz clock rate)
	       time_s <= time_s;
	    end else begin
	       time_ns <= 32'd0;
	       time_s <= time_s + 32'd1;
	    end
	 end else begin
	    time_ns <= time_ns;
	    time_s <= time_s;
	 end // else: !if( log_run_clk100 )
      end // else: !if( log_reset_clk100 )
   end // always @ (posedge clk100)
   assign time_t_clk100 = {time_s, time_ns};
   
endmodule // nand_log

