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

///////////
// AX211 ROM format
// 0x00000 - 0x1FFFF  copy #1 of firmware
//  0x0000 -  0x7FFF   actual code
//  0x8000 - 0x1FFFF   blank space
//
// 0x20000 - 0x3FFFF  copy #2 of firmware
// 0x40000 - 0x5FFFF  copy #3 of firmware
// 0x60000 - 0x7FFFF  copy #4 of firmware
//
// 0x80000 - 0x80FFF  firmware sig #1
// 0xA0000 - 0xA0FFF  firmware sig #2
// 0xC0000 - 0xC0FFF  firmware sig #3
// 0xE0000 - 0xE0FFF  firmware sig #4
//
// 0x100000 - 0x108000 MBR 
//    0x101100 - 0x101200  MBR region
// 0x140800 - 0x142000 signature "SD         " + some sort of offset table
//    two copies: copy #1 at 140A00, copy #2 at 141200
//
// 0x200000 - 0x400000 each erase block (+0x20000) offset there is an allocation table
//    current theory is this is an implementation-specific version of FAT16
// 
// 0x400000 onwards is disk data
//
// every erase block (0x20000), there is a redundant block header
//   located at offsets 0x200, 0x410, and 0x620. coding is unknown but seems to be block # + ECC

////// layout in RAM
// 0x0000 - 0x7FFF  firmware copy
// 0x8000 - 0x8FFF  signature copy
// 0x9000 - 0xFFFF  MBR + FAT copy
// all other reports as 0xFF

module romulator_ddr3(
		      input wire clk,

		      // nand connection signals
		      input wire nand_we,
		      input wire nand_re,
		      input wire nand_ale,
		      input wire nand_cle,
		      output wire nand_rb,
		      input wire nand_wp,
		      input wire nand_cs,
		      
		      input wire [7:0] nand_din,
		      output wire [7:0] nand_dout,
		      output wire nand_drive_out,

		      // ddr3 memory connection
		      input wire rom_ddr3_reset, // reset just the ddr3-romulator interface
		      
		      output wire ddr3_wr_clk,
`ifdef FULL_PAGE_WRITES
		      output wire ddr3_wr_cmd_en,
`else
		      output reg ddr3_wr_cmd_en,
`endif
		      output wire [2:0] ddr3_wr_cmd_instr,
		      output wire [5:0] ddr3_wr_cmd_bl,
		      output reg [29:0] ddr3_wr_adr,
		      input wire ddr3_wr_cmd_full,
		      input wire ddr3_wr_cmd_empty,
		      output reg ddr3_wr_dat_en,
		      output wire [31:0] ddr3_wr_dat,
		      input wire ddr3_wr_full,
		      input wire ddr3_wr_empty,
		      output reg [3:0] ddr3_wr_mask,
		      
		      output wire ddr3_rd_clk,
		      output reg ddr3_rd_cmd_en,
		      output wire [2:0] ddr3_rd_cmd_instr,
		      output wire [5:0] ddr3_rd_cmd_bl,
		      output wire [29:0] ddr3_rd_adr,
		      input wire  ddr3_rd_cmd_full,
		      output reg ddr3_rd_dat_en,
		      input wire [31:0] ddr3_rd_dat,
		      input wire ddr3_rd_dat_empty,
		      input wire [6:0] ddr3_rd_dat_count,
		      input wire ddr3_rd_dat_full,
		      input wire ddr3_rd_dat_overflow, // need to monitor this

		      output reg page_addra_over,
		      output reg outstanding_under,
		      
		      /// common status signals
		      output wire [7:0] nand_uk_cmd,
		      output reg nand_uk_cmd_updated,
		      
		      output wire [7:0] nand_known_cmd,
		      output reg nand_cmd_updated,
		      
		      output wire [29:0] nand_adr,
		      output wire nand_adr_updated,

		      // debug signals
		      output wire [3:0] ddr_cstate_dbg,

		      input wire reset
		 );

   wire 			 local_reset;
   sync_reset nand_res_sync( .glbl_reset(reset), .clk(clk), .reset(local_reset) );
   wire 			 ddr_reset;
   sync_reset ddr_res_sync( .glbl_reset(rom_ddr3_reset), .clk(clk), .reset(ddr_reset) );

   assign ddr3_wr_clk = clk;
   assign ddr3_rd_clk = clk;

   reg 				 ddr3_wr_dat_en_early;
   reg 				 ddr3_wr_cmd_en_early;
   reg [11:0] 			 col_adr_r;
   reg [17:0] 			 row_adr_r;
   ///// wire to lookup ram
   assign nand_adr = {row_adr_r, col_adr_r}; // note logged address != access address

   // this path assumes embedded ECC in RAM image
   wire ram_blank;
   wire [29:0] adr_with_ecc;

   assign adr_with_ecc[29:0] = (row_adr_r[17:0] * 12'd2112) + col_adr_r;
   // assumes all of memory available
   //   assign ram_blank = row_adr_r[17:0] >= 18'h1F07C; // factors in ECC data stored in RAM
   // assumes top 16MB is reserved for logger function, when enabled
   assign ram_blank = row_adr_r[17:0] >= 18'h1D174;
   
   ////// commands to implement:
   //  00 / 30 read
   //  90      read ID
   //  ff      reset
   //  05 / E0 random data output
   //  70      read status
   //
   //  all other commands should be noted as unprocessable via status register
   //   (keep a small FIFO of unusable commands for debug purposes)
   
   parameter NAND_IDLE     = 13'b1 << 0;
   parameter NAND_ID0      = 13'b1 << 1;
   parameter NAND_READ0    = 13'b1 << 2;
   parameter NAND_READ_GO  = 13'b1 << 3;
   parameter NAND_RESET    = 13'b1 << 4;
   parameter NAND_DOUT0    = 13'b1 << 5;
   parameter NAND_STAT0    = 13'b1 << 6;
   parameter NAND_UNKNOWN0 = 13'b1 << 7;
   parameter NAND_ID1      = 13'b1 << 8;
   parameter NAND_PROG0    = 13'b1 << 9;
   parameter NAND_PROG_GO  = 13'b1 << 10;
   parameter NAND_ERASE0   = 13'b1 << 11;
   parameter NAND_ERASE_GO = 13'b1 << 12;
   
   // don't forget to change bit widths of nSTATES and above '1' constant if you add a state
   parameter NAND_nSTATES = 13;
   reg [(NAND_nSTATES - 1):0] 		 cstate;
   reg [(NAND_nSTATES - 1):0] 		 nstate;

   parameter CMD_ID      = 8'h90;
   parameter CMD_READ    = 8'h00;
   parameter CMD_READ_GO = 8'h30;
   parameter CMD_READ_CP = 8'h35;
   parameter CMD_RESET   = 8'hFF;
   parameter CMD_DATAOUT = 8'h05;
   parameter CMD_DATA_GO = 8'hE0;
   parameter CMD_STATUS  = 8'h70;
   parameter CMD_PROG    = 8'h80;
   parameter CMD_PROG_GO = 8'h10;
   parameter CMD_ERASE   = 8'h60;
   parameter CMD_ERASE_GO= 8'hD0;

   reg 					 readybusy_w; // these signals trigger an async timer
   reg					 readybusy_r;
   reg 					 readybusy_e;
   
   reg [7:0] 				 unknown_command;
   reg [7:0] 				 known_command;
   
   assign nand_uk_cmd = unknown_command;
   assign nand_known_cmd = known_command;
   // capture on falling edge of nand_we when nand_cle active
   always @(posedge clk) begin
      nand_cmd_updated <= !(nand_cle & !nand_we);
   end
   
   always @(posedge nand_we or posedge local_reset) begin
      if(local_reset) begin
	 cstate <= NAND_IDLE;
      end else begin
	 cstate <= nstate;
      end
   end
   
   always @(*) begin
      if(!nand_cs && nand_cle && !nand_ale) begin
	 // CLE cycles always reset nstate
	 if( nand_din == CMD_ID ) begin // done
	    readybusy_r <= 1'b0;
	    readybusy_w <= 1'b0;
	    readybusy_e <= 1'b0;
	    nstate <= NAND_ID0;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_READ ) begin // done
	    readybusy_r <= 1'b0;
	    readybusy_w <= 1'b0;
	    readybusy_e <= 1'b0;
	    nstate <= NAND_READ0;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_READ_GO ) begin // done
	    readybusy_r <= 1'b1;
	    readybusy_w <= 1'b0;
	    readybusy_e <= 1'b0;
	    nstate <= NAND_READ_GO;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_RESET ) begin // done
	    readybusy_r <= 1'b0;
	    readybusy_w <= 1'b0;
	    readybusy_e <= 1'b0;
	    nstate <= NAND_RESET;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_DATAOUT ) begin // done
	    readybusy_r <= 1'b0;
	    readybusy_w <= 1'b0;
	    readybusy_e <= 1'b0;
	    nstate <= NAND_DOUT0;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_DATA_GO ) begin // done
	    readybusy_r <= 1'b1;
	    readybusy_w <= 1'b0;
	    readybusy_e <= 1'b0;
	    nstate <= NAND_READ_GO; // note same state as follows CMD_READ_GO
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_STATUS ) begin // done
	    readybusy_r <= 1'b0;
	    readybusy_w <= 1'b0;
	    readybusy_e <= 1'b0;
	    nstate <= NAND_STAT0;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_PROG ) begin 
	    readybusy_r <= 1'b0;
	    readybusy_w <= 1'b0;
	    readybusy_e <= 1'b0;
	    nstate <= NAND_PROG0;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_PROG_GO ) begin 
	    readybusy_r <= 1'b0;
	    readybusy_w <= 1'b1;
	    readybusy_e <= 1'b0;
	    nstate <= NAND_PROG_GO;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_ERASE ) begin 
	    readybusy_r <= 1'b0;
	    readybusy_w <= 1'b0;
	    readybusy_e <= 1'b0;
	    nstate <= NAND_ERASE0;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_ERASE_GO ) begin 
	    readybusy_r <= 1'b0;
	    readybusy_w <= 1'b0;
	    readybusy_e <= 1'b1;
	    nstate <= NAND_ERASE_GO;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else begin
	    readybusy_r <= 1'b0;
	    readybusy_w <= 1'b0;
	    readybusy_e <= 1'b0;
	    nstate <= NAND_UNKNOWN0; // done
	    unknown_command <= nand_din;
	    known_command <= nand_din;
	 end
      end else begin // if (!nand_cs && nand_cle && !nand_ale)
	 readybusy_r <= 1'b0;
	 readybusy_w <= 1'b0;
	 readybusy_e <= 1'b0;
	 unknown_command <= unknown_command;
	 known_command <= known_command;
	 // if not a CLE cycle, decode based upon current state
	 case (cstate)
	   NAND_IDLE: begin
	      nstate <= NAND_IDLE;
	   end
	   NAND_ID0: begin
	      if( nand_din == 8'h00 ) begin
		 // address cycle for id should be 00
		 nstate <= NAND_ID1;
	      end else begin
		 // if not, ignore the rest
		 nstate <= NAND_IDLE;
	      end
	   end
	   NAND_UNKNOWN0: begin
	      // use this cycle to commit the unknown_command value to a FIFO
	      nstate <= NAND_IDLE;
	   end

	   NAND_READ0: begin
	      // locked in this state until the next cle cycle resets us out of it
	      nstate <= NAND_READ0;
	   end

	   NAND_PROG0: begin
	      // locked in this state until the next cle cycle resets us out of it
	      nstate <= NAND_PROG0;
	   end

	   NAND_ERASE0: begin
	      // locked in this state until the next cle cycle resets us out of it
	      nstate <= NAND_ERASE0;
	   end

	   NAND_DOUT0: begin
	      // locked in this state until the next cle cycle resets us out of it
	      nstate <= NAND_DOUT0;
	   end
	   
	   // most other commannds should return to idle if we saw another we clock edge
	   NAND_RESET: begin
	      nstate <= NAND_IDLE;
	   end
	   NAND_STAT0: begin
	      nstate <= NAND_IDLE;
	   end
	   NAND_READ_GO: begin
	      nstate <= NAND_IDLE;
	   end
	   NAND_PROG_GO: begin
	      nstate <= NAND_IDLE;
	   end
	   NAND_ERASE_GO: begin
	      nstate <= NAND_IDLE;
	   end
	   default: begin
	      nstate <= NAND_IDLE;
	   end
	 endcase // case (cstate)
      end
   end // always @ (*)


   // write-cycle actions
   reg [11:0] wr_cyc;
   reg [11:0] col_adr;
   reg [17:0] row_adr; // bits 29-12 of overall address
   reg [17:0] row_adr_w; // save the row_adr value for use later (in case it's changed during prog)
   reg [17:0] row_adr_e; // save row_adr for erase as well, and separately
   reg page_we;
   reg [7:0] nand_din_cap;
   reg [11:0] 				 pagewr_start; // start of partial program 
   reg [11:0] 				 pagewr_end; // end of partial program

   always @(posedge nand_we) begin
      nand_din_cap[7:0] <= nand_din[7:0];
      if( local_reset ) begin
	 nand_uk_cmd_updated <= 1'b0;
	 wr_cyc <= 12'b0;
	 page_we <= 1'b0;
	 pagewr_start <= 12'b0;
	 pagewr_end <= 12'b0;
	 row_adr_w <= 12'h1c2; // default garbage writes to location 0xE1000, or ECC offset 0xE8080
	 row_adr_e <= 12'h1c3; // default garbage erases as well
      end else begin
	 case (cstate)
	   NAND_IDLE: begin
	      nand_uk_cmd_updated <= 1'b0;
	      wr_cyc <= 12'b0;
	      page_we <= 1'b0;
	      pagewr_start <= pagewr_start;
	      pagewr_end <= pagewr_end;
	      row_adr_w <= row_adr_w;
	      row_adr_e <= row_adr_e;
	   end
	   NAND_UNKNOWN0: begin
	      nand_uk_cmd_updated <= 1'b1;
	      wr_cyc <= 12'b0;
	      page_we <= 1'b0;
	      pagewr_start <= pagewr_start;
	      pagewr_end <= pagewr_end;
	      row_adr_w <= row_adr_w;
	      row_adr_e <= row_adr_e;
	   end
	   NAND_RESET: begin
	      nand_uk_cmd_updated <= 1'b0;
	      wr_cyc <= 12'b0;
	      page_we <= 1'b0;
	      pagewr_start <= 12'b0;
	      pagewr_end <= 12'b0;
	      row_adr_w <= 12'h1c2;
	      row_adr_e <= 12'h1c3;
	   end
	   NAND_READ0: begin
	      row_adr_w <= row_adr_w;
	      row_adr_e <= row_adr_e;
	      nand_uk_cmd_updated <= 1'b0;
	      page_we <= 1'b0;
	      if( wr_cyc == 3'h0 ) begin
		 wr_cyc <= wr_cyc + 3'h1;
		 col_adr[7:0] <= nand_din[7:0];
	      end else if( wr_cyc == 3'h1 ) begin
		 wr_cyc <= wr_cyc + 3'h1;
		 col_adr[11:8] <= nand_din[3:0];
	      end else if( wr_cyc == 3'h2) begin
		 wr_cyc <= wr_cyc + 3'h1;
		 row_adr[7:0] <= nand_din[7:0];
	      end else if( wr_cyc == 3'h3 ) begin
		 wr_cyc <= wr_cyc + 3'h1;
		 row_adr[15:8] <= nand_din[7:0];
	      end else if( wr_cyc == 3'h4 ) begin
		 wr_cyc <= wr_cyc + 3'h1;
		 row_adr[17:16] <= nand_din[1:0];
	      end else begin
		 wr_cyc <= wr_cyc;
	      end
	      pagewr_start <= pagewr_start;
	      pagewr_end <= pagewr_end;
	   end // case: NAND_READ0
	   NAND_DOUT0: begin
	      row_adr_w <= row_adr_w;
	      row_adr_e <= row_adr_e;
	      nand_uk_cmd_updated <= 1'b0;
	      page_we <= 1'b0;
	      if( wr_cyc == 3'h0 ) begin
		 wr_cyc <= wr_cyc + 3'h1;
		 col_adr[7:0] <= nand_din[7:0];
	      end else if( wr_cyc == 3'h1 ) begin
		 wr_cyc <= wr_cyc + 3'h1;
		 col_adr[11:8] <= nand_din[3:0];
	      end else begin
		 wr_cyc <= wr_cyc;
	      end
	      pagewr_start <= pagewr_start;
	      pagewr_end <= pagewr_end;
	   end // case: NAND_DOUT0
	   NAND_READ_GO: begin
	      row_adr_w <= row_adr_w;
	      row_adr_e <= row_adr_e;
	      nand_uk_cmd_updated <= 1'b0;
	      wr_cyc <= 12'b0;
	      page_we <= 1'b0;
	      pagewr_start <= pagewr_start;
	      pagewr_end <= pagewr_end;
	   end
	   NAND_PROG0: begin
	      row_adr_e <= row_adr_e;
	      nand_uk_cmd_updated <= 1'b0;
	      if( wr_cyc == 12'h0 ) begin
		 row_adr_w <= row_adr_w;
		 page_we <= 1'b0;
		 wr_cyc <= wr_cyc + 12'h1;
		 col_adr[7:0] <= nand_din[7:0];
		 row_adr <= row_adr;
		 pagewr_start <= pagewr_start;
		 pagewr_end <= pagewr_end;
	      end else if( wr_cyc == 12'h1 ) begin
		 row_adr_w <= row_adr_w;
		 page_we <= 1'b0;
		 wr_cyc <= wr_cyc + 12'h1;
		 col_adr[11:8] <= nand_din[3:0];
		 row_adr <= row_adr;
		 pagewr_start <= pagewr_start;
		 pagewr_end <= pagewr_end;
	      end else if( wr_cyc == 12'h2) begin
		 row_adr_w <= row_adr_w;
		 page_we <= 1'b0;
		 wr_cyc <= wr_cyc + 12'h1;
		 row_adr[7:0] <= nand_din[7:0];
		 col_adr[11:0] <= col_adr[11:0] - 12'h1; // back it up one because we +1 with page_we
		 pagewr_start[11:0] <= col_adr[11:0] - 12'h1;
		 pagewr_end[11:0] <= col_adr[11:0] - 12'h1;
	      end else if( wr_cyc == 12'h3 ) begin
		 row_adr_w <= row_adr_w;
		 page_we <= 1'b0;
		 wr_cyc <= wr_cyc + 12'h1;
		 row_adr[15:8] <= nand_din[7:0];
		 col_adr <= col_adr;
		 pagewr_start <= pagewr_start;
		 pagewr_end <= pagewr_end;
	      end else if( wr_cyc == 12'h4 ) begin
		 row_adr_w <= row_adr_w;
		 page_we <= 1'b0;
		 wr_cyc <= wr_cyc + 12'h1;
		 row_adr[17:16] <= nand_din[1:0];
		 col_adr <= col_adr;
		 pagewr_start <= pagewr_start;
		 pagewr_end <= pagewr_end;
	      end else if( wr_cyc < 12'd2117 ) begin
		 row_adr_w <= row_adr; // commit the row address here
		 pagewr_start <= pagewr_start;
		 row_adr <= row_adr;
		 wr_cyc <= wr_cyc + 12'h1;
		 // write data to local page buffer
		 if( ((col_adr[11:0] < 12'd2112) || (col_adr[11:0] == 12'hFFF)) & !nand_cle ) begin
		    pagewr_end <= pagewr_end + 12'h1;
		    page_we <= 1'b1;
		    col_adr[11:0] <= col_adr[11:0] + 12'h1;
		 end else begin
		    pagewr_end <= pagewr_end;
		    page_we <= 1'b0;
		    col_adr[11:0] <= col_adr[11:0];
		 end
	      end else begin
		 row_adr_w <= row_adr_w;
		 pagewr_start <= pagewr_start;
		 pagewr_end <= pagewr_end;
		 row_adr <= row_adr;
                   		 page_we <= 1'b0;
		 wr_cyc <= wr_cyc;
		 col_adr[11:0] <= col_adr[11:0];
	      end
	   end
	   NAND_PROG_GO: begin
	      row_adr_w <= row_adr_w;
	      row_adr_e <= row_adr_e;
	      pagewr_start <= pagewr_start;
	      pagewr_end <= pagewr_end - 12'h1; 
	      page_we <= 1'b0;
	      nand_uk_cmd_updated <= 1'b0;
	      wr_cyc <= 12'b0;
	   end

	   NAND_ERASE0: begin
	      page_we <= 1'b0;
	      pagewr_start <= pagewr_start;
	      pagewr_end <= pagewr_end;
	      col_adr <= col_adr;
	      row_adr_w <= row_adr_w;
	      nand_uk_cmd_updated <= 1'b0;
	      if( wr_cyc == 12'h0) begin
		 row_adr_e <= row_adr_e;
		 wr_cyc <= wr_cyc + 12'h1;
		 row_adr[7:0] <= {nand_din[7], 7'b0};
	      end else if( wr_cyc == 12'h1 ) begin
		 row_adr_e <= row_adr_e;
		 wr_cyc <= wr_cyc + 12'h1;
		 row_adr[15:8] <= nand_din[7:0];
	      end else if( wr_cyc == 12'h2 ) begin
		 wr_cyc <= wr_cyc + 12'h1;
		 row_adr[17:16] <= nand_din[1:0];
		 row_adr_e[17:0] <= {nand_din[1:0],row_adr[15:0]};
	      end else begin
		 row_adr_e <= row_adr_e;
		 row_adr <= row_adr;
		 wr_cyc <= wr_cyc;
	      end
	   end
	   NAND_ERASE_GO: begin
	      row_adr_w <= row_adr_w;
	      row_adr_e <= row_adr_e;
	      pagewr_start <= pagewr_start;
	      pagewr_end <= pagewr_end; 
	      page_we <= 1'b0;
	      nand_uk_cmd_updated <= 1'b0;
	      wr_cyc <= 12'b0;
	   end

	   default: begin
	      // NAND_ID1 is a nop
	      // NAND_STAT0 is a nop
	      row_adr_e <= row_adr_e;
	      row_adr_w <= row_adr_w;
	      pagewr_start <= pagewr_start;
	      pagewr_end <= pagewr_end;
	      page_we <= 1'b0;
	      nand_uk_cmd_updated <= 1'b0;
	      wr_cyc <= 12'b0;
	   end
	 endcase // case (cstate)
      end
   end

   reg [3:0] rd_cycle;
   reg [7:0] special_data;
   reg 	     special_en;
   reg 	     data_en;
   
   // read-cycle actions
   
   // reflect the clock back into the system so we can route it safely
   wire       nand_re_sig;
   wire       nand_re_to_obuf;
   
//   ODDR2 nand_re_buf (.D0(1'b1), .D1(1'b0), .C0(nand_re), .C1(!nand_re), .Q(nand_re_to_obuf),
//		      .CE(1), .R(0), .S(0) );
//   OBUF  nand_re_obuf( .I(nand_re_to_obuf), .O(nand_re_dummy) );
//   IBUF  nand_re_loopback( .I(nand_re_dummy), .O(nand_re_sig) );
   
//   assign nand_drive_out = !nand_re_sig && !nand_cs;
   wire [7:0] ram_d_from_ram;
   
   assign nand_drive_out = !nand_re && !nand_cs;
   assign nand_dout = special_en ? special_data[7:0] : (ram_blank ? 8'hff : ram_d_from_ram[7:0]);

   reg 	      first_read; // delay incremeting address after the first read falling edge
   assign nand_adr_updated = !first_read; // latch address on rising edge of adr_updated
      always @(negedge nand_re or posedge nand_cle) begin
      if( nand_cle ) begin
	 rd_cycle <= 4'b0;
	 special_data <= 8'b0;
	 special_en <= 1'b0;
	 first_read <= 1'b1;
	 // asynchronously copy over col/row adr when cle is set....
	 col_adr_r <= col_adr;
	 row_adr_r <= row_adr;
      end else begin
	 row_adr_r <= row_adr_r;
	 case (cstate)
	   NAND_ID1: begin
	      first_read <= first_read;
	      col_adr_r <= col_adr_r;
	      if( rd_cycle == 4'h0 ) begin
		 special_data <= 8'hEC;
		 rd_cycle <= rd_cycle + 4'h1;
		 special_en <= 1'b1;
	      end else if( rd_cycle == 4'h1 ) begin
		 special_data <= 8'hDC;
		 rd_cycle <= rd_cycle + 4'h1;
		 special_en <= 1'b1;
	      end else if( rd_cycle == 4'h2 ) begin
		 special_data <= 8'h10;
		 rd_cycle <= rd_cycle + 4'h1;
		 special_en <= 1'b1;
	      end else if( rd_cycle == 4'h3 ) begin
		 special_data <= 8'h95;
		 rd_cycle <= rd_cycle + 4'h1;
		 special_en <= 1'b1;
	      end else if( rd_cycle == 4'h4 ) begin
		 special_data <= 8'h54;
		 rd_cycle <= rd_cycle + 4'h1;
		 special_en <= 1'b1;
	      end else begin
		 special_data <= 8'hFF;
		 rd_cycle <= rd_cycle; // stop the counter here
		 special_en <= 1'b0;
	      end
	   end // case: NAND_ID1
	   NAND_STAT0: begin
	      first_read <= first_read;
	      col_adr_r <= col_adr_r;
	      special_en <= 1'b1;
	      special_data <= 8'b01000000; // not protected, ready, and always pass
	   end
	   NAND_READ_GO: begin
	      if( first_read ) begin
		 first_read <= 1'b0;
		 col_adr_r <= col_adr_r;
	      end else if( col_adr_r < 12'd2112 ) begin
		 col_adr_r <= col_adr_r + 12'b1; // increment the column address
	      end else begin 
		 col_adr_r <= 12'd0;  // wrap column address around to 0 if over-read
	      end
	   end
	   default: begin
	      first_read <= 1'b1;
	      col_adr_r <= col_adr_r;
	      rd_cycle <= 4'b0;
	      special_data <= 8'hFF;
	      special_en <= 1'b0;
	   end
	 endcase // case (cstate)
      end // else: !if( nand_cle )
   end

   (* ASYNC_REG="TRUE" *) reg rb_w1, rb_r1, rb_e1;
   reg rb_w, rb_w_pulse;
   reg rb_r, rb_r_pulse;
   reg rb_e, rb_e_pulse;
   reg [12:0] rb_timer;
   reg 	     rb_counting;
   
   /////// generate an R/B count based on asynchronous pulse input
   always @(posedge clk) begin
      rb_w1 <= readybusy_w;
      rb_w <= rb_w1;

      rb_r1 <= readybusy_r;
      rb_r <= rb_r1;

      rb_e1 <= readybusy_e;
      rb_e <= rb_e1;

      if( !rb_w && rb_w1 ) begin
	 rb_w_pulse <= 1'b1;
      end else begin
	 rb_w_pulse <= 1'b0;
      end

      if( !rb_r && rb_r1 ) begin
	 rb_r_pulse <= 1'b1;
      end else begin
	 rb_r_pulse <= 1'b0;
      end

      if( !rb_e && rb_e1 ) begin
	 rb_e_pulse <= 1'b1;
      end else begin
	 rb_e_pulse <= 1'b0;
      end

      if( local_reset ) begin
	 rb_timer <= 12'b0;
	 rb_counting <= 1'b0;
      end else begin
	 if( (rb_r_pulse || rb_w_pulse) && (rb_timer == 12'b0) ) begin
	    rb_timer <= rb_timer + 12'b1;
	    rb_counting <= 1'b1;
	 end else if( rb_timer == 12'h200 ) begin  // about 1.5us @ 133 MHz clock
	    rb_timer <= 12'h0;
	    rb_counting <= 1'b0;
	 end else if( rb_counting == 1'b1 ) begin
	    rb_timer <= rb_timer + 12'b1;
	    rb_counting <= 1'b1;
	 end else begin
	    rb_timer <= 12'h0;
	    rb_counting <= 1'b0;
	 end
      end // else: !if( local_reset or nand_cle )
   end
//   assign nand_rb = !rb_counting || (rb_timer < 12'h4); // this should null out the above logic
   
   parameter DDR_IDLE     = 4'b1 << 0;
   parameter DDR_FETCH    = 4'b1 << 1;
   parameter DDR_WRITE    = 4'b1 << 2;
   parameter DDR_ERASE    = 4'b1 << 3;

   parameter DDR_nSTATES = 4;
   reg [(DDR_nSTATES - 1):0] 		 ddr_cstate;
   reg [(DDR_nSTATES - 1):0] 		 ddr_nstate;
   reg [(DDR_nSTATES - 1):0] 		 ddr_ostate;

   reg [11:0] 				 page_addra;
   reg [11:0] 				 requested; // total words requested
   reg [6:0] 				 outstanding; // read words currently outstanding
   reg [4:0] 				 wr_bytes;
   reg [11:0] 				 write_ptr;
   reg [19:0] 				 erase_ptr;

   assign ddr_cstate_dbg = ddr_cstate;
   
   assign nand_rb = !(ddr_cstate != DDR_IDLE);

   always @(posedge clk) begin
      ddr_ostate <= ddr_cstate; // "old state" for pipeline balancing computations
      
      if(ddr_reset) begin
	 ddr_cstate <= DDR_IDLE;
      end else begin
	 ddr_cstate <= ddr_nstate;
      end
   end

   always @(*) begin
      case(ddr_cstate)
	DDR_IDLE: begin
	   if( rb_r_pulse ) begin
	      ddr_nstate <= DDR_FETCH;
	   end else if( rb_w_pulse ) begin
	      ddr_nstate <= DDR_WRITE;
	   end else if( rb_e_pulse ) begin
	      ddr_nstate <= DDR_ERASE;
	   end else begin
	      ddr_nstate <= DDR_IDLE;
	   end
	end
	DDR_FETCH: begin
	   if( ((requested >= 12'd511) && (requested < 12'hFF0)) 
	       && (outstanding == 7'd0) ) begin
	      ddr_nstate <= DDR_IDLE;
	   end else begin
	      ddr_nstate <= DDR_FETCH;
	   end
	end
	DDR_WRITE: begin
`ifdef FULL_PAGE_WRITES
	   /// need a termination signal for DDR_WRITE
	   if( (outstanding >= 7'd33) && ddr3_wr_empty ) begin
	      ddr_nstate <= DDR_IDLE; 
	   end else begin
	      ddr_nstate <= DDR_WRITE;
	   end
`else
	   /// need a termination signal for DDR_WRITE
	   if( write_ptr[11:0] == pagewr_end[11:0] ) begin
	      ddr_nstate <= DDR_IDLE; 
	   end else begin
	      ddr_nstate <= DDR_WRITE;
	   end
`endif // !`ifdef FULL_PAGE_WRITES
	end // case: DDR_WRITE
	DDR_ERASE: begin
	   if( (erase_ptr[19:0] >= 20'h107FF) && (erase_ptr[19:0] != 20'hFFFFF) ) begin
	      ddr_nstate <= DDR_IDLE; 
	   end else begin
	      ddr_nstate <= DDR_ERASE;
	   end
	end
      endcase // case (ddr_cstate)
   end // always @ (*)

   // translate row address + column position of request (request is by word = 4 bytes)
   assign ddr3_rd_adr[29:0] = (row_adr_r[17:0] * 30'd2112) + (requested[11:0] * 30'd4);
   assign ddr3_rd_cmd_instr = 3'b001;
   assign ddr3_rd_cmd_bl = 6'd15; // write 16 words at a time (actual burst length is bl+1)

`ifdef FULL_PAGE_WRITES
   assign ddr3_wr_cmd_en = ddr3_wr_cmd_en_early;

   reg [17:0] row_adr_e_pipe; // pipe the row_adr results; row_adr doesn't change during operations
   reg [17:0] row_adr_w_pipe; // eases the timing constraints
   
   always @(posedge clk) begin
      row_adr_e_pipe <= row_adr_e[17:0] * 30'd2112; // pipeline the multiplier
      row_adr_w_pipe <= row_adr_w[17:0] * 30'd2112;
      
      // introduce a one-stage delay so wr_adr matches the data coming out of the
      // page read RAM
      // the enable signal to the memory interface is likewise delayed in the state machine
      ddr3_wr_adr[29:0] <= (ddr_cstate == DDR_ERASE) ? 
			   (row_adr_e_pipe) +
			   (erase_ptr[19:0] * 12'd4)
			   :
			   (row_adr_w_pipe) + 
			   (outstanding[6:0] * 12'd16 + (wr_bytes[4:0] - 5'b1)) * 30'd4 ;
      ddr3_wr_dat_en <= ddr3_wr_dat_en_early;
   end
`else
   reg [17:0] row_adr_e_pipe; // pipe the row_adr results; row_adr doesn't change during operations
   reg [17:0] row_adr_w_pipe; // eases the timing constraints
   
   always @(posedge clk) begin
      row_adr_e_pipe <= row_adr_e[17:0] * 30'd2112; // pipeline the multiplier
      row_adr_w_pipe <= row_adr_w[17:0] * 30'd2112;

      ddr3_wr_dat_en <= ddr3_wr_dat_en_early;
      ddr3_wr_cmd_en <= ddr3_wr_cmd_en_early;
      
      ddr3_wr_adr[29:0] <= (ddr_cstate == DDR_ERASE) ?
			   (row_adr_e_pipe[17:0]) +
			   (erase_ptr[19:0] * 12'd4)
			   :
			   (row_adr_w_pipe[17:0]) + 
			   {write_ptr[11:2], 2'b0};
      if( ddr_cstate == DDR_ERASE ) begin
	 ddr3_wr_mask[3:0] <= 4'b0;
      end else begin
	 case(write_ptr[1:0])
	   2'b00: begin
	      ddr3_wr_mask[3:0] <= 4'b1110;
	   end
	   2'b01: begin
	      ddr3_wr_mask[3:0] <= 4'b1101;
	   end
	   2'b10: begin
	      ddr3_wr_mask[3:0] <= 4'b1011;
	   end
	   2'b11: begin
	      ddr3_wr_mask[3:0] <= 4'b0111;
	   end
	 endcase // case (write_ptr[1:0])
      end // else: !if( ddr_cstate == DDR_ERASE )
   end
`endif
   
   // col_adr has no place because writes are always whole-page
   
   assign ddr3_wr_cmd_instr = 3'b000;
`ifdef FULL_PAGE_WRITES
   assign ddr3_wr_cmd_bl = 6'd15; // write 16 words at a time (actual burst length is bl+1)
`else
   // write 1 word at a time (actual burst length is bl+1) for writes
   // 1 word at a time for erases as well (for simplicity)
   assign ddr3_wr_cmd_bl = (ddr_cstate == DDR_ERASE) ? 6'd0 : 6'd0;
`endif
   
   always @(posedge clk) begin
      case(ddr_cstate)
	DDR_IDLE: begin
	   ddr3_wr_dat_en_early <= 1'b0;
	   wr_bytes <= 5'h0;
	   page_addra <= 12'hFFF;
	   requested <= 12'hFF0; // init to -1 to handle fencepost error
	   outstanding <= 7'b0;
	   if( !ddr3_rd_dat_empty ) begin
	      ddr3_rd_dat_en <= 1'b1; // drain the fifo during idle time, just in case
	   end else begin
	      ddr3_rd_dat_en <= 1'b0;
	   end
	   page_addra_over <= 1'b0;
	   outstanding_under <= 1'b0;
	   ddr3_wr_cmd_en_early <= 1'b0;
	   write_ptr <= pagewr_start;
	   erase_ptr <= 20'hFFFFF; // start at -1
	end
	DDR_FETCH: begin
	   erase_ptr <= 20'hFFFFF;
	   write_ptr <= pagewr_start;
	   ddr3_wr_dat_en_early <= 1'b0;
	   ddr3_wr_cmd_en_early <= 1'b0;
	   wr_bytes <= 5'h0;
	   // note requested starts at -1
	   if( ((requested < 12'd511) || (requested >= 12'hFF0)) && !ddr3_rd_cmd_full && 
	       (outstanding < 7'd47) && ddr3_rd_dat_empty) begin
	      ddr3_rd_cmd_en <= 1'b1;
	      requested <= requested + 12'd16; // fetching 16 words at a time
	      outstanding <= outstanding + 7'd16;
	      ddr3_rd_dat_en <= 1'b0;
	      page_addra <= page_addra;
	      page_addra_over <= 1'b0;
	      outstanding_under <= 1'b0;
	   end else if (((requested < 12'd511) || (requested >= 12'hFF0)) && 
			!ddr3_rd_cmd_full &&
			(outstanding < 7'd47) && !ddr3_rd_dat_empty) begin
	      ddr3_rd_cmd_en <= 1'b1;
	      requested <= requested + 12'd16; 
	      outstanding <= outstanding + 7'd15; // one less outstanding due to simultaneous read
	      ddr3_rd_dat_en <= 1'b1;
	      if( (page_addra < 12'd2112) || (page_addra == 12'hFFF) ) begin
		 page_addra <= page_addra + 12'b1;
		 page_addra_over <= 1'b0;
	      end else begin
		 page_addra <= page_addra;
		 page_addra_over <= 1'b1;
	      end
	      outstanding_under <= 1'b0;
	   end else if( !ddr3_rd_dat_empty ) begin
	      ddr3_rd_cmd_en <= 1'b0;
	      requested <= requested;
	      if( outstanding > 7'b0 ) begin
		 outstanding <= outstanding - 7'b1;
		 outstanding_under <= 1'b0;
	      end else begin
		 outstanding <= 12'b0; // we shouldn't reach this but just in case...
		 outstanding_under <= 1'b1;
	      end
	      ddr3_rd_dat_en <= 1'b1;
	      if( (page_addra < 12'd2112) || (page_addra == 12'hFFF) ) begin
		 page_addra <= page_addra + 12'b1;
		 page_addra_over <= 1'b0;
	      end else begin
		 page_addra <= page_addra;
		 page_addra_over <= 1'b1;
	      end
	   end else begin
	      ddr3_rd_cmd_en <= 1'b0;
	      requested <= requested;
	      outstanding <= outstanding;
	      ddr3_rd_dat_en <= 1'b0;
	      page_addra <= page_addra;
	      page_addra_over <= 1'b0;
	      outstanding_under <= 1'b0;
	   end
	end
	DDR_WRITE: begin
`ifdef FULL_PAGE_WRITES
	   // I believe a page write *always* commits the full 2112-byte page to memory
	   // not just the changed addresses
	   // the data fifo can hold 64 4-byte words
	   // writing 16 words (64 bytes) at a time, there are 33 even write cycles

	   // use the 'outstanding' register to keep track of packets written
	   if( outstanding < 7'd33 ) begin
	      if( ((wr_bytes == 5'h0) && !ddr3_wr_empty) || ddr3_wr_cmd_full ) begin
		 // this is a wait state, we wait until we're empty
		 ddr3_wr_dat_en_early <= 1'b0;
		 outstanding <= outstanding;
		 wr_bytes <= wr_bytes;
		 ddr3_wr_cmd_en_early <= 1'b0;
	      end else if( (wr_bytes == 5'h0) && ddr3_wr_empty ) begin
		 // once we're empty, we'll start filling the fifo again
		 ddr3_wr_dat_en_early <= 1'b1; 
		 wr_bytes <= wr_bytes + 5'h1;
		 outstanding <= outstanding;
		 ddr3_wr_cmd_en_early <= 1'b0;
	      end else if(wr_bytes < 5'd16) begin
		 // fill to the end, don't care if we se wr_empty
		 ddr3_wr_dat_en_early <= 1'b1; 
		 wr_bytes <= wr_bytes + 5'h1;
		 outstanding <= outstanding;
		 ddr3_wr_cmd_en_early <= 1'b0;
	      end else begin
		 // now issue the command
		 wr_bytes <= 5'h0;
		 ddr3_wr_dat_en_early <= 1'b0; 
		 outstanding <= outstanding + 7'b1;
		 // this will always succeed because we waited for queue to be empty 
		 // before starting the cycle
		 ddr3_wr_cmd_en_early <= 1'b1; 
	      end // else: !if(wr_bytes < 5'd16)
	   end else begin // if ( outstanding < 7'd33 )
	      // do nothing here?
	      ddr3_wr_dat_en_early <= 1'b0;
	      outstanding <= outstanding;
	      wr_bytes <= wr_bytes;
	      ddr3_wr_cmd_en_early <= 1'b0;
	   end // else: !if( outstanding < 7'd33 )
`else // !`ifdef FULL_PAGE_WRITES
	   outstanding <= outstanding; // ignored in this code implementation
	   
	   // rework this such that pagewr_start / pagewr_end bound the write command
	   if( ((write_ptr < pagewr_end) || (write_ptr == 12'hFFF)) && ddr3_wr_cmd_empty ) begin
	      ddr3_wr_dat_en_early <= 1'b1;
	      ddr3_wr_cmd_en_early <= 1'b1;
	      write_ptr <= write_ptr + 12'b1;
	   end else begin
	      ddr3_wr_cmd_en_early <= 1'b0;
	      ddr3_wr_dat_en_early <= 1'b0;
	      write_ptr <= write_ptr;
	   end
`endif // !`ifdef FULL_PAGE_WRITES
	   page_addra <= 12'hFFF;
	   requested <= 12'b0;
	   ddr3_rd_dat_en <= 1'b0;
	   page_addra_over <= 1'b0;
	   outstanding_under <= 1'b0;
	   erase_ptr <= 20'hFFFFF;
	end // case: DDR_WRITE
	DDR_ERASE: begin
	   // be a little lazy about the implementation: just issue an erase
	   // once the command fifo is empty. It means we lose some efficiency
	   // but it's easier to debug (about 50% efficiency loss due to
	   // burst length of 4 bytes)
	   if( ddr3_wr_cmd_empty && 
	       ((erase_ptr < 20'h107FF) || (erase_ptr == 20'hFFFFF)) ) begin
	      erase_ptr <= erase_ptr + 20'd1;
	      ddr3_wr_dat_en_early <= 1'b1;
	      ddr3_wr_cmd_en_early <= 1'b1;
	   end else begin
	      ddr3_wr_cmd_en_early <= 1'b0;
	      ddr3_wr_dat_en_early <= 1'b0;
	      erase_ptr <= erase_ptr;
	   end
	   // unused variables in this case
	   outstanding <= outstanding;
	   wr_bytes <= 5'h0;
	   page_addra <= 12'hFFF;
	   requested <= 12'hFF0;
	   ddr3_rd_dat_en <= 1'b0;
	   page_addra_over <= 1'b0;
	   outstanding_under <= 1'b0;
	   write_ptr <= pagewr_start;
	end // case: DDR_ERASE
      endcase // case (ddr_cstate)
   end

   reg page_we_d;
   reg page_we_d2;
   reg page_we_pulse;
   always @(posedge clk) begin
      page_we_d <= page_we & nand_we;
      page_we_d2 <= page_we_d;
      
      page_we_pulse <= !page_we_d2 & page_we_d;
   end
   
   // 2112-byte page buffer
   wire [31:0] wr_dat_from_pagebuf;
   assign ddr3_wr_dat[31:0] = ( ddr_ostate == DDR_ERASE ) ?
			      32'hFFFF_FFFF
			      :
			      wr_dat_from_pagebuf[31:0];
   
   page_buf2112 pagebuf(
			// a-port faces the DDR interface (31-bit)
			.clka(clk),
			.ena(1'b1),
			.wea(ddr3_rd_dat_en && (ddr_cstate == DDR_FETCH)),
`ifdef FULL_PAGE_WRITEs
			.addra(( ddr_cstate == DDR_FETCH) ? page_addra[9:0] : 
			       outstanding[6:0] * 10'd16 + (wr_bytes[4:0] - 5'b1) ),
`else
			.addra(( ddr_cstate == DDR_FETCH) ? page_addra[9:0] : 
			       write_ptr[11:2] ),
`endif
			.dina(ddr3_rd_dat[31:0]),
			.douta(wr_dat_from_pagebuf[31:0]),

			// b-port faces the NAND interface (8-bit)
			.clkb(clk),
			.enb(1'b1),
			// hack: ddr_cstate gate gets rid of trailing pulse during terminating CLE cycle
			.web(page_we_pulse && (ddr_cstate == DDR_IDLE)),
			.addrb(page_we ? col_adr[11:0] : col_adr_r[11:0]),
			.dinb(nand_din_cap[7:0]),
			.doutb(ram_d_from_ram[7:0])
			);
      
endmodule // romulator_ddr3


