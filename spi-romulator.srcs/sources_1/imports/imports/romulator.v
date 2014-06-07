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

module romulator(
		 input wire clk,

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

		 output wire [15:0] ram_adr,
		 output wire [7:0] ram_d_to_ram,
		 input wire [7:0] ram_d_from_ram,
		 output wire ram_we,
		 output wire ram_clk_to_ram,

		 output wire [7:0] nand_uk_cmd,    // pipe to a FIFO to store unknown commands
		 output reg nand_uk_cmd_updated,

		 output wire [7:0] nand_known_cmd,
		 output wire nand_cmd_updated,

		 output wire [29:0] nand_adr,
		 output wire nand_adr_updated,

		 input wire reset

		 );

   wire 	       local_reset;
   sync_reset nand_res_sync( .glbl_reset(reset), .clk(clk), .reset(local_reset) );

   reg [11:0] col_adr_r;
   reg [17:0] row_adr_r;
   ///// wire to lookup ram
   assign ram_clk_to_ram = clk;
   assign ram_d_to_ram = 8'b0; // doesn't matter for now, write is disabled
   assign ram_we = 1'b0;
   assign nand_adr = {row_adr_r, col_adr_r};

`ifdef NOECC_IN_ROM
   reg 	      ram_blank;
   reg 	      ram_adrr;  // registered version of ram_adr
   
   always @(row_adr_r or col_adr_r) begin
      if( col_adr_r[11] == 1'b1 ) begin  // OOB data is blank
	 ram_blank = 1'b1;
	 ram_adrr[15:0] = {1'b0, row_adr_r[2:0],col_adr_r[11:0]}; // actually a don't care
      end else begin
	 if( {row_adr_r,col_adr_r[10:0]} < 28'h80000 ) begin
	    ram_adrr[15:0] = {1'b0, row_adr_r[3:0],col_adr_r[10:0]};
	    ram_blank = row_adr_r[4];
	 end else if( ({row_adr_r,col_adr_r[10:0]} >= 28'h80000) && 
		      ({row_adr_r,col_adr_r[10:0]} < 28'h100000) ) begin
	    ram_adrr[15:0] = {4'b1000,row_adr_r[0],col_adr_r[10:0]};
	    // 81000-9ffff
	    // a1000-bffff
	    // c1000-dffff
	    // e1000-fffff

	    // 81-9f -- 80
	    // a1-bf -- a0
	    // c1-df -- c0
	    // e1-ff -- e0
	    ram_blank = !((row_adr_r[8:1] == 8'h80) || (row_adr_r[8:1] == 8'ha0) ||
			(row_adr_r[8:1] == 8'hc0) || (row_adr_r[8:1] == 8'he0));
	 end else if( ({row_adr_r,col_adr_r[10:0]} >= 28'h100000) && 
		      ({row_adr_r,col_adr_r[10:0]} <  28'h107000) ) begin
	    ram_adrr[15:0] = {4'b1001,row_adr_r[0],col_adr_r[10:0]};
	    ram_blank = row_adr_r[8:5] > 4'h7;
	 end else begin
	    ram_blank = 1'b1;
	    ram_adrr[15:0] = {1'b0, row_adr_r[2:0],col_adr_r[11:0]}; // actually a don't care
	 end
      end // else: !if( col_adr_r[11] == 1'b1 )
   end // always @ (row_adr_r or col_adr_r)

   assign ram_adr = ram_adrr;
`else // !`ifdef NOECC_IN_ROM
   // this path assumes embedded ECC in RAM image
   reg ram_blank;
   reg [15:0] ram_adrr;
   wire [15:0] adr_with_ecc;

   assign adr_with_ecc[15:0] = (row_adr_r[4:0] * 12'd2112) + col_adr_r;
   
   always @(row_adr_r or col_adr_r) begin
      if( {row_adr_r,col_adr_r[10:0]} < 28'h80000 ) begin
	 ram_adrr[15:0] = adr_with_ecc[15:0];
	 ram_blank = 1'b0;
	 // 'b' rev binary puts the firmware signature at 0x8000 in binary
//      end else if( ({row_adr_r,col_adr_r[10:0]} >= 28'h80000) &&
//		   ({row_adr_r,col_adr_r[10:0]} < 28'h100000) ) begin
	 // 'c' rev binary puts the MBR region at 0x8000 in binary
//      end else if( ({row_adr_r,col_adr_r[10:0]} >= 28'h100000) &&
//		   ({row_adr_r,col_adr_r[10:0]} < 28'h200000) ) begin
//	 ram_adrr[15:0] = {1'b1,adr_with_ecc[14:0]};
//	 ram_blank = 1'b0;
      end else begin
	 ram_adrr[15:0] = {1'b1,adr_with_ecc[14:0]};
	 ram_blank = 1'b1;
      end
   end

   assign ram_adr = ram_adrr;
`endif
   
   
   ////// commands to implement:
   //  00 / 30 read
   //  90      read ID
   //  ff      reset
   //  05 / E0 random data output
   //  70      read status
   //
   //  all other commands should be noted as unprocessable via status register
   //   (keep a small FIFO of unusable commands for debug purposes)
   
   parameter NAND_IDLE     = 10'b1 << 0;
   parameter NAND_ID0      = 10'b1 << 1;
   parameter NAND_READ0    = 10'b1 << 2;
   parameter NAND_READ_GO  = 10'b1 << 3;
   parameter NAND_RESET    = 10'b1 << 4;
   parameter NAND_DOUT0    = 10'b1 << 5;
   parameter NAND_STAT0    = 10'b1 << 6;
   parameter NAND_UNKNOWN0 = 10'b1 << 7;
   parameter NAND_ID1      = 10'b1 << 8;
   
   // don't forget to change bit widths of nSTATES and above '1' constant if you add a state

   parameter NAND_nSTATES = 10;
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

   reg 					 readybusy_w; // these signals trigger an async timer
   reg					 readybusy_r;
   
   reg [7:0] 				 unknown_command;
   reg [7:0] 				 known_command;
   
   assign nand_uk_cmd = unknown_command;
   assign nand_known_cmd = known_command;
   // capture on falling edge of nand_we when nand_cle active
   assign nand_cmd_updated = !(nand_cle & !nand_we); 

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
	    nstate <= NAND_ID0;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_READ ) begin // done
	    readybusy_r <= 1'b0;
	    nstate <= NAND_READ0;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_READ_GO ) begin // done
	    readybusy_r <= 1'b1;
	    nstate <= NAND_READ_GO;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_RESET ) begin // done
	    readybusy_r <= 1'b0;
	    nstate <= NAND_RESET;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_DATAOUT ) begin // done
	    readybusy_r <= 1'b0;
	    nstate <= NAND_DOUT0;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_DATA_GO ) begin // done
	    readybusy_r <= 1'b1;
	    nstate <= NAND_READ_GO; // note same state as follows CMD_READ_GO
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else if( nand_din == CMD_STATUS ) begin // done
	    readybusy_r <= 1'b0;
	    nstate <= NAND_STAT0;
	    unknown_command <= unknown_command;
	    known_command <= nand_din;
	 end else begin
	    readybusy_r <= 1'b0;
	    nstate <= NAND_UNKNOWN0; // done
	    unknown_command <= nand_din;
	    known_command <= nand_din;
	 end
      end else begin // if (!nand_cs && nand_cle && !nand_ale)
	 readybusy_r <= 1'b0;
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
	   default: begin
	      nstate <= NAND_IDLE;
	   end
	 endcase // case (cstate)
      end
   end // always @ (*)

   // write-cycle actions
   reg [2:0] wr_cyc;
   reg [11:0] col_adr;
   reg [17:0] row_adr; // bits 29-12 of overall address
   
   always @(posedge nand_we) begin
      if( local_reset ) begin
	 nand_uk_cmd_updated <= 1'b0;
	 readybusy_w <= 1'b0;
	 wr_cyc <= 3'b0;
      end else begin
	 case (cstate)
	   NAND_IDLE: begin
	      nand_uk_cmd_updated <= 1'b0;
	      readybusy_w <= 1'b0;
	      wr_cyc <= 3'b0;
	   end
	   NAND_UNKNOWN0: begin
	      nand_uk_cmd_updated <= 1'b1;
	      readybusy_w <= 1'b0;
	      wr_cyc <= 3'b0;
	   end
	   NAND_RESET: begin
	      nand_uk_cmd_updated <= 1'b0;
	      readybusy_w <= 1'b1;
	      wr_cyc <= 3'b0;
	   end
	   NAND_READ0: begin
	      nand_uk_cmd_updated <= 1'b0;
	      readybusy_w <= 1'b0;
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
	   end // case: NAND_READ0
	   NAND_DOUT0: begin
	      nand_uk_cmd_updated <= 1'b0;
	      readybusy_w <= 1'b0;
	      if( wr_cyc == 3'h0 ) begin
		 wr_cyc <= wr_cyc + 3'h1;
		 col_adr[7:0] <= nand_din[7:0];
	      end else if( wr_cyc == 3'h1 ) begin
		 wr_cyc <= wr_cyc + 3'h1;
		 col_adr[11:8] <= nand_din[3:0];
	      end else begin
		 wr_cyc <= wr_cyc;
	      end
	   end // case: NAND_DOUT0
	   NAND_READ_GO: begin
	      nand_uk_cmd_updated <= 1'b0;
	      readybusy_w <= 1'b0;
	      wr_cyc <= 3'b0;
	   end
	   default: begin
	      // NAND_ID1 is a nop
	      // NAND_STAT0 is a nop
	      nand_uk_cmd_updated <= 1'b0;
	      readybusy_w <= 1'b0;
	      wr_cyc <= 3'b0;
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
   assign nand_drive_out = !nand_re && !nand_cs;
   assign nand_dout = special_en ? special_data[7:0] : (ram_blank ? 8'hff : ram_d_from_ram[7:0]);

   reg 	      first_read; // delay incremeting address after the first read falling edge
   assign nand_adr_updated = !first_read; // latch address on rising edge of adr_updated
      always @(negedge nand_re or posedge nand_cle or posedge local_reset) begin
      if( nand_cle || local_reset ) begin
	 rd_cycle <= 4'b0;
	 special_data <= 8'b0;
	 special_en <= 1'b0;
	 first_read <= 1'b1;
	 // asynchronously copy over col/row adr when cle is set, or if in reset....
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
      end
   end

   reg rb_w, rb_w1, rb_w_pulse;
   reg rb_r, rb_r1, rb_r_pulse;
   reg [12:0] rb_timer;
   reg 	     rb_counting;
   
   /////// generate an R/B count based on asynchronous pulse input
   always @(posedge clk) begin
      rb_w1 <= readybusy_w;
      rb_w <= rb_w1;

      rb_r1 <= readybusy_r;
      rb_r <= rb_r1;

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
   assign nand_rb = !rb_counting || (rb_timer < 12'h4);
      
endmodule // romulator
