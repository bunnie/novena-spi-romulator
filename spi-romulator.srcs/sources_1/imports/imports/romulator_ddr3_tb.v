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

module romulator_ddr3_tb;
   reg clk;
   
   reg nand_we;
   reg nand_re;
   reg nand_ale;
   reg nand_cle;
   wire nand_rb;
   reg 	nand_wp;
   reg 	nand_cs;
   
   reg [7:0]  nand_din;
   wire [7:0] nand_dout;
   wire       nand_drive_out;

   wire [15:0] ram_adr;
   wire [7:0]  ram_d_to_ram;
   wire [7:0]   ram_d_from_ram;
   wire        ram_we;
   wire        ram_clk_to_ram;

   wire [7:0]  nand_uk_cmd;    // pipe to a FIFO to store unknown commands
   wire        nand_uk_cmd_updated;

   reg 	       reset;

   reg 	       rom_ddr3_reset; // reset just the ddr3-romulator interface
		      
   wire        ddr3_wr_clk;
   wire        ddr3_wr_cmd_en;
   wire [2:0]  ddr3_wr_cmd_instr;
   wire [5:0]  ddr3_wr_cmd_bl;
   wire [29:0] ddr3_wr_adr;
   reg 	       ddr3_wr_cmd_full;
   wire        ddr3_wr_cmd_empty;

   wire        ddr3_wr_dat_en;
   wire [31:0] ddr3_wr_dat;
   reg 	       ddr3_wr_full;
   reg 	       ddr3_wr_empty;
	      
   wire        ddr3_rd_clk;
   wire        ddr3_rd_cmd_en;
   wire [2:0]  ddr3_rd_cmd_instr;
   wire [5:0]  ddr3_rd_cmd_bl;
   wire [29:0] ddr3_rd_adr;
   reg 	       ddr3_rd_cmd_full;
   wire        ddr3_rd_dat_en;
   reg [31:0]  ddr3_rd_dat;
   reg 	       ddr3_rd_dat_empty;
   reg 	       ddr3_rd_dat_full;
   reg 	       ddr3_rd_dat_overflow; // need to monitor this
   
   wire        page_addra_over;
   wire        outstanding_under;

   glbl glbl(); // add global reset capabiilities
   
   romulator_ddr3 dut (
		       .clk(clk),
		       .nand_we(  nand_we),
		       .nand_re(  nand_re),
		       .nand_ale( nand_ale),
		       .nand_cle( nand_cle),
		       .nand_rb(  nand_rb),
		       .nand_wp(  nand_wp),
		       .nand_cs(  nand_cs),

		       .nand_din(nand_din),
		       .nand_dout(nand_dout),
		       .nand_drive_out(nand_drive_out),

		       .rom_ddr3_reset(rom_ddr3_reset),
		      
		       .ddr3_wr_clk(ddr3_wr_clk),
		       .ddr3_wr_cmd_en(ddr3_wr_cmd_en),
		       .ddr3_wr_cmd_instr(ddr3_wr_cmd_instr[2:0]),
		       .ddr3_wr_cmd_bl(ddr3_wr_cmd_bl[5:0]),
		       .ddr3_wr_adr(ddr3_wr_adr[29:0]),
		       .ddr3_wr_cmd_full(ddr3_wr_cmd_full),
		       .ddr3_wr_cmd_empty(ddr3_wr_cmd_empty),
		       .ddr3_wr_dat_en(ddr3_wr_dat_en),
		       .ddr3_wr_dat(ddr3_wr_dat[31:0]),
		       .ddr3_wr_full(ddr3_wr_full),
		       .ddr3_wr_empty(ddr3_wr_empty),
		      
		       .ddr3_rd_clk(ddr3_rd_clk),
		       .ddr3_rd_cmd_en(ddr3_rd_cmd_en),
		       .ddr3_rd_cmd_instr(ddr3_rd_cmd_instr[2:0]),
		       .ddr3_rd_cmd_bl(ddr3_rd_cmd_bl[5:0]),
		       .ddr3_rd_adr(ddr3_rd_adr[29:0]),
		       .ddr3_rd_cmd_full(ddr3_rd_cmd_full),
		       .ddr3_rd_dat_en(ddr3_rd_dat_en),
		       .ddr3_rd_dat(ddr3_rd_dat[31:0]),
		       .ddr3_rd_dat_empty(ddr3_rd_dat_empty),
		       .ddr3_rd_dat_count(data_count[6:0]),
		       .ddr3_rd_dat_full(ddr3_rd_dat_full),
		       .ddr3_rd_dat_overflow(ddr3_rd_dat_overflow),
		       
		       .page_addra_over(page_addra_over),
		       .outstanding_under(outstanding_under),

		       .nand_uk_cmd(nand_uk_cmd),
		       .nand_uk_cmd_updated(nand_uk_cmd_updated),

		       .reset(reset)
		  );

   // emulate a very simple DDR3-MIG moderated memory interface here
   reg [1:0]   cmd_count;
   reg [7:0]   data_count;
   reg [7:0]   wdata_count;
   reg [31:0]  latency_timer;
   reg [31:0]  wlatency_timer;
   reg 	       add_data;
   reg 	       minus_data;
   reg 	       ddr3_go;
   reg 	       ddr3_wgo;
   reg [1:0]   wcmd_count;

   parameter DDR3_TURNAROUND_TIME = 32'h20;

   assign ddr3_wr_cmd_empty = (wcmd_count == 2'b00);
   
   // we're going to cheat and use blocking assignments.
   always @(posedge clk) begin
      if( ddr3_rd_cmd_en ) begin
	 if( cmd_count == 2'b11 ) begin
	    cmd_count = cmd_count;
	 end else begin
	    cmd_count = cmd_count + 1;
	 end
      end else begin
	 cmd_count = cmd_count;
      end

      if( ddr3_wr_cmd_en ) begin
	 if( wcmd_count == 2'b11 ) begin
	    wcmd_count = wcmd_count;
	 end else begin
	    wcmd_count = wcmd_count + 1;
	 end
      end else begin
	 wcmd_count = wcmd_count;
      end
      
      if( cmd_count == 2'b11 ) begin
	 ddr3_rd_cmd_full = 1'b1;
      end else begin
	 ddr3_rd_cmd_full = 1'b0;
      end

      if( wcmd_count == 2'b11  || (wcmd_count == 2'b10 && ddr3_wr_cmd_en)) begin
	 ddr3_wr_cmd_full = 1'b1;
      end else begin
	 ddr3_wr_cmd_full = 1'b0;
      end
      
      if( cmd_count > 2'b00 ) begin
	 ddr3_go = 1;
      end else begin
	 ddr3_go = 0;
      end

      if( wcmd_count > 2'b00 ) begin
	 ddr3_wgo = 1;
      end else begin
	 ddr3_wgo = 0;
      end

      if( ddr3_wr_dat_en ) begin
	 wdata_count <= wdata_count + 1;
      end

      if( wdata_count > 0 ) begin
	 ddr3_wr_empty <= 1'b0;
      end else begin
	 ddr3_wr_empty <= 1'b1;
      end

      if( ddr3_wgo ) begin
	 wlatency_timer = wlatency_timer + 1;
	 if( wdata_count != 8'b0 ) begin
	    wdata_count <= wdata_count - 1;
	 end
      end

      if( wlatency_timer >= 32'd16 ) begin
	 wlatency_timer <= 32'd0;
	 wcmd_count <= wcmd_count - 2'b1;
      end

      if( wdata_count >= 8'd64 ) begin
	 ddr3_wr_full <= 1'b1;
      end else begin
	 ddr3_wr_full <= 1'b0;
      end
      
      if( ddr3_go && !add_data ) begin
	 latency_timer = latency_timer + 1;
      end else if (add_data || !ddr3_go) begin
	 latency_timer = 0;
      end else begin
	 latency_timer = latency_timer;
      end
	 
      if( latency_timer > DDR3_TURNAROUND_TIME ) begin
	 add_data = 1;
	 cmd_count = cmd_count - 1;
	 latency_timer = 0;
      end else begin
	 add_data = 0;
      end

      if( add_data ) begin
	 if( data_count < 8'd64 ) begin
	    data_count = data_count + 8'd16;  // we get 16 words at a time
	    ddr3_rd_dat_full = 1'b0;
	 end else if (data_count == 8'd64) begin
	    ddr3_rd_dat_overflow = 1'b0;
	    ddr3_rd_dat_full = 1'b1;
	    data_count = data_count;
	 end else begin
	    ddr3_rd_dat_overflow = 1'b1;
	    ddr3_rd_dat_full = 1'b1;
	    data_count = data_count;
	 end
      end else begin
	 // need a delete data entry here too
	 data_count = data_count;
      end // else: !if( add_data )


      if( ddr3_rd_dat_en ) begin
	 if( data_count > 8'd0 ) begin
	    ddr3_rd_dat = ddr3_rd_dat + 32'h01010101;
	    data_count = data_count - 1;
	 end else begin
	    data_count = 0;
	 end
      end

      if( data_count[6:0] > 7'd0 ) begin
	 ddr3_rd_dat_empty = 1'b0;
      end else begin
	 ddr3_rd_dat_empty = 1'b1;
      end
      
   end
   
   parameter PERIOD = 16'd8;   // 125 MHz (close to 133 MHz actual)
   always begin
      clk = 1'b0;
      #(PERIOD/2) clk = 1'b1;
      #(PERIOD/2);
   end

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
	 nand_din = 8'hZZ;
	 #20;

	 nand_ale = 1'b1;
	 #25;

	 nand_we = 1'b0;
	 nand_din = 8'h00;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 nand_din = 8'hZZ;
	 #20;

	 nand_ale = 1'b0;

	 #10;
	 nand_re = 1'b0;
	 #25;
	 nand_re = 1'b1;
	 #25;
	 nand_re = 1'b0;
	 #25;
	 nand_re = 1'b1;
	 #25;
	 nand_re = 1'b0;
	 #25;
	 nand_re = 1'b1;
	 #25;
	 nand_re = 1'b0;
	 #25;
	 nand_re = 1'b1;
	 #25;
	 nand_re = 1'b0;
	 #25;
	 nand_re = 1'b1;
	 #25;

	 nand_cs = 1'b1;
      end
   endtask; // nand_read_id

   task unknown_op;
      begin
	 nand_cs = 1'b0;

	 nand_cle = 1'b1;
	 nand_ale = 1'b0;
	 nand_we = 1'b0;
	 nand_din = 8'h44; // some random unknown opcode
	 #25;
	 nand_we = 1'b1;
	 #5;
	 nand_cle = 1'b0;

	 nand_cs = 1'b1;
      end
   endtask; // unknown_op
   
   task reset_op;
      begin
	 nand_cs = 1'b0;

	 nand_cle = 1'b1;
	 nand_ale = 1'b0;
	 nand_we = 1'b0;
	 nand_din = 8'hFF; // some random unknown opcode
	 #25;
	 nand_we = 1'b1;
	 #5;
	 nand_cle = 1'b0;

	 nand_cs = 1'b1;

	 #6000;
      end
   endtask; // reset_op
   
   task status_op;
      begin
	 nand_cs = 1'b0;

	 nand_cle = 1'b1;
	 nand_ale = 1'b0;
	 nand_we = 1'b0;
	 nand_din = 8'h70; // some random unknown opcode
	 #25;
	 nand_we = 1'b1;
	 #5;
	 nand_cle = 1'b0;

	 #100;
	 nand_re = 1'b0;
	 #25;
	 nand_re = 1'b1;
	 #20;

	 nand_cs = 1'b1;
      end
   endtask; // status_op
   

   task nand_read_op;
      input [29:0] adr;
      begin
	 nand_cs = 1'b0;

	 nand_cle = 1'b1;
	 nand_ale = 1'b0;
	 nand_we = 1'b0;
	 nand_din = 8'h00;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 nand_cle = 1'b0;
	 nand_ale = 1'b1;
	 
	 nand_din = adr[7:0];
	 #20;
	 
	 nand_we = 1'b0; 
	 #25;
	 nand_we = 1'b1;
	 #5;
	 
	 nand_din = {4'b0,adr[11:8]};
	 #20;

	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 
	 nand_din = adr[19:12];
	 #20;

	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 
	 nand_din = adr[27:20];
	 #20;
	 
	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 
	 nand_din = {6'b0,adr[29:28]};
	 #20;
	 
	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;

	 nand_cle = 1'b1;
	 nand_din = 8'h30;
	 nand_ale = 1'b0;
	 #20;
	 
	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;

	 nand_cle = 1'b0;

	 while( nand_rb == 1'b0 ) begin
	    #50;
	 end
	 
//	 #7000;
	 #500;

	 nand_re = 1'b0;
	 #25;
	 nand_re = 1'b1;
	 #25;
	 nand_re = 1'b0;
	 #25;
	 nand_re = 1'b1;
	 #25;
	 nand_re = 1'b0;
	 #25;
	 nand_re = 1'b1;
	 #25;
	 nand_re = 1'b0;
	 #25;
	 nand_re = 1'b1;
	 #25;
	 
	 nand_cs = 1'b1;
	 
      end
   endtask; // nand_read_op

   reg [7:0] testdat;
   task nand_write_op;
      input [29:0] adr;
      begin
	 testdat  = 8'b0;
	 
	 nand_cs = 1'b0;

	 nand_cle = 1'b1;
	 nand_ale = 1'b0;
	 nand_we = 1'b0;
	 nand_din = 8'h80;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 nand_cle = 1'b0;
	 nand_ale = 1'b1;
	 
	 nand_din = adr[7:0];
	 #20;
	 
	 nand_we = 1'b0; 
	 #25;
	 nand_we = 1'b1;
	 #5;
	 
	 nand_din = {4'b0,adr[11:8]};
	 #20;

	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 
	 nand_din = adr[19:12];
	 #20;

	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 
	 nand_din = adr[27:20];
	 #20;
	 
	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 
	 nand_din = {6'b0,adr[29:28]};
	 #20;
	 
	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 nand_ale = 1'b0;
	 #25;

	 nand_we = 1'b0;
	 nand_din = testdat;
	 #25;
	 nand_we = 1'b1;
	 #25;
	 testdat = testdat + 8'b10;

	 nand_we = 1'b0;
	 nand_din = testdat;
	 #25;
	 nand_we = 1'b1;
	 #25;
	 testdat = testdat + 8'b10;

	 nand_we = 1'b0;
	 nand_din = testdat;
	 #25;
	 nand_we = 1'b1;
	 #25;
	 testdat = testdat + 8'b10;

	 nand_we = 1'b0;
	 nand_din = testdat;
	 #25;
	 nand_we = 1'b1;
	 #25;
	 testdat = testdat + 8'b10;

	 nand_we = 1'b0;
	 nand_din = testdat;
	 #25;
	 nand_we = 1'b1;
	 #25;
	 testdat = testdat + 8'b10;

	 nand_we = 1'b0;
	 nand_din = testdat;
	 #25;
	 nand_we = 1'b1;
	 #25;
	 testdat = testdat + 8'b10;

	 nand_we = 1'b0;
	 nand_din = testdat;
	 #25;
	 nand_we = 1'b1;
	 #25;
	 testdat = testdat + 8'b10;

	 nand_we = 1'b0;
	 nand_din = testdat;
	 #25;
	 nand_we = 1'b1;
	 #25;
	 testdat = testdat + 8'b10;

	 nand_cle = 1'b1;
	 nand_din = 8'h10;
	 nand_ale = 1'b0;
	 #20;
	 
	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;

	 nand_cle = 1'b0;

	 while( nand_rb == 1'b0 ) begin
	    #50;
	 end
	 
	 nand_cs = 1'b1;
	 
      end
   endtask; // nand_write_op


   task nand_erase_op;
      input [29:0] adr;
      begin
	 testdat  = 8'b0;
	 
	 nand_cs = 1'b0;

	 nand_cle = 1'b1;
	 nand_ale = 1'b0;
	 nand_we = 1'b0;
	 nand_din = 8'h60;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 nand_cle = 1'b0;
	 nand_ale = 1'b1;
	 
	 nand_din = adr[19:12];
	 #20;

	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 
	 nand_din = adr[27:20];
	 #20;
	 
	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 
	 nand_din = {6'b0,adr[29:28]};
	 #20;
	 
	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;
	 nand_ale = 1'b0;
	 #25;

	 nand_cle = 1'b1;
	 nand_din = 8'hD0;
	 nand_ale = 1'b0;
	 #20;
	 
	 nand_we = 1'b0;
	 #25;
	 nand_we = 1'b1;
	 #5;

	 nand_cle = 1'b0;

	 while( nand_rb == 1'b0 ) begin
	    #50;
	 end
	 
	 nand_cs = 1'b1;
	 
      end
   endtask; // nand_erase_op
   
   
   initial begin
      // emu params
      cmd_count = 0;
      data_count = 0;
      latency_timer = 0;
      add_data = 0;
      ddr3_go = 0;
      
      wcmd_count = 0;
      wdata_count = 0;
      wlatency_timer = 0;
      ddr3_wgo = 0;
      
      // normal params
      nand_wp = 0;
      reset = 0;
      ddr3_wr_cmd_full = 0;
      ddr3_wr_full = 0;
      ddr3_rd_cmd_full = 0;
      ddr3_rd_dat = 32'h04030201;
      ddr3_rd_dat_full = 0;
      ddr3_rd_dat_overflow = 0;
      rom_ddr3_reset = 0;

      // reset
      nand_idle();
      #(PERIOD*4);
      reset = 1;
      #(PERIOD*4);
      reset = 0;
      #(PERIOD*4);
      #(PERIOD*4);
      reset = 1;
      #(PERIOD*4);
      reset = 0;
      #(PERIOD*4);
      #100;
      

      // now test
      nand_read_id();
      #100;
      nand_read_id();

      // postamble
      #50;
      nand_idle();
      #100;

      unknown_op();
      #100;

      status_op();
      #100;

      nand_read_op(29'h0);
      #100;
      // address 20100 -> row/col 40100
      nand_read_op(29'h40100); // maps to 100
      #100;
      // address 80210 -> 100210
      nand_read_op(29'h100210); // maps to 8210
      #100;
      // address 81000 -> 120000
      nand_read_op(29'h120000); // maps to invalid
      #100;
      // address 100000 -> 200000
      nand_read_op(29'h200000); // maps to 9000
      #100;
      // address 120000 -> 240000
      nand_read_op(29'h240000); // maps to invalid

      #1000;

      nand_write_op(29'h0);
      
      #1000;
      
      nand_write_op(29'h100210);
      #100;
      
      #1000;

      nand_read_op(29'h0);
      #100;
      // address 20100 -> row/col 40100
      
      reset_op();
      #100;

      nand_erase_op(29'h100210);
      #1000;
      
      nand_read_id();
      #500;
      
      $stop;
   end // initial begin

endmodule // romulator_ddr3_tb

