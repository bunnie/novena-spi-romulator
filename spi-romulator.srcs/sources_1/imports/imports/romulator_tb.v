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

module romulator_tb;
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

   // a RAM for sim sake
   reg [15:0]  extram [7:0];

   assign  ram_d_from_ram[7:0] = extram[ram_adr];
   
   romulator dut (
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

		  .ram_adr(ram_adr),
		  .ram_d_to_ram(ram_d_to_ram),
		  .ram_d_from_ram(ram_d_from_ram),
		  .ram_we(ram_we),
		  .ram_clk_to_ram(ram_clk_to_ram),

		  .nand_uk_cmd(nand_uk_cmd),
		  .nand_uk_cmd_updated(nand_uk_cmd_updated),

		  .reset(reset)
		  );
   
   
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
   
   
   initial begin
      nand_wp = 0;
      reset = 0;
      $readmemb( "f:\largework\fpga\novena-sd-fpga\extram.bin", extram, 0, 65535 );

      $stop;

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

      #100;
      
      reset_op();
      #100;
      
      nand_read_id();
      #500;
      
      $stop;
   end // initial begin

endmodule // romulator_tb
