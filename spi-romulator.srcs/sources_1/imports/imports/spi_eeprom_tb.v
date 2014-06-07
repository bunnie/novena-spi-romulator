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

module spi_eeprom_tb;

   wire sdout;
   reg 	sdin;
   reg 	scs;
   reg 	sclk;
   reg 	swp;
   reg 	shold;

   reg 	eim_clk;
   reg [18:0] bus_a;
   reg [15:0] bus_d;
   reg 	      we;
   reg 	      re;
   wire [15:0] reg_d;
   wire [15:0] rbk_d;
   
   wire [7:0]  spi_uk_cmd;  // bitbucket for unknown commands
   wire        spi_uk_cmd_updated;
   
   reg 	       reset;
   reg 	       sclk_on;

   // a RAM for sim sake
//   reg [15:0]  extram [7:0];

   spi_eeprom dut (
		   .sdout(sdout),
		   .sdin(sdin),
		   .scs(scs),
		   .sclk(sclk),
		   .swp(swp),
		   .shold(shold),

		   .eim_clk(eim_clk),
		   .bus_a(bus_a),
		   .bus_d(bus_d),
		   .we(we),
		   .re(re),
		   .reg_d(reg_d),
		   .rbk_d(rbk_d),

		   .spi_uk_cmd(spi_uk_cmd),
		   .spi_uk_cmd_updated(spi_uk_cmd_updated),

		   .reset(reset)
		   );

   parameter PERIOD = 16'd26;   // 37 MHz
   always begin
      if( sclk_on ) begin
	 sclk = 1'b0;
	 #(PERIOD/2) sclk = 1'b1;
	 #(PERIOD/2);
      end else begin
	 sclk = 1'b0;
	 #PERIOD;
      end
   end

   parameter PERIOD_EIM = 16'd26; // should be 125 MHz but adjust to this due to sim artifacts
   always begin
      eim_clk = 1'b0;
      #(PERIOD_EIM/2) eim_clk = 1'b1;
      #(PERIOD_EIM/2);
   end

   task seeprom_idle;
      begin
	 sclk_on = 0;
	 scs = 1;
	 swp = 1;
	 shold = 1;
      end
   endtask // nand_idle

   task eim_idle;
      begin
	 we = 1'b0;
	 re = 1'b0;
	 bus_a = 18'h4E73;
	 bus_d = 16'hzzzz;
      end
   endtask // nand_read_id

   task eim_write;
      input [18:0] t_a;
      input [15:0] t_d;
      begin   // a poor approximation of what happens but good enough
	 bus_a = t_a;
	 #PERIOD_EIM;
	 bus_d = t_d;
	 #PERIOD_EIM;
	 we = 1;
	 #PERIOD_EIM;
	 we = 0;
	 #PERIOD_EIM;
      end
   endtask // eim_write

   task seeprom_id;
      begin
	 scs = 1;
	 sdin = 0;
	 #PERIOD;

	 scs = 0;
	 sdin = 0;
	 #PERIOD;

	 // 9f
	 sclk_on = 1;
	 sdin = 1'b1;
	 #PERIOD;
	 sdin = 1'b0;
	 #PERIOD;
	 sdin = 1'b0;
	 #PERIOD;
	 sdin = 1'b1;
	 #PERIOD;

	 sdin = 1'b1;
	 #PERIOD;
	 sdin = 1'b1;
	 #PERIOD;
	 sdin = 1'b1;
	 #PERIOD;
	 sdin = 1'b1;
	 #PERIOD;

	 #(PERIOD*24);
	 sclk_on = 0;

	 #PERIOD;
	 scs = 1;
      end
   endtask // seeprom_id

   task seeprom_invalid;
      begin
	 scs = 1;
	 sdin = 0;
	 #PERIOD;

	 scs = 0;
	 sdin = 0;
	 #PERIOD;

	 // 03
	 sclk_on = 1;
	 sdin = 1'b0;
	 #PERIOD;
	 sdin = 1'b0;
	 #PERIOD;
	 sdin = 1'b0;
	 #PERIOD;
	 sdin = 1'b0;
	 #PERIOD;

	 sdin = 1'b0;
	 #PERIOD;
	 sdin = 1'b0;
	 #PERIOD;
	 sdin = 1'b1;
	 #PERIOD;
	 sdin = 1'b1;
	 #PERIOD;

	 sdin = 1'b0;
	 #(PERIOD*31);
	 sclk_on = 0;

	 #PERIOD;
	 scs = 1;
      end
   endtask // seeprom_invalid

   integer index;
   task seeprom_read;
      input [15:0] t_a;
      begin
	 scs = 1;
	 sdin = 0;
	 #PERIOD;

	 scs = 0;
	 sdin = 0;
	 #PERIOD;

	 // 0b
	 sclk_on = 1;
	 sdin = 1'b0;
	 #PERIOD;
	 sdin = 1'b0;
	 #PERIOD;
	 sdin = 1'b0;
	 #PERIOD;
	 sdin = 1'b0;
	 #PERIOD;

	 sdin = 1'b1;
	 #PERIOD;
	 sdin = 1'b0;
	 #PERIOD;
	 sdin = 1'b1;
	 #PERIOD;
	 sdin = 1'b1;
	 #PERIOD;

	 // top 8 bits always 0
	 sdin = 1'b0;
	 #(PERIOD*8);

	 for( index = 15; index >=0; index = index - 1 ) begin
	    sdin = t_a[index];
	    #PERIOD;
	 end

	 // dummy cycle
	 sdin = 1'b0;
	 #(PERIOD*8);

	 // and eight bytes of data
	 sdin = 1'b0;
	 #(PERIOD*64);
	 
	 sclk_on = 0;

	 #PERIOD;
	 scs = 1;
	 #PERIOD;
	 #PERIOD;
      end
   endtask // seeprom_read
   
   initial begin
      reset = 0;

      sclk_on = 0;
      sdin = 0;
      scs = 1;
      swp = 1;
      shold = 1;

      bus_a = 0;
      bus_d = 0;
      we = 0;
      re = 0;

      #(PERIOD);
      scs = 0;
      #(PERIOD*2);
      scs = 1;
      //      $readmemb( "f:\largework\fpga\novena-sd-fpga\extram.bin", extram, 0, 65535 );

      #(PERIOD*4);
      $stop;

      // reset
      eim_idle();
      seeprom_idle();
      #(PERIOD*4);
      reset = 1;
      #(PERIOD*4);
      reset = 0;
      #(PERIOD*4);
      #(PERIOD*4);

      #(PERIOD*4);

      // now test
      eim_write(18'h1_0000, 16'hbeef);
      eim_write(18'h1_0004, 16'hdead);
      eim_write(18'h1_0006, 16'h8181);
      eim_write(18'h1_0008, 16'ha581);
      eim_write(18'h1_000A, 16'h6009);

      eim_write(18'h0_0000, 16'h3333);  // this should do nothing
      #(PERIOD*4);

      seeprom_id();
      
      #(PERIOD*4);
      
      seeprom_read(16'h0004);

      #(PERIOD*4);
      
      seeprom_invalid();
      
      #(PERIOD*4);
      
      $stop;
   end // initial begin

endmodule // romulator_tb
