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

module novena_eim(
		  input wire [15:0] din,
		  output wire [15:0] dout,
		  input wire clk,
		  input wire reset,

		  input wire bclk,
		  input wire [1:0] cs,
		  input wire [18:16] hi_addr,
		  input wire lba,
		  input wire oe,
		  input wire rw,
		  output wire rb_wait,

		  input nram_clk,
		  input [15:0] nram_a,
		  input [7:0] nram_din,
		  output [7:0] nram_dout,
		  input nram_we
		  
		  );

   reg 		      bus_write;
   reg 		      rw_del;
   reg 		      lba_del;
   reg [18:0] 	      bus_addr;
   reg [15:0] 	      bus_data;

   reg [15:0] 	      din_r;
   reg 		      rw_r;
   reg 		      cs0_r;
   reg [18:0] 	      addr_r;
   reg 		      lba_r;

   always @(posedge bclk) begin
      addr_r <= {hi_addr, din};
      cs0_r <= cs[0];
      rw_r <= rw;
      din_r <= din;
      lba_r <= lba;
   end

   // leave out the EIM ram on the DDR3 implementation, freeing up space for chipscope
//   novena_eim2 eimram(
//		   .clka(bclk),
//		   .ena(!cs0_r && (bus_addr[18:16] == 3'b0)),
//		   .wea(!rw_r),
//		   .addra(bus_addr[15:1]),
//		   .douta(dout[15:0]),
//		   .dina(din_r[15:0]),
		      
//		   .clkb(nram_clk),
//		   .addrb(nram_a[15:0]),
//		   .web(nram_we),
//		   .dinb(nram_din),
//		   .doutb(nram_dout)
//		   );

   always @(posedge bclk) begin
      if( !lba ) begin // latch address on LBA low
	 bus_addr <= {hi_addr, din};
      end else begin
	 bus_addr <= bus_addr;
      end
   end

   assign rb_wait = 1'b1; // wait is active low, no wait state for now
   
endmodule // novena_eim
