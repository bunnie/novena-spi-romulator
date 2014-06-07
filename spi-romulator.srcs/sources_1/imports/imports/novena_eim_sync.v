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

module novena_eim_sync(
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

		  
		  );

   reg 		      bus_write;
   reg 		      rw_del;
   reg 		      lba_del;
   reg [18:0] 	      bus_addr;
   reg [15:0] 	      bus_data;

   reg [15:0] 	      din_r;
   reg 		      rw_r;
   reg 		      cs1_r;
   reg [18:0] 	      addr_r;
   reg 		      lba_r;

   always @(posedge bclk) begin
      addr_r <= {hi_addr, din};
      cs1_r <= cs[1];
      rw_r <= rw;
      din_r <= din;
      lba_r <= lba;
   end

   always @(posedge bclk) begin
      if( !lba ) begin // latch address on LBA low
	 bus_addr <= {hi_addr, din};
      end else begin
	 bus_addr <= bus_addr;
      end
   end

   assign rb_wait = 1'b1; // wait is active low, no wait state for now
   
endmodule // novena_eim_sync

