module spi_eeprom(
		  output  wire sdout,
		  input   wire sdin,
		  input   wire scs,
		  input   wire sclk,
		  input   wire swp,
		  input   wire shold,

		  input wire eim_clk,
		  input wire [18:0] bus_a,
		  input wire [15:0] bus_d,
		  input wire we,
		  input wire re,
		  output wire [15:0] reg_d,
		  output reg [15:0] rbk_d,
		  
		  output reg [7:0] spi_uk_cmd,  // bitbucket for unknown commands
		  output reg spi_uk_cmd_updated,
		  
		  output reg spi_byte_updated,  // updates every byte
		  output reg [7:0] spi_byte,
		  output reg spi_obyte_updated,
		  output reg [7:0] spi_obyte,

		  output wire [23:0] spi_adr,
		  output reg spi_adr_updated,
		  
		  input   reset
		  );

   reg [7:0] 		  inreg;
   reg [31:0] 		  addreg; // top 8 bits are dummy (replica of command)
   reg [23:0] 		  address; // address to read
   reg [15:0] 		  cycle; // cycle counter
   reg [7:0] 		  command; 

   reg [23:0] 		  shiftout; // ID shift out register
   reg 			  readmode; // 1 if in read mode
   reg [7:0] 		  uk_cmd;  // capture unknown commands
   reg 			  uk_cmd_stb; // pulse high once sync to sclk for fifo

   wire [31:0] 		  rom_dout;
   reg [31:0] 		  dataout; // shift register for data going out

   assign spi_adr = address;
   
   always @(posedge sclk or posedge scs) begin
      if( scs ) begin
	 inreg[7:0] <= 8'b0;
      end else begin
	 inreg[7:0] <= {inreg[6:0],sdin};
      end
   end
   
   always @(posedge sclk or posedge scs) begin
      if( scs ) begin
	 addreg[31:0] <= 32'b0;
      end else begin
	 addreg[31:0] <= {addreg[30:0],sdin};
      end
   end

   always @(posedge sclk or posedge scs) begin
      if( scs ) begin
	 cycle <= 16'b0;
      end else begin
	 cycle <= cycle + 16'b1;
      end
   end

   always @(posedge sclk) begin
      spi_obyte[7:0] <= {spi_obyte[6:0],sdout};
      
      if( cycle[2:0] == 3'b111 ) begin
	 spi_obyte_updated <= 1'b1;
      end else begin
	 spi_obyte_updated <= 1'b0;
      end
   end

   always @(posedge sclk) begin
      spi_byte[7:0] <= {spi_byte[6:0],sdin};
      
      if( cycle[2:0] == 3'b111 ) begin
	 spi_byte_updated <= 1'b1;
      end else begin
	 spi_byte_updated <= 1'b0;
      end
   end
	 
   always @(negedge sclk) begin
      if( cycle == 16'd9 ) begin
	 command <= inreg;
      end else begin
	 command <= command;
      end
      
      if( cycle == 16'h20 ) begin
	 address[23:0] <= {addreg[23:2],2'b00}; // we're just going to make this bad
	 spi_adr_updated <= 1'b1;
	 // assumption that all reads are word-aligned
      end else if( (cycle > 16'd40) && (cycle[4:0] == 5'b1_1111) ) begin
	 address[23:0] <= address[23:0] + 24'd4;
	 spi_adr_updated <= 1'b0;
      end else begin
	 address <= address;
	 spi_adr_updated <= 1'b0;
      end
   end

 `ifdef ORIG_CODE
   always @(negedge sclk) begin
      if( cycle < 16'd8 ) begin // command not valid until 9 but need to know sooner
	 shiftout <= shiftout;
	 readmode <= 1'b0;
	 spi_uk_cmd_updated <= 1'b0;
	 spi_uk_cmd <= spi_uk_cmd;
      end else if( cycle == 16'd8 ) begin // do command dispatch
	 if( inreg[7:0] == 8'h9f ) begin
	    shiftout <= 24'hc86017;
	    readmode <= 1'b0;
	    spi_uk_cmd_updated <= 1'b1; // grab known for now
	    spi_uk_cmd <= inreg[7:0];
	 end else if( inreg[7:0] == 8'h0b ) begin
	    readmode <= 1'b1;
	    shiftout <= shiftout;
	    spi_uk_cmd_updated <= 1'b1; // grab known for now
	    spi_uk_cmd <= inreg[7:0];
	 end else begin
	    spi_uk_cmd <= inreg[7:0]; // record the unknown command
	    spi_uk_cmd_updated <= 1'b1;
	    readmode <= 1'b0;
	    shiftout <= shiftout;
	 end
      end else begin  // cycle > 16'd7
	 shiftout[23:0] <= {shiftout[22:0],shiftout[23]};
	 readmode <= readmode;
	 spi_uk_cmd_updated <= 1'b0;
	 spi_uk_cmd <= spi_uk_cmd;
      end
   end // always @ (negedge sclk)

   assign sdout = readmode ? dataout[31] : shiftout[23];

   always @(negedge sclk) begin
      // 8 cycles for command
      // 24 cycles for address
      // 8 dummy cycles because it's command 0B (only command we support)
      // 40th cycle start shifting the data
      if ( (cycle >= 16'd33) && (cycle[4:0] == 5'b0_1000) ) begin
	 dataout[31:0] <= {rom_dout[7:0],rom_dout[15:8],rom_dout[23:16],rom_dout[31:24]};
      end else if( cycle >= 16'd41 ) begin
	 dataout[31:0] <= {dataout[30:0],dataout[31]};
      end else begin
	 dataout <= dataout;
      end
   end // always @ (negedge sclk)
 `else // !`ifdef ORIG_CODE
   always @(posedge sclk) begin
      if( cycle < 16'd7 ) begin // command not valid until 9 but need to know sooner
	 shiftout <= shiftout;
	 readmode <= 1'b0;
	 spi_uk_cmd_updated <= 1'b0;
	 spi_uk_cmd <= spi_uk_cmd;
      end else if( cycle == 16'd7 ) begin // do command dispatch
	 if( {inreg[6:0],sdin} == 8'h9f ) begin
	    shiftout <= 24'hc86017;
	    readmode <= 1'b0;
	    spi_uk_cmd_updated <= 1'b1; // grab known for now
	    spi_uk_cmd <= inreg[7:0];
	 end else if( {inreg[6:0],sdin} == 8'h0b ) begin
	    readmode <= 1'b1;
	    shiftout <= shiftout;
	    spi_uk_cmd_updated <= 1'b1; // grab known for now
	    spi_uk_cmd <= inreg[7:0];
	 end else begin
	    spi_uk_cmd <= inreg[7:0]; // record the unknown command
	    spi_uk_cmd_updated <= 1'b1;
	    readmode <= 1'b0;
	    shiftout <= shiftout;
	 end
      end else begin  // cycle > 16'd7
	 shiftout[23:0] <= {shiftout[22:0],shiftout[23]};
	 readmode <= readmode;
	 spi_uk_cmd_updated <= 1'b0;
	 spi_uk_cmd <= spi_uk_cmd;
      end
   end // always @ (negedge sclk)

   assign sdout = readmode ? dataout[31] : shiftout[23];

   always @(posedge sclk) begin
      // 8 cycles for command
      // 24 cycles for address
      // 8 dummy cycles because it's command 0B (only command we support)
      // 40th cycle start shifting the data
      if ( (cycle >= 16'd32) && (cycle[4:0] == 5'b0_0111) ) begin
	 dataout[31:0] <= {rom_dout[7:0],rom_dout[15:8],rom_dout[23:16],rom_dout[31:24]};
      end else if( cycle >= 16'd40 ) begin
	 dataout[31:0] <= {dataout[30:0],dataout[31]};
      end else begin
	 dataout <= dataout;
      end
   end // always @ (negedge sclk)
   
`endif   

   // if cycle == 8'd34 && readmode, issue the read request to the dram interface
   // time to respond is thus 6 cycles * 37ns = 222 ns. Max DDR3 latency is 85ns. We should be ok.
   // this gives 130ns for data readback, which is enough time to trasnfer 208 bytes -- not 
   // quite a whole page. We need 160ns to get the full page.
   // see if we can optimize to 7 cycles allowed response time to first data out

   wire my_a;

   // 19'h1_0000 >> 1 due to short-align
   assign my_a = (bus_a[18:16] == 3'b001); 
   
   eimram spi_romulator_64k (
     .clka(eim_clk), // input clka
     .ena(my_a), // input ena
     .wea(we), // input [0 : 0] wea
     .addra(bus_a[15:1]), // input [14 : 0] addra
     .dina(bus_d[15:0]), // input [15 : 0] dina
     .clkb(sclk), // input clkb
     .addrb(address[15:2]), // input [13 : 0] addrb
     .doutb(rom_dout) // output [31 : 0] doutb  // 4 bytes at a time
     );

   // shadow for readback (chee-z three portnedness
   wire [31:0] state;
   eimram spi_romulator_64k_shadow (
     .clka(eim_clk), // input clka
     .ena(my_a), // input ena
     .wea(we), // input [0 : 0] wea
     .addra(bus_a[15:1]), // input [14 : 0] addra
     .dina(bus_d[15:0]), // input [15 : 0] dina
     .clkb(eim_clk), // input clkb
     .addrb(bus_a[15:2]), // input [13 : 0] addrb
     .doutb(state) // output [31 : 0] doutb
     );
   
   always @(bus_a or my_a or re or state) begin
      if( my_a && re ) begin
	 if( bus_a[1] == 1'b0 ) begin
	    rbk_d = state[15:0];
	 end else begin
	    rbk_d = state[31:16];
	 end
      end else begin
	 rbk_d = 16'hZZZZ;
      end
   end
   
endmodule // spi_eeprom
