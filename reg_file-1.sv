/*
Mina Gao and Justin Sim
4/18/2024
EE 469 Hussein
Lab 1 & 2
	
Reg File:
This module represents a register file, a storage component in a digital system composed of multiple registers.
It serves as a bank of storage locations, each typically capable of storing a single binary value.
The register file allows reading from and writing to individual registers based on specified control signals.
It facilitates data movement and storage within the system.

*/
module reg_file(
	input logic 			clk, wr_en,
	input logic [31:0]   write_data,
	input logic [3:0]    write_addr, read_addr1, read_addr2,
	output logic [31:0]  read_data1, read_data2
);
	
	logic [15:0][31:0] mem;
	
	// write operation
	always_ff@(posedge clk) begin
		if (wr_en) begin
			mem[write_addr] <= write_data;
		end
	end
	
	// read operations
	assign read_data1 = mem[read_addr1];
	assign read_data2 = mem[read_addr2];
endmodule

//-----------------------------------------------------------------------------
// Register File Testbench
// Covers 3 test scenarios given in spec to verify functionality
module reg_file_tb;

	logic 			clk, wr_en;
	logic [31:0]   write_data;
	logic [3:0]    write_addr, read_addr1, read_addr2;
	logic [31:0]  read_data1, read_data2;
	
  // Instantiate reg_file module
  reg_file reg_file_inst(.*);
  
  // Clock generation
  parameter clock_period = 100;
  initial begin clk <= 0;
    forever #(clock_period /2) clk <= ~clk;
  end
  
  // Run Test Scenarios (uncomment to run each individually)
  initial begin
//		write_test;
		read_test;
//		write_read_test;
		$stop;
	end
  
  //--------------------------------------------------------------------------------------------
  // Test scenario 1: Write data is written the cycle after wr_en is asserted
  
  // "Write data is written into the register file the clock cycle after wr_en is asserted"
  //--------------------------------------------------------------------------------------------
  task write_test; begin
  
    // Test write operation
    wr_en = 1;
    write_addr = 2; 
    write_data = 32'hABCDEFFF;
	 @(posedge clk);
	 
    // Test read operation in the next cycle
    wr_en       = 0;
    write_addr  = 0;
    write_data  = 0;
    read_addr1  = 2;
	 @(posedge clk);
	 
    // Check if the written data is available in the next cycle
    if(read_data1 != 32'hABCDEFFF) $fatal("Test Scenario 1 FAIL at", $time);
	 else $display("Test Scenario 1 GOOD at", $time);
	 
  end endtask : write_test
  //--------------------------------------------------------------------------------------------
  // Test scenario 2: Read data is updated the same cycle the address was provided
  
  // "Read data is updated to the register data at an address the same cycle the address was
  // provided. Do this for both read addresses and data outputs"
  //--------------------------------------------------------------------------------------------
  task read_test; begin
  
	 wr_en = 1;
	 write_addr = 2; write_data = 32'hAAAAAAAA;
	 @(posedge clk);
	 
	 read_addr1 = 2; if (read_data1 != 32'hAAAAAAAA) $fatal("read_addr1: Test Scenario 2 FAIL at", $time);
						  else $display("read_addr1: Test Scenario 2 GOOD at", $time); #5; 
	 read_addr2 = 2; if (read_data2 != 32'hBBBBBBBB) $fatal("read_addr2: Test Scenario 2 FAIL at", $time);
						  else $display("read_addr2: Test Scenario 2 GOOD at", $time);	#5;
						  
  end endtask
	//--------------------------------------------------------------------------------------------
  // Test scenario 3: Read data is updated to write data the cycle after the address was provided
  
  // "Read data is updated to write data at an address the cycle after the address was provided
  // if the write address is the same and wr_en was asserted. Do this for both read addresses
  // and data outputs"
  //--------------------------------------------------------------------------------------------
  task write_read_test; begin
    // Write operation
    wr_en = 1;
    write_addr = 2;
    write_data = 32'h12345678;
    @(posedge clk);
	 // provide a read address to same write_addr
	 read_addr1 = 2;
	 @(posedge clk);
	 // change write data at same address
    wr_en = 1;
    write_addr = 2;
    write_data = 32'hCCCCCCCC;
    @(posedge clk);
	 @(posedge clk);
    // Check if the read data 1 is updated to write data in the next cycle
    if (read_data1 != 32'hCCCCCCCC) $fatal("write_read_test FAIL: read_data: %h, \n expected read_data: %h", read_data1, write_data);
	 else $display("read_addr1: Test Scenario 3 GOOD");
	 //------------------------------
	 //--- Repeat for read_addr2--//
	 //------------------------------
	 // Write operation
    wr_en = 1;
    write_addr = 2;
    write_data = 32'h12345678;
    @(posedge clk);
	 // provide a read address to same write_addr
	 read_addr2 = 2;
	 @(posedge clk);
	 // change write data at same address
    wr_en = 1;
    write_addr = 2;
    write_data = 32'hCCCCCCCC;
    @(posedge clk);
	 @(posedge clk);
    // Check if the read data 1 is updated to write data in the next cycle
    if (read_data2 != 32'hCCCCCCCC) $fatal("write_read_test FAIL: read_data: %h, \n expected read_data: %h", read_data2, write_data);
	 else $display("read_addr1: Test Scenario 3 GOOD");
  end endtask

endmodule