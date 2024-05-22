/*
Mina Gao and Justin Sim
4/18/2024
EE 469 Hussein
Lab 1 & 2
	
	Carry Ripple Adder Module: 
   This module implements a carry ripple adder, supporting binary addition operations. 
   It consists of multiple full adders connected in a ripple-carry configuration. 
   The module accepts two input vectors for addition/subtraction and produces a sum vector. 
   Additionally, it generates a carry-out signal to indicate overflow during addition. 
   The carry ripple adder module accommodates addition and subtraction of binary numbers 

*/
module CarryRippleAdder #(
	parameter WIDTH = 32
	)(
    input logic 	[WIDTH-1:0] a,b,
	 input logic 	cin,
    output logic 	[WIDTH-1:0] sum,
	 output logic 	cout
);
	// hold value of cin for each iteration form 0 to WIDTH
	logic [WIDTH:0] c;
	assign c[0] = cin;
	
	// iterate fullAdder through each bit of a and b
	generate 
		genvar i;
				for (i=0; i < WIDTH; i++) begin
					fullAdder fa (.a(a[i]),
									  .b(b[i]),
									  .cin(c[i]),
									  .sum(sum[i]),
									  .cout(c[i+1])
									  );
				end
		endgenerate
	
	assign cout = c[WIDTH];

endmodule : CarryRippleAdder
//-----------------------------------------------------------------------------
// CarryRippleAdder Testbench
// It has 3 different tasks to verify functionality of CarryRippleAdder
// It includes a constrained Random Verification task called rand_vect_10
// It has an adjustable WIDTH to verify successful implementation of varying
// input sizes

module CarryRippleAdder_tb ();
parameter WIDTH=4;
	//--------------------------------
	// Define IN OUT for FUT
	//--------------------------------
	logic [WIDTH-1:0] a,b,sum;
	logic cin, cout;	
	
	//--------------------------------
	// Instantiate FullAdder
	//--------------------------------
	CarryRippleAdder #(WIDTH) dut(.*);
	
	
	//--------------------------------
	// Run Testing Procedures
	//--------------------------------
	initial begin
		rand_vect_10; //SUCCESS
		$stop;
	end
	
	
	//---------------------------------------------------------------------
	// Define Tasks for testing Functionality
	//---------------------------------------------------------------------
	task test; begin
		a = 4'd5;
		b = 4'd5;
		cin = 0;
		#10;
		if (sum == a + b)
				$display("SUCCESSFUL ADDITION \n a: %b,b: %b,sum: %b,cout: %b", a,b,sum,cout);
			else 
				$fatal("ERROR in ADD\n a: %d, b: %d, sum: %d,cout: %b", a,b,sum,cout);
			#10;
	end endtask
	
	// test_32: Test all cases
	task test_all; begin
		integer i;
		// Test Addition
		for(i=0;i<2**2*WIDTH;i++) begin
			cin = 0;
			{a,b} = i; #10;
			
			if (sum == a + b)
				$display("SUCCESSFUL ADDITION \n a: %d,b: %d,sum: %d,cout: %b", a,b,sum,cout);
			else 
				$fatal("ERROR in ADD\n a: %d, b: %d, sum: %d,cout: %b", a,b,sum,cout);
			#10;
		end
	end endtask
	
	// rand_vect_10: create 10 random test vectors
	task rand_vect_10; begin
		// Test Addition
		repeat (10) begin
			a = $random;
			b = $random;
			cin = 0;
			#10;
			if (sum == a + b)
				$display("SUCCESSFUL ADDITION \n a: %d,b: %d,sum: %d,cout: %b", a,b,sum,cout);
			else 
				$fatal("ERROR in ADD\n a: %d, b: %d, sum: %d,cout: %b", a,b,sum,cout);
			#10;
		end
	end endtask
	
endmodule : CarryRippleAdder_tb
