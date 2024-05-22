/*
Mina Gao and Justin Sim
4/18/2024
EE 469 Hussein
Lab 1 & 2
	
	Full Adder: A digital circuit that performs addition of three bits - two inputs and a carry-in bit.
   It produces a sum and a carry-out bit.
   The sum is the XOR of the inputs and the carry-in.
   The carry-out is generated when two or more of the inputs are high.

*/
module fullAdder (
    input logic a,
    input logic b,
    input logic cin,
    output logic sum,
    output logic cout
);
    assign sum = a ^ b ^ cin;
    assign cout = (a & b) | (cin & (a ^ b));
endmodule 
//-----------------------------------------------------------------------------
// Full Adder Testbench
// It tests each case of a 1 bit adder
// It provides a full functional coverage for the fullAdder

module fullAdder_tb();
	logic a,b,cin,sum,cout;
	
	fullAdder dut(.*);
	
	integer i;
	initial begin
		for(i=0;i<2**3;i++) begin
			{a,b,cin} = i; #10;
		end
	end
endmodule
