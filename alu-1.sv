/*
Mina Gao and Justin Sim
4/18/2024
EE 469 Hussein
Lab 1 & 2
	
	ALU (Arithmetic Logic Unit): A digital circuit that performs arithmetic and logical operations.
   It supports addition, subtraction, bitwise AND, and bitwise OR operations.
    For addition, it takes two input operands and produces a sum.
    For subtraction, it takes two input operands and produces a difference.
    For bitwise AND, it takes two input operands and produces a bitwise AND result.
    For bitwise OR, it takes two input operands and produces a bitwise OR result.
    The ALU operation is determined by a control signal.

*/
module alu#(
	parameter WIDTH = 32
	)(
	input logic [WIDTH-1:0] a,b,
	input logic [1:0] ALUControl,
	output logic [WIDTH-1:0] Result,
	output logic [3:0] ALUFlags
);

	//--------------------------------------
	// Define Intermediate logic
	//--------------------------------------
	logic [WIDTH-1:0] sum;
	logic cout;
	logic [WIDTH-1:0] b_inv;
	bit cra_enable;	// enables carryrippleadder
	
	//-----------------------------------------
	// Execute operations based on CTRL signal
	//-----------------------------------------
	always_comb begin
		case(ALUControl[1])
			0: begin //Addition or Subtraction
			// Selective Inverter :
			// 	'flips' b if ALUControl == 2'b01
				b_inv = b ^ {WIDTH{ALUControl[0]}};			
				Result = sum;
//				$display("Adding or Subtracting");
			end
			
			1: begin // Bitwise AND
				if (ALUControl[0] == 0) begin
					Result = a & b;
				end else begin // Bitwise OR
					Result = a | b;
				end
				
				cout = 0;
			end
		endcase
	end
	//-----------------------------------------
	// Instantiate CarryRippleAdder if Selected
	//-----------------------------------------
		CarryRippleAdder #(WIDTH) cra(.a(a),
												.b(b_inv),
												.sum(sum),
												.cin(ALUControl[0]),
												.cout(cout));	

	//--------------------------------------
	// Assignments for ALUFlags
	//--------------------------------------
	assign ALUFlags[0] =(( (ALUControl == 2'b00) && ((a>0 && b>0 && sum<0) || (a<0 && b<0 && sum>0)) )
							|| ( (ALUControl == 2'b01) && ((a<0 && b>0 && sum>0) || (a>0 && b<0 && sum<0)) ))
							   ? 1 : 0;
	assign ALUFlags[1] = (cout == 1'b1) ? 1: 0;
	assign ALUFlags[2] = (Result == 0) ? 1: 0;
	assign ALUFlags[3] = Result[WIDTH-1] ? 1: 0;
	
endmodule : alu
//---------------------------------------------------------------------------------------
// ALU Testbench
// It does 1 random test for each ALU function
// It provides a full functional coverage for the ALU Testbench
// It provides verification for 2 Overflow cases

module alu_tb;
parameter WIDTH = 32;
    // Inputs
    logic [WIDTH-1:0] a;
    logic [WIDTH-1:0] b;
    logic [1:0] ALUControl;

    // Outputs
    logic [WIDTH-1:0] Result;
    logic [3:0] ALUFlags;

    // Instantiate the ALU module
    alu #(WIDTH) dut(.*);

    // Test cases
    initial begin
        // Test Addition at
        a = $random;
        b = $random;
        ALUControl = 2'b00;
		  #10;
		  if (Result == a + b)
				$display("SUCCESSFUL ADDITION a: %d, b: %d,result: %d at: %t", a,b,Result, $time);
			else 
				$fatal("ERROR in ADD\n a: %d, b: %d,result: %d", a,b,Result);
		  
		  // Test Subtraction
        a = $random;
        b = $random;
        ALUControl = 2'b01;
		  #10;
		  if (Result == a - b)
				$display("SUCCESSFUL SUBTRACTION a: %d, b: %d,result: %d at: %t", a,b,Result, $time);
			else 
				$fatal("ERROR in SUSBTRACT\n a: %d, b: %d,result: %d", a,b,Result);
		  
		  // Test OR
        a = $random;
        b = $random;
        ALUControl = 2'b10;
		  #10;
		  if (Result == (a & b))
				$display("SUCCESSFUL AND a: %d, b: %d,result: %d at: %t", a,b,Result, $time);
			else 
				$fatal("ERROR in AND a: %b, b: %b,result: %b", a,b,Result);
		  
		  // Test OR
        a = $random;
        b = $random;
        ALUControl = 2'b11;
		  #10;
		  if (Result == (a | b))
				$display("SUCCESSFUL OR a: %d b: %d,result: %d at: %t", a,b,Result, $time);
			else 
				$fatal("ERROR in OR a: %b, b: %b,result: %b", a,b,Result);
		  
		  // Test Subtraction OVerflow
         ALUControl = 2'b01;
//			constraint a_c {a<0;};
//			constraint b_c {b>0;};
//			a.randomize() with {a_c;};
//			b.randomize() with {b_c;};
			a = -8;
			b = 8;
		   #10;
		  if ((Result > 0) && (ALUControl[0] != 1)) begin
				$fatal("FAIL TO SEE OVERFLOW: %d-%d!=%d", a,b,Result);
			end
        #10;
//			constraint a_c {a>0;};
//			constraint b_c {b<0;};
			a = $urandom_range(1, 100);
			b = $urandom_range(-100, -1);
		   #10;
		  if ((Result < 0) && (ALUControl[0] != 1)) begin
				$fatal("FAIL TO SEE OVERFLOW: %d-%d!=%d", a,b,Result);
			end
        #10;
		  $display("SUB OVERFLOW DETECTION SUCCESSFUL at: %t", $time);

	 
			// Test Addition OVerflow
         ALUControl = 2'b00;

			a = $urandom_range(1, 100);;
			b = $urandom_range(1, 100);
		   #10;
		  if ((Result < 0) && (ALUControl[0] != 1)) begin
				$fatal("FAIL TO SEE OVERFLOW: %d+%d!=%d", a,b,Result);
			end
        #10;

			a = $urandom_range(-100, -1);
			b = $urandom_range(-100, -1);
		   #10;
		  if ((Result[WIDTH-1] == 0) && (ALUControl[0] != 1)) begin
				$fatal("FAIL TO SEE OVERFLOW at %t",$time);
			end
        #10;
		  $display("ADD OVERFLOW DETECTION SUCCESSFUL at: %t", $time);
        $stop;
		  
	end // initial begin

	 
	
endmodule : alu_tb
