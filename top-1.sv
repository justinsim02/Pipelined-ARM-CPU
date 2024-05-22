/*
Justin Sim and Mina Gao
4/28/2024
EE 469 Hussein
Lab 3
*/
/* top is a structurally made toplevel module. It consists of 3 instantiations, as well as the signals that link them. 
** It is almost totally self-contained, with no outputs and two system inputs: clk and rst. clk represents the clock 
** the system runs on, with one instruction being read and executed every cycle. rst is the system reset and should 
** be run for at least a cycle when simulating the system.
*/

// clk - system clock
// rst - system reset. Technically unnecessary
module top(
    input logic clk, rst
);
    
    // processor io signals
    logic [31:0] InstrF, InstrD;
    logic [31:0] ReadDataM;
    logic [31:0] WriteDataE;
    logic [31:0] PC, ALUResultE;
	 logic [31:0] ALUOutM, WriteDataM;	// From Execute Pipeline
	 logic [31:0] ALUOutW, ReadDataW;	// For Writebak Pipeline
    logic        MemWrite;
	 
	 // Pipeline Registers for Writeback
	 	always_ff @(posedge clk) begin
			if (!rst) begin
				ReadDataW <= ReadDataM;
				ALUOutW <= ALUOutM;
			end
		end
	 
    // our single cycle arm processor
    arm processor (
        .clk        (clk        ), 
        .rst        (rst        ),
        .InstrF      (InstrF     ),
        .ReadDataW  (ReadDataW  ),
        .WriteDataE (WriteDataE ), 
        .PCF         (PC         ), 	// <- change
        .ALUResultE (ALUResultE ),
        .MemWrite   (MemWrite   ),
		  .ALUOutM	  (ALUOutM    ),
		  .WriteDataM (WriteDataM )
    );

    // instruction memory
    // contained machine code instructions which instruct processor on which operations to make
    // effectively a rom because our processor cannot write to it
    imem imemory (
        .addr   (PC     ),
        .instr  (InstrF )
    );

    // data memory
    // containes data accessible by the processor through ldr and str commands
    dmem dmemory (
        .clk     (clk       ), 
        .wr_en   (MemWrite  ),
        .addr    (ALUOutM   ),
        .wr_data (WriteDataM),
        .rd_data (ReadDataM )
    );


endmodule : top