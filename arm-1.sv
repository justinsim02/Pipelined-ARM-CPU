/*
Justin Sim and Mina Gao
4/28/2024
EE 469 Hussein
Lab 3

arm 5 stage pipelined processor
arm is the spotlight of the show and contains the bulk of the datapath and control logic. 
This module is split into two parts, the datapath and control. 
It simulates a simplified ARM processor with a 5-stage pipeline with 
instruction fetch, decode, execute, memory access, and write back stages. 
*/

// clk - system clock
// rst - system reset
// InstrF - fetched 32 bit instruction from instruction memory
// ReadDataW - data read out of the register pipelining memory and writeback
// WriteDataE - data to be written to the dmem
// MemWrite - write enable to allowed WriteData to overwrite an existing dmem word
// PC - the current program count value, goes to imem to fetch instruciton
// ALUResultE - result of the ALU operation during EXECUTION, sent as address to the dmem

module arm (
    input  logic        clk, rst,
    input  logic [31:0] InstrF,
    input  logic [31:0] ReadDataW,
	 
    output logic [31:0] WriteDataE, 
    output logic [31:0] PCF, ALUResultE,
    output logic        MemWrite,
    output logic [31:0] ALUOutM, WriteDataM	// From Execute Pipeline
);

    // datapath buses and signals
    logic [31:0] PCPrime, PCPlus4, PCPlus8, PCPlus8_temp, PC;	// pc signals, adding PCF

//	 logic [ 3:0] RA1, RA2;                  	 	// regfile input addresses
//    logic [31:0] RD1, RD2;                  		// raw regfile outputs
    logic [ 3:0] ALUFlags;                  		// alu combinational flag outputs
    logic [31:0] ExtImm, SrcA, SrcB;   			// immediate and alu inputs 
    logic [31:0] ResultW;                   		// computed or fetched value to be written into regfile or pc
    logic [ 3:0] FlagsReg;		    		// register to store the flags from the most recent CMP command
    logic [31:0] RA1E, RA2E;  				// regfile inputs addresses
    logic [31:0] ALUOutW;

    // control signals
    // adding FlagWrite to control if loading the FlagsReg, control signal for the decoder 
    logic PCSrc, MemtoReg, ALUSrc, RegWrite, FlagWrite;
    logic [1:0] RegSrc, ImmSrc, ALUControl;
	 logic [5:0] Funct;
	 logic NoWrite = 0;

    // fetch stage
    logic StallF;

    // decode stage
    logic PCSrcD, RegWriteD, MemtoRegD, MemWriteD, BranchD=0, ALUSrcD, FlagWriteD, ImmSrcD, RegSrcD, StallD, FlushD;
    logic [1:0] ALUControlD;
	 logic [3:0] RA1D, RA2D;
	 logic [31:0] InstrD;
    logic [31:0] RD1D, RD2D;

    // execute stage
    logic PCSrcE, RegWriteE, MemtoRegE, MemWriteE, BranchE, ALUSrcE, FlushE;
    logic [3:0] CondE, FlagsE, Flags, WA3E;
    logic [1:0] ALUControlE, FlagWriteE;
    logic [1:0] ForwardAE, ForwardBE;
    logic CondExE, BranchTakenE;
    logic [31:0] RD1E, RD2E, ExtImmE;                  // decoder pipeline outputs


    // memory stage
    logic PCSrcM, RegWriteM, MemtoRegM, MemWriteM;
    logic [3:0] WA3M;

    // writeback (back to register file WD3)
    logic PCSrcW, RegWriteW, MemtoRegW;
    logic [3:0] WA3W;
	 
	 // Additional Floating Logic
	 logic [3:0] FlagsPrime;
	 logic PCS;
	 
	 // PCSrc Control Unit Logic
	 assign PCS = ((InstrD[15:12] == 'd15) & RegWriteD) | BranchD;
	 assign PCSrcD = CondExE ? PCS : 0;
	 


    /* The datapath consists of a PC as well as a series of muxes to make decisions about which data words to pass forward and operate on. It is 
    ** noticeably missing the register file and alu, which you will fill in using the modules made in lab 1. To correctly match up signals to the 
    ** ports of the register file and alu take some time to study and understand the logic and flow of the datapath.
    */
    //-------------------------------------------------------------------------------
    //                                      DATAPATH
    //-------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						// FETCH STAGE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // update PCPrime according to branch condition 
	assign PC = PCSrcW ? ResultW : PCPlus4;
	assign PCPrime = BranchTakenE ? ALUResultE : PC;
   assign PCPlus4 = PCF + 'd4;                  // default value to access next instruction
	 // PCPlus8
	always_ff @(posedge clk) begin
		PCPlus8_temp <= PCPlus4 + 'd4; end
			always_ff @(posedge clk) begin
		PCPlus8 <= PCPlus8_temp; end
	 	
    // update PCF based on the stall and reset
    always_ff @(posedge clk) begin
        if (rst) 		PCF <= '0;
	else if (StallF)	PCF <= PCF;
        else			PCF <= PCPrime;
    end
	 	 
	 // FETCH registers 
    always_ff @(posedge clk) begin
		if (FlushD)		InstrD <= '0;
		else if (StallD)	InstrD <= InstrD;  // holds the old value
		else			InstrD <= InstrF;
	end
	 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								// DECODE STAGE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	  
	 
    // determine the register addresses based on control signals
    // RegSrc[0] is set if doing a branch instruction
    // RefSrc[1] is set when doing memory instructions
    assign RA1D = RegSrc[0] ? 4'd15         : InstrD[19:16];
    assign RA2D = RegSrc[1] ? InstrD[15:12] : InstrD[ 3: 0];

    // -------------------------------------------------------
    // Instantiate register
    // Inputs: clk, RegWriteW, ResultW, WA3W, RA1, RA2
    // Outputs: , RD2D
    // -------------------------------------------------------
    reg_file u_reg_file (
        .clk       (!clk), 
        .wr_en     (RegWriteW),
        .write_data(ResultW),
        .write_addr(WA3W),
        .read_addr1(RA1D), 
        .read_addr2(RA2D),
        .read_data1(RD1D), 
        .read_data2(RD2D)
    );

    // two muxes, put together into an always_comb for clarity
    // determines which set of instruction bits are used for the immediate
    always_comb begin
        if      (ImmSrc == 'b00) ExtImm = {{24{InstrD[7]}},InstrD[7:0]};          // 8 bit immediate - reg operations
        else if (ImmSrc == 'b01) ExtImm = {20'b0, InstrD[11:0]};                  // 12 bit immediate - mem operations
        else                     ExtImm = {{6{InstrD[23]}}, InstrD[23:0], 2'b00}; // 24 bit immediate - branch operation
    end
	 
    //------------------------------
    // DECODER PIPE
    //------------------------------
		
    always_ff @(posedge clk) begin
	if (~FlushE) begin
		PCSrcE <= PCSrcD;
        	RegWriteE <= RegWriteD;
		MemtoRegE <= MemtoRegD;
        	MemWriteE <= MemWriteD;
		ALUControlE <= ALUControlD;
		RD1E <= RD1D;
        	RD2E <= RD2D;
        	RA1E <= RA1D;
        	RA2E <= RA2D; 
		WA3E <= InstrD[15:12];
		BranchE <= BranchD;
        	ALUSrcE <= ALUSrcD;
		FlagWriteE <= FlagWriteD;
        	CondE <= InstrD[31:28];
        	FlagsE <= FlagsPrime;
		ExtImmE <= ExtImm;
	end else begin
		PCSrcE <= '0;
        	RegWriteE <= '0;
		MemtoRegE <= '0;
        	MemWriteE <= '0;
		ALUControlE <= '0;
		RD1E <= '0;
        	RD2E <= '0;
        	RA1E <= '0;
        	RA2E <= '0; 
		WA3E <= '0;
		BranchE <= '0;
        	ALUSrcE <= '0;
		FlagWriteE <= '0;
        	CondE <= '0;
        	FlagsE <= '0;
		ExtImmE <= '0;
//		$display("FlushE is HIGH at time %d", $time);
	end
    end
	
	
	 // Additional Control logic assignment
	 assign Funct = InstrD[25:20];	
	 
	 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						// EXECUTE STAGE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // ---------------------------------------------------------------------
    // Data forwarding logic
    // check if execute stage register matches Memory stage register
    // check if execute stage register matches Writeback stage register
    // ---------------------------------------------------------------------
    logic Match_1E_M, Match_2E_M, Match_1E_W, Match_2E_W;
    assign Match_1E_M = (RA1E == WA3M);
    assign Match_2E_M = (RA2E == WA3M);
	 
    assign Match_1E_W = (RA1E == WA3W);
    assign Match_2E_W = (RA2E == WA3W);

    
    always_comb begin 
		// ForwardAE
		if (Match_1E_M & RegWriteM)		ForwardAE = 2'b10;
		else if (Match_1E_W & RegWriteW)	ForwardAE = 2'b01;
		else 					ForwardAE = 2'b00;
    
    	// ForwardBE
		if (Match_2E_M & RegWriteM)		ForwardBE = 2'b10;
		else if (Match_2E_W & RegWriteW)	ForwardBE = 2'b01;
		else 					ForwardBE = 2'b00;
    end
    
    // ---------------------------------------------------------------------
    //Stalling and flushing logic
    // Match_12D_E: check if either source register in the Decode stage the same
    //		    as the one being written in the Execute stage
    // ldrStall: if a LDR in the Execute stage AND Match_12D_E
    // Control Stalling Logic
    // PCWrPendingF: if write to PC in Decode, Execute or Memory
    // StallF: if PCWrPendingF
    // FlushD: if PCWrPendingF OR PC is written in Writeback OR branch is taken
    // FlushE: if branch is taken
    // StallD: if ldrStall
    // ---------------------------------------------------------------------
    logic Match_12D_E, PCWrPendingF, ldrStall;
    assign Match_12D_E = ((RA1D == WA3E) || (RA2D == WA3E)) ? 1 : 0;
    assign ldrStall = Match_12D_E & MemtoRegE;
    assign PCWrPendingF = PCSrcD | PCSrcE | PCSrcM;
    assign StallF = ldrStall | PCWrPendingF;
    assign FlushD = PCWrPendingF | PCSrcW | BranchTakenE;
    assign FlushE = ldrStall | BranchTakenE;
    assign StallD = ldrStall;
    
    // WriteDataE and SrcAE are outputs of the register file, wheras SrcEB is chosen between reg file output and the immediate
    // Forwarding mux for SrcAE, SrcBE, and WriteDataE
    logic [31:0] SrcAE, SrcBE;
    always_comb begin
	case (ForwardAE) 
		2'b00:		SrcAE = (RA1E == 'd15) ? PCPlus8 : RD1E; // substitute the 15th regfile register for PC 
		2'b01:		SrcAE = ResultW;
		2'b10:		SrcAE = ALUOutM; // previous instruction
		default:	SrcAE = 0;
	endcase
	case (ForwardBE)
		2'b00:		WriteDataE = (RA2E == 'd15) ? PCPlus8 : RD2E; // substitute the 15th regfile register for PC
		2'b01:		WriteDataE = ResultW;
		2'b10:		WriteDataE = ALUOutM; // previous instructionS
		default:	WriteDataE = 0; 
	endcase
    end
    assign SrcBE = ALUSrcE ? ExtImmE : WriteDataE;	// determine alu operand to be either from reg file or from immediate

    // --------------------------------------
    // Instantiate ALU
    // Inputs: a, b, ALUControl
    // Outputs: ResultW, ALUFlags
    // --------------------------------------
    alu u_alu (
        .a          (SrcAE), 
        .b          (SrcBE),
        .ALUControl (ALUControlE),
        .Result     (ALUResultE),
        .ALUFlags   (ALUFlags)
    );


	 always_ff@(posedge clk) begin
		if (FlagWriteE[0]) FlagsPrime[1:0] <= ALUFlags[1:0];
		if (FlagWriteE[1]) FlagsPrime[3:2] <= ALUFlags[3:2];
	 end
	 
	 //--------------------------------
	 // Cond Unit
	 //------------------------------
	 
	 // Cond Logic
	 logic N;
    logic Z;
    logic C;
    logic V;
    assign N = ALUFlags[3];
    assign Z = ALUFlags[2];
    assign C = ALUFlags[1];
    assign V = ALUFlags[0];
	 // Cond Assignment
	 // CondExE = instruction executed
    always_comb begin
	case(CondE) 
		4'b0000:	CondExE = Z;
		4'b0001:	CondExE = !Z;
		4'b0010:	CondExE = C;
		4'b0011:	CondExE = !C;
		4'b0100:	CondExE = N;
		4'b0101:	CondExE = !N;
		4'b0110:	CondExE = V;
		4'b0111:	CondExE = !V;
		4'b1000:	CondExE = !Z & C;
		4'b1001:	CondExE = Z | !C;
		4'b1010:	CondExE = !(N ^ V);
		4'b1011:	CondExE = N ^ V;
		4'b1100:	CondExE = !Z & !(N ^ V);
		4'b1101:	CondExE = Z | (N ^ V);
		4'b1110:	CondExE = 1;
		default: 	CondExE = 0;
	endcase
    end
	
	 
    // Execute -> Memory Pipeline
    always_ff @(posedge clk) begin
	 
		MemtoRegM <= MemtoRegE;
		WA3M <= WA3E;
		ALUOutM <= ALUResultE;
		WriteDataM <= WriteDataE;
		
		// AND CondExE
		BranchTakenE <= BranchE & CondExE;
		RegWriteM <= RegWriteE & CondExE & ~NoWrite;
      MemWriteM <= MemWriteE & CondExE;
		PCSrcM <= PCSrcE & CondExE;

    end

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						// WRITEBACK
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // memory -> Writeback Pipelin
    always_ff @(posedge clk) begin
		PCSrcW <= PCSrcM;
		RegWriteW <= RegWriteM;
			  MemtoRegW <= MemtoRegM;
		WA3W <= WA3M;
		ALUOutW <= ALUOutM;
    end
	 

    // determine the result to run back to PC or the register file based on whether we used a memory instruction
    assign ResultW = MemtoRegW ? ReadDataW : ALUOutW;    // determine whether final writeback result is from dmemory or alu
    
	 
//	 	initial begin
//		if (StallF == StallD == FlushE == ldrStall) $display("Stall Control Logic is GOOD!");
//		else $display("Stall ERROR! at time %d", $time);
//			
//		if (ldrStall == (Match_12D_E & MemtoRegE)) $display("ldrstall logic is GOOD!");
//			else $display("ldrstall ERROR! at time %d", $time);
//	 end
//	 

    /* The control conists of a large decoder, which evaluates the top bits of the instruction and produces the control bits 
    ** which become the select bits and write enables of the system. The write enables (RegWrite, MemWrite and PCSrc) are 
    ** especially important because they are represefntative of your processors current state. 
    */
    //-------------------------------------------------------------------------------
    //                                      CONTROL
    //-------------------------------------------------------------------------------
    
	 // Control Unit
    always_comb begin
	casez (InstrD[31:20])
	
		 //------------------
       // ADD (Imm or Reg)
		 //------------------
	    12'b111000?01000 : begin
		 
					 // Main Decoder
					 MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25];
					 ImmSrc   = Funct[5] ? 'b00 : 'bXX;
                RegWriteD = 1;
                RegSrc   = Funct[5] ? 'bX0 : 'b00;
					 
					 // ALU Decoder
                ALUControlD = 'b00;
					 FlagWriteD = Funct[0] ? 2'b11 : 2'b00;
					 
					 // $display("ADD at time %d", $time); // uncomment for verification
       end // ADD
		 
	    //------------------
       // SUB (Imm or Reg)
		 //------------------
	    12'b111000?00100 : begin
		 
					 // Main Decoder
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25];
					 ImmSrc   = Funct[5] ? 'b00 : 'bXX;
                RegWriteD = 1;
                RegSrc   = Funct[5] ? 'bX0 : 'b00;
					 
					 // ALU Decoder
                ALUControlD = 'b01;
					 FlagWriteD = Funct[0] ? 2'b11 : 2'b00;
					 
					 // $display("SUB at time %d", $time); // uncomment for verification
       end // SUB
		 
		 //------------------
		 // AND
		 //------------------
	    12'b111000000000 : begin
		 
					 // Main Decoder
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25]; 
					 ImmSrc   = Funct[5] ? 'b00 : 'bXX;
                RegWriteD = 1;
                RegSrc   = Funct[5] ? 'bX0 : 'b00;
					 
					 // ALU Decoder
                ALUControlD = 'b10;  
					 FlagWriteD = Funct[0] ? 2'b10 : 2'b00;

//					 $display("AND at time %d", $time); // uncomment for veri..
	    end // AND
	    
		 //------------------
       // ORR
		 //------------------
	    12'b111000011000 : begin
		 
					 // Main Decoder
                MemtoRegD = 0; 
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25]; 
					 ImmSrc   = Funct[5] ? 'b00 : 'bXX;
                RegWriteD = 1;
                RegSrc   = Funct[5] ? 'bX0 : 'b00;
					 
					 // ALU Decoder
                ALUControlD = 'b11; 
					 FlagWriteD = Funct[0] ? 2'b10 : 2'b00;

//					 $display("OR at time %d", $time);
	    end // ORR
		 
		 //------------------
       // LDR
		 //------------------
	    12'b111001011001 : begin
		 
					 // Main Decoder
                MemtoRegD = 1; 
                MemWriteD = 0; 
                ALUSrcD   = 1;
                RegWriteD = 1;
                RegSrc   = 'bX0; 
                ImmSrc   = 'b01; 
					 
					 // ALU Decoder
                ALUControlD = 'b00; 
					 FlagWriteD = 0;
		
//					 $display("LDR at time %d", $time);
	    end
		 
		 //------------------
       // STR
		 //------------------
	    12'b111001011000 : begin
		 
					 // Main Decoder
                MemtoRegD = 'bX; // doesn't matter
                MemWriteD = 1; 
                ALUSrcD   = 1;
                RegWriteD = 0;
                RegSrc   = 'b10;    // msb doesn't matter
                ImmSrc   = 'b01; 
					 
					 // ALU Decoder
                ALUControlD = 'b00;  // do an add
					 FlagWriteD = 0;
					 
//					 $display("STR at time %d", $time);
	    end
		 
		 //------------------
       // B
		 //------------------
	    12'b????1010???? : begin
			case (InstrD[31:28])
				// B, BEQ, BNE, BGE, BGT, BLE, BLT
				4'b1110, 4'b0000, 4'b0001, 4'b1010, 4'b1100, 4'b1101, 4'b1011: begin
				
					   // Main Decoder
						BranchD = 1;
						MemtoRegD = 0;
                	MemWriteD = 0; 
                	ALUSrcD   = 1;
              		RegWriteD = 0;
              		RegSrc   = 'bX1;
           			ImmSrc   = 'b10;
							
						// ALU Decoder
              		ALUControlD = 'b00;
						FlagWriteD = 0;
						
//						$display("B at time %d", $time);
            		end
			endcase
		end // B
		
		 //------------------
 	    // CMP
		 //------------------
	    12'b111000?00101 : begin
		 
					 // Main Decoder
                MemtoRegD = 0;
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25];
					 RegWriteD = 0;
                RegSrc   = 'b00;
                ImmSrc   = 'b00; 
					 
					 // ALU Decoder
                ALUControlD = 'b01; 
					 FlagWriteD = 2'b11;
					 
					 // NoWrite
					 NoWrite = 1;
					
//					$display("CMP at time %d", $time);
            end

		 //------------------
		 // Default Skin
		 //------------------
	    default: begin
                MemtoRegD = 0;
                MemWriteD = 0; 
                ALUSrcD   = InstrD[25];
					RegWriteD = 1;
               RegSrc   = 'b00;
               ImmSrc   = 'b00; 
               ALUControlD = 'b00;  // do an add
					FlagWriteD = 0;
					BranchD = 0;
	    end
     endcase
    end // always_comb
 
endmodule : arm