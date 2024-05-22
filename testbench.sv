/*
Justin Sim and Mina Gao
4/18/2024
EE 469 Hussein
Lab 2
*/
/* testbench is a simulation module which simply instantiates the processor system and runs 50 cycles 
** of instructions before terminating. At termination, specific register file values are checked to
** verify the processors’ ability to execute the implemented instructions.
*/
module testbench();

    // system signals
    logic clk, rst;

    // generate clock with 100ps clk period 
    initial begin
        clk = '1;
        forever #50 clk = ~clk;
    end

    // processor instantion. Within is the processor as well as imem and dmem
    top cpu (.clk(clk), .rst(rst));

    initial begin
        // start with a basic reset
        rst = 1; @(posedge clk);
        rst <= 0; @(posedge clk);

        // repeat for 50 cycles. Not all 50 are necessary, however a loop at the end of the program will keep anything weird from happening
        repeat(50) @(posedge clk);

        // basic checking to ensure the right final answer is achieved. These DO NOT prove your system works. A more careful look at your 
        // simulation and code will be made.

        // task 1:
//        assert(cpu.processor.u_reg_file.mem[8] == 32'hb) $display("Task 1 Passed R8");
//        else                                                 $display("Task 1 Failed R8");
//		  assert(cpu.processor.u_reg_file.mem[7] == 32'hb) $display("Task 1 Passed R7");
//        else                                                 $display("Task 1 Failed R7");
//		  assert(cpu.processor.u_reg_file.mem[6] == 32'hf) $display("Task 1 Passed R6");
//        else                                                 $display("Task 1 Failed R6");
//		  assert(cpu.processor.u_reg_file.mem[5] == 32'hb) $display("Task 1 Passed R5");
//        else                                                 $display("Task 1 Failed R5");
//		  assert(cpu.processor.u_reg_file.mem[4] == 32'h7) $display("Task 1 Passed R4");
//        else                                                 $display("Task 1 Failed R4");
//		  assert(cpu.processor.u_reg_file.mem[3] == 32'h12) $display("Task 1 Passed R3");
//        else                                                 $display("Task 1 Failed R3");
//		  assert(cpu.processor.u_reg_file.mem[2] == 32'ha) $display("Task 1 Passed R2");
//        else                                                 $display("Task 1 Failed R2");
//		  assert(cpu.processor.u_reg_file.mem[1] == 32'h0) $display("Task 1 Passed R1");
//        else                                                 $display("Task 1 Failed R1");
//		  assert(cpu.processor.u_reg_file.mem[0] == 32'h8) $display("Task 1 Passed R0");
//        else                                                 $display("Task 1 Failed R0");

        // task 2:
        assert(cpu.processor.u_reg_file.mem[8] == 32'd1)  $display("Task 2 Passed");
        else                                                 $display("Task 2 Failed");
		  // verify skips
		  assert(cpu.processor.PC == 32'd40 || cpu.processor.PC == 32'd40) $display("Task 2 Failed SKIP at PC 36");
		  else																				 $display("Task 2 Passed SKIP at PC 36");
		  assert(cpu.processor.PC == 32'd68) 										 $display("Task 2 Failed SKIP at PC 64");
		  else																				 $display("Task 2 Passed SKIP at PC 64");
		  assert(cpu.processor.PC == 32'd88) 										 $display("Task 2 Failed SKIP at PC 84");
		  else																				 $display("Task 2 Passed SKIP at PC 84");
		  assert(cpu.processor.PC == 32'd108) 										 $display("Task 2 Failed SKIP at PC 104");
		  else																				 $display("Task 2 Passed SKIP at PC 104");
		  																				 
		  // verify end loops
		  repeat(5) assert(cpu.processor.PC == 32'd116) 						 $display("Task 2 Passed LOOP at PC 116 !");
		  else																				 $display("Task 2 Failed LOOP at PC 116");
        $stop;
    end


endmodule 