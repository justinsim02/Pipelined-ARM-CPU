/*
Justin Sim and Mina Gao
4/18/2024
EE 469 Hussein
Lab 2
*/
/* imem is the read only, 64 word x 32 bit per word instruction memory for our processor. 
** Its module is written in RTL, and it strongly resembles a ROM (read only memory) or LUT 
** (look up table). This memory has no clock, and cannot be written to, but rather it 
** asynchronously reads out the word stored in its memory as soon as an address is given. 
** The address and memory are byte aligned, meaning that the bottom two bits are discarded 
** when looking for the word. One important line to note is the
**      Initial $readmemb(“memfile.dat”, memory);
** which determines the contents of the memory when the system is initialized. You will alter 
** this line to use programs given to you as a part of this lab.
*/

// addr - 32 bit address to determine the instruction to return. Note not all 32 bits are used since this
//        memory only has 64 words
// instr - 32 bit instruction to be sent to the processor
module imem(
    input  logic [31:0] addr,
    output logic [31:0] instr
);
    logic [31:0] memory [63:0];

    // modify the name and potentially directory prefix of the file within to load the correct program and preprocessing
    initial $readmemb("memfile2.dat", memory);


    assign instr = memory[addr[31:2]]; // word aligned, drops bottom 2 bits

endmodule : imem