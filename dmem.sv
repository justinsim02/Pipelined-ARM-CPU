/*
Justin Sim and Mina Gao
4/18/2024
EE 469 Hussein
Lab 2
*/
/* dmem is a more traditional, albeit very uninteresting, random access 64 word x 32 bit per word memory.
** This module is also written in RTL, and likely strongly resembles your own register file except for a 
** few minor differences. The first is that there is only a single read port, compared to the register 
** file’s two read ports. The other difference is that the dmem is also byte aligned, and therefore 
** discards the bottom two bits of the address when doing a read or write.
*/

// clk - system clock, same as the processor
// wr_en - write enable, allows the wr_data to overwrite the 32 bit word stored in memory[addr]
// addr - the location to which you intend to read or write from
// wr_data - the 32 bit data word which you intend to write into memory
// rd_data - the data currently stored at memory[addr]
module dmem (
    input  logic        clk, wr_en,
    input  logic [31:0] addr,
    input  logic [31:0] wr_data,
    output logic [31:0] rd_data
);

    logic [31:0] memory [63:0];

    // asyncrhnous read
    assign rd_data = memory[addr[31:2]]; // word aligned, drop bottom 2 bits

    // syncrhonous gated write
    always_ff @(posedge clk) begin
        if (wr_en) memory[addr[31:2]] <= wr_data; // word aligned, drop bottom 2 bits
    end

endmodule : dmem