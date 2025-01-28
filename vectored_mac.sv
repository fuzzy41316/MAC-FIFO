module vectored_mac #
(
    parameter DEPTH = 8    // Depth of the vector instantiations (8 entries of 8 bits, 16 entries, etc.)
)
(
    input clk,
    input rst_n,
    input En [DEPTH-1:0],
    input Clr,
    input [7:0] Ain [DEPTH-1:0] [DEPTH-1:0],    // Ain is a vectored entry with A entries of size 8 bits as an input to the MAC provided by a FIFO
    input [7:0] Bin [DEPTH-1:0],                // Bin is a vectored entry with B entries of size 8 bits as an input to the MAC provided by a FIFO
    output [23:0] Cout [DEPTH-1:0]              // Cout is a vectored entry with C entries of size 24 bits as an output from the MAC
);
    // Only begin MAC operations after ALL FIFOs are full
    genvar i;
    genvar j;
    generate
        for (i=0; i < DEPTH-1; i++)        // Row
        begin
            for (j=0; j < DEPTH-1; j++)    // Col
            begin
                mac mac_inst(
                    // Inputs
                    .clk(clk),
                    .rst_n(rst_n),
                    .En(En[i]),
                    .Clr(Clr),
                    .Ain(Ain[i][j]),
                    .Bin(Bin[i]),
                    // Outputs
                    .Cout(Cout[i]));  
            end
        end
    endgenerate
endmodule