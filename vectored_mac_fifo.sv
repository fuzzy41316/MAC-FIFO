module vectored_mac_fifo #
(
    parameter DATA_WIDTH = 8    // DATA_WIDTH of the vector instantiations (8 entries of 8 bits, 16 entries, etc.)
)
(   
    // SHARED
    input clk,
    // MAC
    input rst_n,
    input [DATA_WIDTH-1:0] En,                              // En turns on MAC (when FIFO is full)
    input Clr,                                              // Asynchronous signal to clear all MAC data
    input [7:0] Ain [DATA_WIDTH-1:0] [DATA_WIDTH-1:0],      // Ain is a vectored entry with A entries of size 8 bits as an input to the MAC provided by a FIFO
    input [7:0] Bin [DATA_WIDTH-1:0],                       // Bin is a vectored entry with B entries of size 8 bits as an input to the MAC provided by a FIFO
    input [23:0] Cout [DATA_WIDTH-1:0],                      // Cout is a vectored entry with C entries of size 24 bits as an output from the MAC
    // FIFO
    input [7:0] datain [DATA_WIDTH-1:0],                    // Memory outputs 64-bit data for each FIFO to store as 8 entries of 8-bits
    input rden,                                             // Read enable for FIFO to pop data
    input wren,                                             // Write enable for FIFO to push data
    input [DATA_WIDTH-1:0] full,                            // Asserted when FIFO is empty
    input [DATA_WIDTH-1:0] empty,                           // Asserted when FIFO is full
    output [7:0] dataout [DATA_WIDTH-1:0]                   // Output of FIFO
);
    genvar k;
    generate
        for (k=0; k < DATA_WIDTH-1; k++) 
        begin
            fifo fifo_inst(
                // Inputs
                .data(datain[k]),
                .rdclk(clk),
                .rdreq(rden),
                .wrclk(clk),
                .wrreq(wren),
                // Outputs
                .q(dataout[k]),
                .rdempty(empty[k]),
                .wrfull(full[k])
            );
        end
    endgenerate

    genvar i;
    genvar j;
    generate
        for (i=0; i < DATA_WIDTH-1; i++)        // Row
        begin
            for (j=0; j < DATA_WIDTH-1; j++)    // Col
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