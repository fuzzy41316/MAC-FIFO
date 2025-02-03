module vectored_mac_fifo
    // Internal wires
    /* SHARED */
    logic [7:0] datain;
    logic clk, rst_n, clr;

    /* A ARRAY */
    logic [7:0] dataoutA [7:0];      // Row of the 2D A array stored into the FIFO
    logic [7:0] rdenA, wrenA, emptyA, fullA;    // Control signals for each row inserted into 8 FIFOs

    /* B ARRAY */
    logic [7:0] dataoutB [7:0];      // B array 
    logic rdenB, wrenB, emptyB, fullB;          // Control signals for the B FIFO
    logic [7:0] emptyB, fullB;


    // Generate FIFOs for the 8x8 A input
    genvar k;
    generate
        for (k=0; k < 7; k++) 
        begin
            fifo A_fifo(
                // Inputs
                .data(datain),
                .rdclk(clk),
                .rdreq(rdenA[k]),
                .wrclk(clk),
                .wrreq(wrenA[k]),
                // Outputs
                .q(dataoutA[k]),
                .rdempty(emptyA[k]),
                .wrfull(fullA[k])
            );
        end
    endgenerate

    generate
        begin
            fifo B_fifo(
                // Inputs
                .data(Bin),
                .rdclk(clk),
                .rdreq(rdenB),
                .wrclk(clk),
                .wrreq(wrenB),
                // Outputs
                .q(dataoutB),
                .rdempty(emptyB),
                .wrfull(fullB)
            );
        end
    endgenerate

    genvar i;
    genvar j;
    generate
        for (i=0; i < 7; i++)        // Row
        begin
            for (j=0; j < 7; j++)    // Col
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