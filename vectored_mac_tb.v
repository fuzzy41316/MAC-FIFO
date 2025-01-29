module vectored_mac_tb #
(
    parameter DATA_WIDTH = 8
);

    // create signals for DUT
    logic [DATA_WIDTH-1:0] Ain;
    logic [DATA_WIDTH-1:0] Bin;
    logic [DATA_WIDTH*3-1:0] Cout;
    logic   clk, 
            rst_n, 
            En, 
            Clr;


    // instantiate DUT
    vectored_mac DUT(.Ain(Ain), .Bin(Bin), .Cout(Cout), .clk(clk), .rst_n(rst_n), .En(En), .Clr(Clr));

    initial begin 
        clk = 0;
        rst_n = 0;
        En = 0;
        Clr = 0;
        Ain = {DATA_WIDTH{1'b0}};
        Bin = {DATA_WIDTH{1'b0}};

        @(posedge clk);
        @(negedge clk) rst_n = 1;

        
    end

    always 
        #5 clk = ~clk;

endmodule