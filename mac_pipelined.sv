module mac_pipelined
(
input clk,
input rst_n,
input En,
input Clr,
input [7:0] Ain,
input [7:0] Bin,
output logic [23:0] Cout
);
    // Input stage
    logic [7:0]Ain_ff;
    logic [7:0]Bin_ff;
    logic En_ff;

    // Multiplication stage
    logic [15:0] mult_AB;
    logic [15:0] mult_AB_ff;
    logic [23:0] mult_AB_zeroExt;

    // Accumulation stage
    logic [23:0] accum_result;
    logic [23:0] accum_result_ff;

    // Pipeline inputs
    always_ff @(posedge clk, negedge rst_n) begin 
        if (!rst_n) begin
            Ain_ff <= '0;
            Bin_ff <= '0;
            En_ff <= 0;
        end
        else if (En) begin
            Ain_ff <= Ain;
            Bin_ff <= Bin;
            En_ff <= En;
        end
    end

    // Multiplication pipeline stage
    assign mult_AB = Ain_ff * Bin_ff;
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            mult_AB_ff <= '0;
        end
        else if (En_ff) begin
            mult_AB_ff <= mult_AB;
        end
    end

    // Sign extend the output to 24 bits, and accumulate the result
    assign mult_AB_zeroExt = {8'b0, mult_AB};
    assign accum_result = (!rst_n) ? '0 : Clr ? '0 : En ? accum_result + mult_AB_zeroExt : accum_result;

    // Accumulation pipeline stage
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            accum_result_ff <= '0;
        end
        else if (Clr) begin
            accum_result_ff <= '0;
        end
        else if (En_ff) begin
            accum_result_ff <= accum_result;
        end
    end

    // Cout is given the accumulated result unless reset or cleared
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            Cout <= '0;
        else if (Clr)
            Cout <= '0;
        else if (En_ff)
            Cout <= accum_result;
    end

endmodule