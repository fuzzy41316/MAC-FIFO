module mac_ip
(
input clk,
input rst_n,
input En,
input Clr,
input [7:0] Ain,
input [7:0] Bin,
output logic [23:0] Cout
);

    // Instantiate the multiplier
    logic [23:0] mult_res;
    LPM_MULT_ip mult(.dataa(Ain), .datab(Bin), .result(mult_res));

    // Instantiate the acumulator
    logic [23:0] accum_res;
    logic [23:0] result;
    LPM_ADD_SUB_ip accum(.dataa(accum_res), .datab(mult_res), .result(result));

    always_ff @(posedge clk, negedge rst_n)
    begin
        if (!rst_n) begin
            Cout <= '0;
            accum_res <= '0;
        end
        else if (Clr) begin
            Cout <= '0;
            accum_res <= '0;
        end
        else if (En) begin
            Cout <= result;
            accum_res <= result;
        end
    end
endmodule