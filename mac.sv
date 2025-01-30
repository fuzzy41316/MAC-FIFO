module mac
(
input clk,
input rst_n,
input En,
input Clr,
input [7:0] Ain,
input [7:0] Bin,
output logic [23:0] Cout
output logic [7:0] Bout;
output logic EnOut;
);

    // Instantiate the multiplier
    logic [23:0] mult_res;
    LPM_MULT_ip mult(.dataa(Ain), .datab(Bin), .result(mult_res));

    // Instantiate the acumulator
    logic [23:0] accum_res;
    logic [23:0] result;
    LPM_ADD_SUB_ip accum(.dataa(accum_res), .datab(mult_res), .result(result));

    // Propogate the B and En;
    always_ff @(posedge clk, negedge rst_n)
    begin
        if (!rst_n) begin
            Cout <= '0;
            accum_res <= '0;
            EnOut <= '0;
            Bout <= '0
        end
        else if (Clr) begin
            Cout <= '0;
            accum_res <= '0;
            EnOut <= '0;
            Bout <= '0;
        end
        else if (En) begin
            Cout <= result;
            accum_res <= result;
            EnOut <= '0;
            Bout <= Bin;
        end
    end
endmodule