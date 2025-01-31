`timescale 1 ps / 1 ps
module minilab_test();
	//////////// CLOCK //////////
	logic 		          		CLOCK2_50;
	logic 		          		CLOCK3_50;
	logic 		          		CLOCK4_50;
	logic 		          		CLOCK_50;
	//////////// KEY //////////
	logic 		     [3:0]		KEY;

	//////////// SW //////////
	logic 		     [9:0]		SW;

    // Instantiate the DUT
    minilab1_2 minilab1(
        .CLOCK2_50(CLOCK2_50),
        .CLOCK3_50(CLOCK3_50),
        .CLOCK4_50(CLOCK4_50),
        .CLOCK_50(CLOCK_50),
        .KEY(KEY),
        .SW(SW)
    );

    initial 
    begin
        $display("Resetting the minilab1 DUT");
        CLOCK_50 = 0;
        @(negedge CLOCK_50) KEY[0] = 0;
        repeat(5)@(posedge CLOCK_50);
        @(negedge CLOCK_50) KEY[0] = 1;

        

        @(minilab1.state == 5)
        begin
            $display("End of test.");
            $stop();
        end




    
    end
    always #5 CLOCK_50 = ~CLOCK_50;
endmodule