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
    minilab1 minilab1(
        .CLOCK2_50(CLOCK2_50),
        .CLOCK3_50(CLOCK3_50),
        .CLOCK4_50(CLOCK4_50),
        .CLOCK_50(CLOCK_50),
        .KEY(KEY),
        .SW(SW)
    );

    initial 
    begin
        CLOCK_50 = 0;
        @(negedge CLOCK_50) KEY[0] = 0;
        repeat(5)@(posedge CLOCK_50);
        @(negedge CLOCK_50) KEY[0] = 1;

        ///////////////////////////////////////////////////////////////////
        // Test series #1. make sure state machine follows high level flow
        ///////////////////////////////////////////////////////////////////
        $display("///////////////////////////////////////////////////////////////////\nTest 1: test that SM fills up all FIFOs\n///////////////////////////////////////////////////////////////////");
        fork
            begin: full_to
                repeat(25000) @(posedge CLOCK_50); 
                $display("ERR: timeout reached waiting for fifos to reach full state");
                $stop(); 
            end: full_to
            begin
                @(posedge minilab1.allFull)
                disable full_to;
            end
        join

        fork
            begin: macdone_to
                repeat(25000) @(posedge CLOCK_50); 
                $display("ERR: timeout reached waiting for mac to finish");
                $stop(); 
            end: macdone_to
            begin
                @(posedge &minilab1.emptyA)
                disable macdone_to;
            end
        join
        $display("TEST PASSED\n///////////////////////////////////////////////////////////////////");


        ///////////////////////////////////////////////////////////////////
        // Test series #2. Filling the B FIFO
        ///////////////////////////////////////////////////////////////////
        CLOCK_50 = 0;
        @(negedge CLOCK_50) KEY[0] = 0;
        repeat(5)@(posedge CLOCK_50);
        @(negedge CLOCK_50) KEY[0] = 1; 

        $display("Test 2: Filling the B FIFO from memory\n///////////////////////////////////////////////////////////////////");
        fork 
            begin: fillb_to
                repeat(25000) @(posedge CLOCK_50);
                $display("ERR: timeout waiting to fill B_FIFO");
                $stop();
            end: fillb_to
            begin
                @(posedge minilab1.state == 2);     // Wait till we enter the FILLB state
                for (integer i = 0; i < 8; i++) begin
                    @(posedge CLOCK_50);
                    if (minilab1.next_state !== 0) begin
                        if (!minilab1.wrenB) begin 
                            $display("ERROR: write enable isn't high when data is ready");
                            repeat(1)@(posedge CLOCK_50);
                            $stop();
                        end
                        if (minilab1.datain != minilab1.readdata_byte[i]) begin
                            $display("ERROR: expected datain from memory: %h, actual: %h for entry %x at time %t", minilab1.readdata_byte[i], minilab1.datain, i+1, $time());
                            repeat(1)@(posedge CLOCK_50);
                            $stop();
                        end
                        $display("  FIFO entry %d filled with value: %h", i+1, minilab1.datain);
                    end
                end
                disable fillb_to;
            end
        join
        $display("TEST PASSED\n///////////////////////////////////////////////////////////////////");

        ///////////////////////////////////////////////////////////////////
        // Test series #3. Filling the A FIFO
        ///////////////////////////////////////////////////////////////////
        CLOCK_50 = 0;
        @(negedge CLOCK_50) KEY[0] = 0;
        repeat(5)@(posedge CLOCK_50);
        @(negedge CLOCK_50) KEY[0] = 1; 

        $display("Test 3: Filling the A FIFOs from memory\n///////////////////////////////////////////////////////////////////");
        fork 
            begin: filla_to
                repeat(25000) @(posedge CLOCK_50);
                $display("ERR: timeout waiting to fill A_FIFOs");
                $stop();
            end: filla_to
            begin
                @(posedge minilab1.state == 1);     // Wait till we enter the FILLA state

                for (int i = 0; i < 8; i++) begin
                    @(posedge |minilab1.wrenA) begin
                        for (int j = 0; j < 8; j++) begin
                            @(posedge CLOCK_50);
                            if (!minilab1.fullA[i]) begin // ignore when the FIFO becomes full
                                if (!minilab1.wrenA[i]) begin
                                    $display("ERROR: write enable isn't high when data is ready at time %t", $time);
                                    repeat(1)@(posedge CLOCK_50);
                                    $stop();
                                end
                                if (minilab1.datain != minilab1.readdata_byte[j]) begin
                                    $display("ERROR: expected datain from memory: %h, actual: %h for entry %x at time %t", minilab1.readdata_byte[j], minilab1.datain, j+1, $time());
                                    repeat(1)@(posedge CLOCK_50);
                                    $stop();
                                end
                                $display("  FIFO %d, entry %d filled with value: %h", i+1, j+1, minilab1.datain);
                            end
                        end
                    end
                end
                disable filla_to;
            end
        join
        $display("TEST PASSED\n///////////////////////////////////////////////////////////////////");
        
        ///////////////////////////////////////////////////////////////////
        // Test series #4. Popping the B FIFO & checking Cout from the MAC
        ///////////////////////////////////////////////////////////////////
        CLOCK_50 = 0;
        @(negedge CLOCK_50) KEY[0] = 0;
        repeat(5)@(posedge CLOCK_50);
        @(negedge CLOCK_50) KEY[0] = 1; 
        
        $display("Test 4: checking Cout values\n///////////////////////////////////////////////////////////////////");
        @(minilab1.state == 5) begin
            if (minilab1.cout_reg[0] !== 588) begin
                $display("Expected c00: 588, actual: %d", minilab1.cout_reg[0]);
                $stop();
            end
            if (minilab1.cout_reg[1] !== 1036) begin
                $display("Expected c01: 1036, actual: %d", minilab1.cout_reg[1]);
                $stop();
            end
            if (minilab1.cout_reg[2] !== 1484) begin
                $display("Expected c02: 1484, actual: %d", minilab1.cout_reg[2]);
                $stop();
            end
            if (minilab1.cout_reg[3] !== 1932) begin
                $display("Expected c03: 1932, actual: %d", minilab1.cout_reg[3]);
                $stop();
            end
            if (minilab1.cout_reg[4] !== 2380) begin
                $display("Expected c04: 2380, actual: %d", minilab1.cout_reg[4]);
                $stop();
            end
            if (minilab1.cout_reg[5] !== 2828) begin
                $display("Expected c05: 2828, actual: %d", minilab1.cout_reg[5]);
                $stop();
            end
            if (minilab1.cout_reg[6] !== 3276) begin
                $display("Expected c06: 3276, actual: %d", minilab1.cout_reg[6]);
                $stop();
            end
            if (minilab1.cout_reg[7] !== 3724) begin
                $display("Expected c07: 3724, actual: %d", minilab1.cout_reg[7]);
                $stop();
            end
        end

        for (int i = 0; i < 8; i++) 
            $display("  Cout%d:%d", i, minilab1.cout_reg[i]);
        $display("TEST PASSED\n///////////////////////////////////////////////////////////////////");

        $display("End of test.");
        $stop();
    end
    always #5 CLOCK_50 = ~CLOCK_50;
endmodule