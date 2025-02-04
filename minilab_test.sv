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
        @(negedge CLOCK_50) KEY[0] = 1; 

        $display("Test 2: Filling the B FIFO from memory\n///////////////////////////////////////////////////////////////////");
        fork 
            begin: fillb_to
                repeat(25000) @(posedge CLOCK_50);
                $display("ERR: timeout waiting to fill B_FIFO");
                $stop();
            end: fillb_to
            begin
                @(posedge minilab1.state == 3);     // Wait till we enter the FILLB state
                for (integer i = 0; i < 8; i++) begin
                    @(posedge CLOCK_50);
                    if (minilab1.next_state !== 1) begin
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
        @(negedge CLOCK_50) KEY[0] = 1; 

        $display("Test 3: Filling the A FIFOs from memory\n///////////////////////////////////////////////////////////////////");
        fork 
            begin: filla_to
                repeat(25000) @(posedge CLOCK_50);
                $display("ERR: timeout waiting to fill A_FIFOs");
                $stop();
            end: filla_to
            begin
                @(posedge minilab1.state == 2);     // Wait till we enter the FILLA state

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
        @(negedge CLOCK_50) KEY[0] = 1; 
        
        $display("Test 4: checking Cout values\n///////////////////////////////////////////////////////////////////");
        @(minilab1.state == 6);        
        repeat(6)@(posedge CLOCK_50);

        if (minilab1.cout_reg_00 !== 780) begin
            $display("Expected c00: 780, actual: %d", minilab1.cout_reg_00);
            $stop();
        end
        if (minilab1.cout_reg_01 !== 1356) begin
            $display("Expected c01: 1356, actual: %d", minilab1.cout_reg_01);
            $stop();
        end
        if (minilab1.cout_reg_02 !== 1932) begin
            $display("Expected c02: 1932, actual: %d", minilab1.cout_reg_02);
            $stop();
        end
        if (minilab1.cout_reg_03 !== 2508) begin
            $display("Expected c03: 2508, actual: %d", minilab1.cout_reg_03);
            $stop();
        end
        if (minilab1.cout_reg_04 !== 3084) begin
            $display("Expected c04: 3084, actual: %d", minilab1.cout_reg_04);
            $stop();
        end
        if (minilab1.cout_reg_05 !== 3660) begin
            $display("Expected c05: 3660, actual: %d", minilab1.cout_reg_05);
            $stop();
        end
        if (minilab1.cout_reg_06 !== 4236) begin
            $display("Expected c06: 4236, actual: %d", minilab1.cout_reg_06);
            $stop();
        end
        if (minilab1.cout_reg_07 !== 4812) begin
            $display("Expected c07: 4812, actual: %d", minilab1.cout_reg_07);
            $stop();
        end
        
        $display("  c00:%d", minilab1.cout_reg_00);
        $display("  c01:%d", minilab1.cout_reg_01);
        $display("  c02:%d", minilab1.cout_reg_02);
        $display("  c03:%d", minilab1.cout_reg_03);
        $display("  c04:%d", minilab1.cout_reg_04);
        $display("  c05:%d", minilab1.cout_reg_05);
        $display("  c06:%d", minilab1.cout_reg_06);
        $display("  c07:%d", minilab1.cout_reg_07);
        $display("TEST PASSED\n///////////////////////////////////////////////////////////////////");

        $display("End of test.");
        $stop();
    end
    always #5 CLOCK_50 = ~CLOCK_50;
endmodule