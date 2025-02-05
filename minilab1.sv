module minilab1(
	//////////// CLOCK //////////
	input 		          		CLOCK2_50,
	input 		          		CLOCK3_50,
	input 		          		CLOCK4_50,
	input 		          		CLOCK_50,

	//////////// SEG7 //////////
	output	reg	     [6:0]		HEX0,
	output	reg	     [6:0]		HEX1,
	output	reg	     [6:0]		HEX2,
	output	reg	     [6:0]		HEX3,
	output	reg	     [6:0]		HEX4,
	output	reg	     [6:0]		HEX5,
	
	//////////// LED //////////
	output		     [9:0]		LEDR,

	//////////// KEY //////////
	input 		     [3:0]		KEY,

	//////////// SW //////////
	input 		     [9:0]		SW
);
    parameter HEX_0 = 7'b1000000;		// zero
    parameter HEX_1 = 7'b1111001;		// one
    parameter HEX_2 = 7'b0100100;		// two
    parameter HEX_3 = 7'b0110000;		// three
    parameter HEX_4 = 7'b0011001;		// four
    parameter HEX_5 = 7'b0010010;		// five
    parameter HEX_6 = 7'b0000010;		// six
    parameter HEX_7 = 7'b1111000;		// seven
    parameter HEX_8 = 7'b0000000;		// eight
    parameter HEX_9 = 7'b0011000;		// nine
    parameter HEX_10 = 7'b0001000;	// ten
    parameter HEX_11 = 7'b0000011;	// eleven
    parameter HEX_12 = 7'b1000110;	// twelve
    parameter HEX_13 = 7'b0100001;	// thirteen
    parameter HEX_14 = 7'b0000110;	// fourteen
    parameter HEX_15 = 7'b0001110;	// fifteen
    parameter OFF   = 7'b1111111;		// all off


    // Internal wires
    /* SHARED */
    logic [7:0] datain;
    logic rst_n, Clr, aclr;
    logic [63:0] readdata_ff; 
    logic readdatavalid_ff;
    logic waitrequest_ff;
    logic Exec;


    /* MEMORY */
    reg [31:0] address;
    logic [63:0] readdata;
    logic readdatavalid, read, waitrequest;
    logic [7:0] readdata_byte [7:0];
    assign readdata_byte[0] = readdata_ff[63:56];
    assign readdata_byte[1] = readdata_ff[55:48];
    assign readdata_byte[2] = readdata_ff[47:40];
    assign readdata_byte[3] = readdata_ff[39:32];
    assign readdata_byte[4] = readdata_ff[31:24];
    assign readdata_byte[5] = readdata_ff[23:16];
    assign readdata_byte[6] = readdata_ff[15:8];
    assign readdata_byte[7] = readdata_ff[7:0];

    /* A ARRAY */
    logic [7:0] dataoutA [7:0];                 // Row of the 2D A array stored into the FIFO
    logic allFull;
    logic [7:0] rdenA, wrenA, emptyA, fullA;
    assign allFull = &fullA;

    /* B ARRAY */
    logic [7:0] dataoutB;                       // B array 
    logic rdenB, wrenB, emptyB, fullB;          // Control signals for the B FIFO

    /* MAC */
    logic startExec;
    logic [7:0] Ain;
    logic [7:0] Ain_ff;
    logic En;
    logic En_ff;
    logic [7:0] Bin;
    logic [7:0] Bin_ff;
    logic [23:0] Cout;
    logic [23:0] cout_reg_00;
    logic [23:0] cout_reg_01;
    logic [23:0] cout_reg_02;
    logic [23:0] cout_reg_03;
    logic [23:0] cout_reg_04;
    logic [23:0] cout_reg_05;
    logic [23:0] cout_reg_06;
    logic [23:0] cout_reg_07;
    reg [2:0] mac_count;        // increment the mac_count depending on the step you're on

    //MEM TO FIFO flops
    logic [7:0] rdenA_ff, wrenA_ff, datain_ff;
    logic rdenB_ff, wrenB_ff;
    always_ff @(posedge CLOCK_50, negedge rst_n) begin 
        if (!rst_n) begin 
            rdenA_ff <= '0;
            wrenA_ff <= '0; 
            datain_ff <= '0;
            rdenB_ff <= '0;
            wrenB_ff <= '0;
        end 
        else begin 
            rdenA_ff <= rdenA;
            wrenA_ff <= wrenA;
            datain_ff <= datain;
            rdenB_ff <= rdenB;
            wrenB_ff <= wrenB;
        end
    end

    // Propogate enable
    always_ff @(posedge CLOCK_50, negedge rst_n) begin
        if (!rst_n) 
            En <= 0;
        else if (|rdenA_ff & rdenB_ff) 
            En <= 1;
        else 
            En <= 0;
    end
    
    // Instantiate the MAC module
    mac mac(
        .clk(CLOCK_50),
        .rst_n(rst_n),
        .En(En_ff),    // Enable MAC when reading from exactly any one A FIFO and exactly one B FIFO
        .Clr(Clr),
        .Ain(Ain_ff),
        .Bin(Bin_ff),
        .Cout(Cout));

    // Instantiate the memory module
    mem_wrapper memory(
        .clk(CLOCK_50),    
        .reset_n(rst_n),
        .address(address),              // 32-bit address for 8 rows
        .read(read),                    // Read request
        // Outputs
        .readdata(readdata),            // 64-bit read data (one row)
        .readdatavalid(readdatavalid),  // Data valid signal
        .waitrequest(waitrequest)       // Busy signal to indicate logic is processing
    );

    // Pipeline memory outputs
    always_ff @(posedge CLOCK_50, negedge rst_n) begin 
        if (!rst_n) begin 
            readdata_ff <= '0;
            readdatavalid_ff <= '0;
            waitrequest_ff <= '0;
        end
        else begin
            readdata_ff <= readdata;
            readdatavalid_ff <= readdatavalid;
            waitrequest_ff <= waitrequest;
        end
    end

    // Generate FIFOs for the 8x8 A input
    genvar k;
    generate 
    for (k = 0; k < 8; k++)  
    begin: A_fifo_gen
        fifo A_fifo(
            // Inputs
            .aclr(aclr),
            .data(datain_ff),
            .rdclk(CLOCK_50),
            .rdreq(rdenA_ff[k]),
            .wrclk(CLOCK_50),
            .wrreq(wrenA_ff[k]),
            // Outputs
            .q(dataoutA[k]),
            .rdempty(emptyA[k]),
            .wrfull(fullA[k])
        );
    end
    endgenerate

    generate
        begin: B_fifo_gen
            fifo B_fifo(
                // Inputs
                .aclr(aclr),
                .data(datain_ff),
                .rdclk(CLOCK_50),
                .rdreq(rdenB_ff),
                .wrclk(CLOCK_50),
                .wrreq(wrenB_ff),
                // Outputs
                .q(dataoutB),
                .rdempty(emptyB),
                .wrfull(fullB)
            );
        end
    endgenerate

    // State and next state logic
    typedef enum reg [2:0] {IDLE, READ, FILLA, FILLB, WAIT, EXEC, DONE} state_t;
    state_t state, next_state;

    always_ff @(posedge CLOCK_50, negedge rst_n) 
    begin
        if (!rst_n)
            state <= IDLE;
        else 
            state <= next_state;
    end
    
    // Structural coding
    assign rst_n = KEY[0];
    logic reading;          // Variable to tell the column to start incrementing
    logic nextrow;          // For filling A, move a row down after A is full
    logic next_cout;        // Used for changing what wire of the MAC output is there

    // Counter for incrementing which byte-size column we're writing to the FIFO
    reg [2:0] column;
    always_ff@(posedge CLOCK_50, negedge rst_n) begin
        if (!rst_n) 
            column <= '0;
        else if (nextrow)
            column <= '0;
        else if (reading) 
            column <= column + 1;
        else if (&emptyA) 
            column <= '0;
        else if (Exec)
            column <= '0;
    end

    // Counter for address, to increment rows
    logic fillB;
    always_ff @(posedge CLOCK_50, negedge rst_n) begin
        if (!rst_n)
            address <= '0;
        else if (fillB)
            address <= '0;
        else if (nextrow)
            address <= address + 1;
    end

    // Counter for controlling the otput from MAC
    always_ff @(posedge CLOCK_50, negedge rst_n) begin
        if (!rst_n) begin
            mac_count <= '0;
            Clr <= 1;
        end
        else if (next_cout) begin
            Clr <= 1;
            mac_count <= mac_count + 1;
        end
        else 
            Clr <= 0;
    end

    always_ff @(posedge CLOCK_50, negedge rst_n) begin
        if (!rst_n) begin
            cout_reg_00 <= '0;
            cout_reg_01 <= '0;
            cout_reg_02 <= '0;
            cout_reg_03 <= '0;
            cout_reg_04 <= '0;
            cout_reg_05 <= '0;
            cout_reg_06 <= '0;
            cout_reg_07 <= '0;
        end
        else begin
            case(mac_count)
                3'd0: cout_reg_00 <= Cout;
                3'd1: cout_reg_01 <= Cout;
                3'd2: cout_reg_02 <= Cout;
                3'd3: cout_reg_03 <= Cout;
                3'd4: cout_reg_04 <= Cout;
                3'd5: cout_reg_05 <= Cout;
                3'd6: cout_reg_06 <= Cout;
                3'd7: cout_reg_07 <= Cout;
            endcase
        end
    end

    always_ff @(posedge CLOCK_50, negedge rst_n) begin
        if(!rst_n)
            Exec <= 0;
        else if (startExec)
            Exec <= 1;
    end

    // State machine
    always_comb
    begin
        // Defaults
        next_state = state;
        next_cout = 0;
        datain = '0;
        reading = 0;
        nextrow = 0;
        read = 0;
        wrenA = '0;
        wrenB = 0;
        rdenA = '0;
        rdenB = 0;
        Ain = '0;
        Bin = '0;
        fillB = 0;
        startExec = 0;
        aclr = 0;

        case(state)
            IDLE:
            begin
                next_state = READ;
                rdenA = '0;
                rdenB = 0;
                aclr = 1;
            end
            READ:
            begin
                read = 1'b1;
                // Need to check for data_valid signal from memory before filling the FIFO
                if (readdatavalid_ff & Exec) begin                                 // Refill B 
                    next_state = FILLB;
                    next_cout = 1;
                end
                else if (readdatavalid_ff & !fullB)            // Fill B 
                    next_state = FILLB;
                else if (readdatavalid_ff & !(allFull))        // Fill A
                    next_state = FILLA;      
            end
            FILLB:
            begin
                // As long as B FIFO is not full, keep filling
                if (!fullB)
                begin
                    reading = 1;
                    wrenB = 1'b1;
                    datain = readdata_byte[column];
                end
                else if (Exec)
                begin
                    wrenB = 0;
                    next_state = EXEC;
                end
                else 
                begin
                    wrenB = 0;
                    next_state = READ;
                    nextrow = 1;
                end
            end
            FILLA:
            begin
                if(!allFull & ~waitrequest_ff)
                begin
                    reading = 1;
                    wrenA |= (1 << (address-1));
                    datain = readdata_byte[column];

                    // Read the next row when current FIFO is full
                    if (fullA[address-1])
                    begin
                        nextrow = 1;
                        wrenA = '0;
                        next_state = READ;
                    end
                end
                else if (allFull & ~waitrequest_ff) begin
                    wrenA = '0;
                    next_state = EXEC;

                    // Set up to fill B many times 
                    fillB = 1;                      // Resets address to fill B again
                    startExec = 1;
                end
            end
            WAIT:
            begin
                next_state = EXEC;
            end
            EXEC:
            begin
                // When the A FIFOs are exhausted, stop the MAC unit
                if (&emptyA)
                    next_state = DONE;
                // Once B is empty, refill before emptying the next row in A
                else if (emptyB) 
                    next_state = READ;  
                // Pop off from A and B, takes one cycle
                rdenB = 1;
                rdenA |= (1 << (mac_count));
                Ain = dataoutA[mac_count];
                Bin = dataoutB;
            end
            // DONE STATE
            default:
            begin
                next_state = DONE;
            end
        endcase
    end

always_ff @(posedge CLOCK_50, negedge rst_n) begin 
    if (!rst_n) begin
        Ain_ff <= '0; 
        Bin_ff <= '0;
        En_ff <= '0;
    end
    else if (Clr) begin
        Ain_ff <= '0; 
        Bin_ff <= '0;
        En_ff <= '0;
    end 
    else begin
        Ain_ff <= Ain; 
        Bin_ff <= Bin;
        En_ff <= En;
    end
end

// Implement the LED display
always @(*) begin
  if (state == DONE & SW[0]) begin
    case(cout_reg_00[3:0])
      4'd0: HEX0 = HEX_0;
      4'd1: HEX0 = HEX_1;
      4'd2: HEX0 = HEX_2;
      4'd3: HEX0 = HEX_3;
      4'd4: HEX0 = HEX_4;
      4'd5: HEX0 = HEX_5;
      4'd6: HEX0 = HEX_6;
      4'd7: HEX0 = HEX_7;
      4'd8: HEX0 = HEX_8;
      4'd9: HEX0 = HEX_9;
      4'd10: HEX0 = HEX_10;
      4'd11: HEX0 = HEX_11;
      4'd12: HEX0 = HEX_12;
      4'd13: HEX0 = HEX_13;
      4'd14: HEX0 = HEX_14;
      4'd15: HEX0 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[1]) begin
    case(cout_reg_01[3:0])
      4'd0: HEX0 = HEX_0;
      4'd1: HEX0 = HEX_1;
      4'd2: HEX0 = HEX_2;
      4'd3: HEX0 = HEX_3;
      4'd4: HEX0 = HEX_4;
      4'd5: HEX0 = HEX_5;
      4'd6: HEX0 = HEX_6;
      4'd7: HEX0 = HEX_7;
      4'd8: HEX0 = HEX_8;
      4'd9: HEX0 = HEX_9;
      4'd10: HEX0 = HEX_10;
      4'd11: HEX0 = HEX_11;
      4'd12: HEX0 = HEX_12;
      4'd13: HEX0 = HEX_13;
      4'd14: HEX0 = HEX_14;
      4'd15: HEX0 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[2]) begin
    case(cout_reg_02[3:0])
      4'd0: HEX0 = HEX_0;
      4'd1: HEX0 = HEX_1;
      4'd2: HEX0 = HEX_2;
      4'd3: HEX0 = HEX_3;
      4'd4: HEX0 = HEX_4;
      4'd5: HEX0 = HEX_5;
      4'd6: HEX0 = HEX_6;
      4'd7: HEX0 = HEX_7;
      4'd8: HEX0 = HEX_8;
      4'd9: HEX0 = HEX_9;
      4'd10: HEX0 = HEX_10;
      4'd11: HEX0 = HEX_11;
      4'd12: HEX0 = HEX_12;
      4'd13: HEX0 = HEX_13;
      4'd14: HEX0 = HEX_14;
      4'd15: HEX0 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[3]) begin
    case(cout_reg_03[3:0])
      4'd0: HEX0 = HEX_0;
      4'd1: HEX0 = HEX_1;
      4'd2: HEX0 = HEX_2;
      4'd3: HEX0 = HEX_3;
      4'd4: HEX0 = HEX_4;
      4'd5: HEX0 = HEX_5;
      4'd6: HEX0 = HEX_6;
      4'd7: HEX0 = HEX_7;
      4'd8: HEX0 = HEX_8;
      4'd9: HEX0 = HEX_9;
      4'd10: HEX0 = HEX_10;
      4'd11: HEX0 = HEX_11;
      4'd12: HEX0 = HEX_12;
      4'd13: HEX0 = HEX_13;
      4'd14: HEX0 = HEX_14;
      4'd15: HEX0 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[4]) begin
    case(cout_reg_04[3:0])
      4'd0: HEX0 = HEX_0;
      4'd1: HEX0 = HEX_1;
      4'd2: HEX0 = HEX_2;
      4'd3: HEX0 = HEX_3;
      4'd4: HEX0 = HEX_4;
      4'd5: HEX0 = HEX_5;
      4'd6: HEX0 = HEX_6;
      4'd7: HEX0 = HEX_7;
      4'd8: HEX0 = HEX_8;
      4'd9: HEX0 = HEX_9;
      4'd10: HEX0 = HEX_10;
      4'd11: HEX0 = HEX_11;
      4'd12: HEX0 = HEX_12;
      4'd13: HEX0 = HEX_13;
      4'd14: HEX0 = HEX_14;
      4'd15: HEX0 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[5]) begin
    case(cout_reg_05[3:0])
      4'd0: HEX0 = HEX_0;
      4'd1: HEX0 = HEX_1;
      4'd2: HEX0 = HEX_2;
      4'd3: HEX0 = HEX_3;
      4'd4: HEX0 = HEX_4;
      4'd5: HEX0 = HEX_5;
      4'd6: HEX0 = HEX_6;
      4'd7: HEX0 = HEX_7;
      4'd8: HEX0 = HEX_8;
      4'd9: HEX0 = HEX_9;
      4'd10: HEX0 = HEX_10;
      4'd11: HEX0 = HEX_11;
      4'd12: HEX0 = HEX_12;
      4'd13: HEX0 = HEX_13;
      4'd14: HEX0 = HEX_14;
      4'd15: HEX0 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[6]) begin
    case(cout_reg_06[3:0])
      4'd0: HEX0 = HEX_0;
      4'd1: HEX0 = HEX_1;
      4'd2: HEX0 = HEX_2;
      4'd3: HEX0 = HEX_3;
      4'd4: HEX0 = HEX_4;
      4'd5: HEX0 = HEX_5;
      4'd6: HEX0 = HEX_6;
      4'd7: HEX0 = HEX_7;
      4'd8: HEX0 = HEX_8;
      4'd9: HEX0 = HEX_9;
      4'd10: HEX0 = HEX_10;
      4'd11: HEX0 = HEX_11;
      4'd12: HEX0 = HEX_12;
      4'd13: HEX0 = HEX_13;
      4'd14: HEX0 = HEX_14;
      4'd15: HEX0 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[7]) begin
    case(cout_reg_07[3:0])
      4'd0: HEX0 = HEX_0;
      4'd1: HEX0 = HEX_1;
      4'd2: HEX0 = HEX_2;
      4'd3: HEX0 = HEX_3;
      4'd4: HEX0 = HEX_4;
      4'd5: HEX0 = HEX_5;
      4'd6: HEX0 = HEX_6;
      4'd7: HEX0 = HEX_7;
      4'd8: HEX0 = HEX_8;
      4'd9: HEX0 = HEX_9;
      4'd10: HEX0 = HEX_10;
      4'd11: HEX0 = HEX_11;
      4'd12: HEX0 = HEX_12;
      4'd13: HEX0 = HEX_13;
      4'd14: HEX0 = HEX_14;
      4'd15: HEX0 = HEX_15;
    endcase
  end
  else begin
    HEX0 = OFF;
  end
end

always @(*) begin
  if (state == DONE & SW[0]) begin
    case(cout_reg_00[7:4])
      4'd0: HEX1 = HEX_0;
      4'd1: HEX1 = HEX_1;
      4'd2: HEX1 = HEX_2;
      4'd3: HEX1 = HEX_3;
      4'd4: HEX1 = HEX_4;
      4'd5: HEX1 = HEX_5;
      4'd6: HEX1 = HEX_6;
      4'd7: HEX1 = HEX_7;
      4'd8: HEX1 = HEX_8;
      4'd9: HEX1 = HEX_9;
      4'd10: HEX1 = HEX_10;
      4'd11: HEX1 = HEX_11;
      4'd12: HEX1 = HEX_12;
      4'd13: HEX1 = HEX_13;
      4'd14: HEX1 = HEX_14;
      4'd15: HEX1 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[1]) begin
    case(cout_reg_01[7:4])
      4'd0: HEX1 = HEX_0;
      4'd1: HEX1 = HEX_1;
      4'd2: HEX1 = HEX_2;
      4'd3: HEX1 = HEX_3;
      4'd4: HEX1 = HEX_4;
      4'd5: HEX1 = HEX_5;
      4'd6: HEX1 = HEX_6;
      4'd7: HEX1 = HEX_7;
      4'd8: HEX1 = HEX_8;
      4'd9: HEX1 = HEX_9;
      4'd10: HEX1 = HEX_10;
      4'd11: HEX1 = HEX_11;
      4'd12: HEX1 = HEX_12;
      4'd13: HEX1 = HEX_13;
      4'd14: HEX1 = HEX_14;
      4'd15: HEX1 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[2]) begin
    case(cout_reg_02[7:4])
      4'd0: HEX1 = HEX_0;
      4'd1: HEX1 = HEX_1;
      4'd2: HEX1 = HEX_2;
      4'd3: HEX1 = HEX_3;
      4'd4: HEX1 = HEX_4;
      4'd5: HEX1 = HEX_5;
      4'd6: HEX1 = HEX_6;
      4'd7: HEX1 = HEX_7;
      4'd8: HEX1 = HEX_8;
      4'd9: HEX1 = HEX_9;
      4'd10: HEX1 = HEX_10;
      4'd11: HEX1 = HEX_11;
      4'd12: HEX1 = HEX_12;
      4'd13: HEX1 = HEX_13;
      4'd14: HEX1 = HEX_14;
      4'd15: HEX1 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[3]) begin
    case(cout_reg_03[7:4])
      4'd0: HEX1 = HEX_0;
      4'd1: HEX1 = HEX_1;
      4'd2: HEX1 = HEX_2;
      4'd3: HEX1 = HEX_3;
      4'd4: HEX1 = HEX_4;
      4'd5: HEX1 = HEX_5;
      4'd6: HEX1 = HEX_6;
      4'd7: HEX1 = HEX_7;
      4'd8: HEX1 = HEX_8;
      4'd9: HEX1 = HEX_9;
      4'd10: HEX1 = HEX_10;
      4'd11: HEX1 = HEX_11;
      4'd12: HEX1 = HEX_12;
      4'd13: HEX1 = HEX_13;
      4'd14: HEX1 = HEX_14;
      4'd15: HEX1 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[4]) begin
    case(cout_reg_04[7:4])
      4'd0: HEX1 = HEX_0;
      4'd1: HEX1 = HEX_1;
      4'd2: HEX1 = HEX_2;
      4'd3: HEX1 = HEX_3;
      4'd4: HEX1 = HEX_4;
      4'd5: HEX1 = HEX_5;
      4'd6: HEX1 = HEX_6;
      4'd7: HEX1 = HEX_7;
      4'd8: HEX1 = HEX_8;
      4'd9: HEX1 = HEX_9;
      4'd10: HEX1 = HEX_10;
      4'd11: HEX1 = HEX_11;
      4'd12: HEX1 = HEX_12;
      4'd13: HEX1 = HEX_13;
      4'd14: HEX1 = HEX_14;
      4'd15: HEX1 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[5]) begin
    case(cout_reg_05[7:4])
      4'd0: HEX1 = HEX_0;
      4'd1: HEX1 = HEX_1;
      4'd2: HEX1 = HEX_2;
      4'd3: HEX1 = HEX_3;
      4'd4: HEX1 = HEX_4;
      4'd5: HEX1 = HEX_5;
      4'd6: HEX1 = HEX_6;
      4'd7: HEX1 = HEX_7;
      4'd8: HEX1 = HEX_8;
      4'd9: HEX1 = HEX_9;
      4'd10: HEX1 = HEX_10;
      4'd11: HEX1 = HEX_11;
      4'd12: HEX1 = HEX_12;
      4'd13: HEX1 = HEX_13;
      4'd14: HEX1 = HEX_14;
      4'd15: HEX1 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[6]) begin
    case(cout_reg_06[7:4])
      4'd0: HEX1 = HEX_0;
      4'd1: HEX1 = HEX_1;
      4'd2: HEX1 = HEX_2;
      4'd3: HEX1 = HEX_3;
      4'd4: HEX1 = HEX_4;
      4'd5: HEX1 = HEX_5;
      4'd6: HEX1 = HEX_6;
      4'd7: HEX1 = HEX_7;
      4'd8: HEX1 = HEX_8;
      4'd9: HEX1 = HEX_9;
      4'd10: HEX1 = HEX_10;
      4'd11: HEX1 = HEX_11;
      4'd12: HEX1 = HEX_12;
      4'd13: HEX1 = HEX_13;
      4'd14: HEX1 = HEX_14;
      4'd15: HEX1 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[7]) begin
    case(cout_reg_07[7:4])
      4'd0: HEX1 = HEX_0;
      4'd1: HEX1 = HEX_1;
      4'd2: HEX1 = HEX_2;
      4'd3: HEX1 = HEX_3;
      4'd4: HEX1 = HEX_4;
      4'd5: HEX1 = HEX_5;
      4'd6: HEX1 = HEX_6;
      4'd7: HEX1 = HEX_7;
      4'd8: HEX1 = HEX_8;
      4'd9: HEX1 = HEX_9;
      4'd10: HEX1 = HEX_10;
      4'd11: HEX1 = HEX_11;
      4'd12: HEX1 = HEX_12;
      4'd13: HEX1 = HEX_13;
      4'd14: HEX1 = HEX_14;
      4'd15: HEX1 = HEX_15;
    endcase
  end
  else begin
    HEX1 = OFF;
  end
end

always @(*) begin
  if (state == DONE & SW[0]) begin
    case(cout_reg_00[11:8])
      4'd0: HEX2 = HEX_0;
      4'd1: HEX2 = HEX_1;
      4'd2: HEX2 = HEX_2;
      4'd3: HEX2 = HEX_3;
      4'd4: HEX2 = HEX_4;
      4'd5: HEX2 = HEX_5;
      4'd6: HEX2 = HEX_6;
      4'd7: HEX2 = HEX_7;
      4'd8: HEX2 = HEX_8;
      4'd9: HEX2 = HEX_9;
      4'd10: HEX2 = HEX_10;
      4'd11: HEX2 = HEX_11;
      4'd12: HEX2 = HEX_12;
      4'd13: HEX2 = HEX_13;
      4'd14: HEX2 = HEX_14;
      4'd15: HEX2 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[1]) begin
    case(cout_reg_01[11:8])
      4'd0: HEX2 = HEX_0;
      4'd1: HEX2 = HEX_1;
      4'd2: HEX2 = HEX_2;
      4'd3: HEX2 = HEX_3;
      4'd4: HEX2 = HEX_4;
      4'd5: HEX2 = HEX_5;
      4'd6: HEX2 = HEX_6;
      4'd7: HEX2 = HEX_7;
      4'd8: HEX2 = HEX_8;
      4'd9: HEX2 = HEX_9;
      4'd10: HEX2 = HEX_10;
      4'd11: HEX2 = HEX_11;
      4'd12: HEX2 = HEX_12;
      4'd13: HEX2 = HEX_13;
      4'd14: HEX2 = HEX_14;
      4'd15: HEX2 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[2]) begin
    case(cout_reg_02[11:8])
      4'd0: HEX2 = HEX_0;
      4'd1: HEX2 = HEX_1;
      4'd2: HEX2 = HEX_2;
      4'd3: HEX2 = HEX_3;
      4'd4: HEX2 = HEX_4;
      4'd5: HEX2 = HEX_5;
      4'd6: HEX2 = HEX_6;
      4'd7: HEX2 = HEX_7;
      4'd8: HEX2 = HEX_8;
      4'd9: HEX2 = HEX_9;
      4'd10: HEX2 = HEX_10;
      4'd11: HEX2 = HEX_11;
      4'd12: HEX2 = HEX_12;
      4'd13: HEX2 = HEX_13;
      4'd14: HEX2 = HEX_14;
      4'd15: HEX2 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[3]) begin
    case(cout_reg_03[11:8])
      4'd0: HEX2 = HEX_0;
      4'd1: HEX2 = HEX_1;
      4'd2: HEX2 = HEX_2;
      4'd3: HEX2 = HEX_3;
      4'd4: HEX2 = HEX_4;
      4'd5: HEX2 = HEX_5;
      4'd6: HEX2 = HEX_6;
      4'd7: HEX2 = HEX_7;
      4'd8: HEX2 = HEX_8;
      4'd9: HEX2 = HEX_9;
      4'd10: HEX2 = HEX_10;
      4'd11: HEX2 = HEX_11;
      4'd12: HEX2 = HEX_12;
      4'd13: HEX2 = HEX_13;
      4'd14: HEX2 = HEX_14;
      4'd15: HEX2 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[4]) begin
    case(cout_reg_04[11:8])
      4'd0: HEX2 = HEX_0;
      4'd1: HEX2 = HEX_1;
      4'd2: HEX2 = HEX_2;
      4'd3: HEX2 = HEX_3;
      4'd4: HEX2 = HEX_4;
      4'd5: HEX2 = HEX_5;
      4'd6: HEX2 = HEX_6;
      4'd7: HEX2 = HEX_7;
      4'd8: HEX2 = HEX_8;
      4'd9: HEX2 = HEX_9;
      4'd10: HEX2 = HEX_10;
      4'd11: HEX2 = HEX_11;
      4'd12: HEX2 = HEX_12;
      4'd13: HEX2 = HEX_13;
      4'd14: HEX2 = HEX_14;
      4'd15: HEX2 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[5]) begin
    case(cout_reg_05[11:8])
      4'd0: HEX2 = HEX_0;
      4'd1: HEX2 = HEX_1;
      4'd2: HEX2 = HEX_2;
      4'd3: HEX2 = HEX_3;
      4'd4: HEX2 = HEX_4;
      4'd5: HEX2 = HEX_5;
      4'd6: HEX2 = HEX_6;
      4'd7: HEX2 = HEX_7;
      4'd8: HEX2 = HEX_8;
      4'd9: HEX2 = HEX_9;
      4'd10: HEX2 = HEX_10;
      4'd11: HEX2 = HEX_11;
      4'd12: HEX2 = HEX_12;
      4'd13: HEX2 = HEX_13;
      4'd14: HEX2 = HEX_14;
      4'd15: HEX2 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[6]) begin
    case(cout_reg_06[11:8])
      4'd0: HEX2 = HEX_0;
      4'd1: HEX2 = HEX_1;
      4'd2: HEX2 = HEX_2;
      4'd3: HEX2 = HEX_3;
      4'd4: HEX2 = HEX_4;
      4'd5: HEX2 = HEX_5;
      4'd6: HEX2 = HEX_6;
      4'd7: HEX2 = HEX_7;
      4'd8: HEX2 = HEX_8;
      4'd9: HEX2 = HEX_9;
      4'd10: HEX2 = HEX_10;
      4'd11: HEX2 = HEX_11;
      4'd12: HEX2 = HEX_12;
      4'd13: HEX2 = HEX_13;
      4'd14: HEX2 = HEX_14;
      4'd15: HEX2 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[7]) begin
    case(cout_reg_07[11:8])
      4'd0: HEX2 = HEX_0;
      4'd1: HEX2 = HEX_1;
      4'd2: HEX2 = HEX_2;
      4'd3: HEX2 = HEX_3;
      4'd4: HEX2 = HEX_4;
      4'd5: HEX2 = HEX_5;
      4'd6: HEX2 = HEX_6;
      4'd7: HEX2 = HEX_7;
      4'd8: HEX2 = HEX_8;
      4'd9: HEX2 = HEX_9;
      4'd10: HEX2 = HEX_10;
      4'd11: HEX2 = HEX_11;
      4'd12: HEX2 = HEX_12;
      4'd13: HEX2 = HEX_13;
      4'd14: HEX2 = HEX_14;
      4'd15: HEX2 = HEX_15;
    endcase
  end
  else begin
    HEX2 = OFF;
  end
end

always @(*) begin
  if (state == DONE & SW[0]) begin
    case(cout_reg_00[15:12])
      4'd0: HEX3 = HEX_0;
      4'd1: HEX3 = HEX_1;
      4'd2: HEX3 = HEX_2;
      4'd3: HEX3 = HEX_3;
      4'd4: HEX3 = HEX_4;
      4'd5: HEX3 = HEX_5;
      4'd6: HEX3 = HEX_6;
      4'd7: HEX3 = HEX_7;
      4'd8: HEX3 = HEX_8;
      4'd9: HEX3 = HEX_9;
      4'd10: HEX3 = HEX_10;
      4'd11: HEX3 = HEX_11;
      4'd12: HEX3 = HEX_12;
      4'd13: HEX3 = HEX_13;
      4'd14: HEX3 = HEX_14;
      4'd15: HEX3 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[1]) begin
    case(cout_reg_01[15:12])
      4'd0: HEX3 = HEX_0;
      4'd1: HEX3 = HEX_1;
      4'd2: HEX3 = HEX_2;
      4'd3: HEX3 = HEX_3;
      4'd4: HEX3 = HEX_4;
      4'd5: HEX3 = HEX_5;
      4'd6: HEX3 = HEX_6;
      4'd7: HEX3 = HEX_7;
      4'd8: HEX3 = HEX_8;
      4'd9: HEX3 = HEX_9;
      4'd10: HEX3 = HEX_10;
      4'd11: HEX3 = HEX_11;
      4'd12: HEX3 = HEX_12;
      4'd13: HEX3 = HEX_13;
      4'd14: HEX3 = HEX_14;
      4'd15: HEX3 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[2]) begin
    case(cout_reg_02[15:12])
      4'd0: HEX3 = HEX_0;
      4'd1: HEX3 = HEX_1;
      4'd2: HEX3 = HEX_2;
      4'd3: HEX3 = HEX_3;
      4'd4: HEX3 = HEX_4;
      4'd5: HEX3 = HEX_5;
      4'd6: HEX3 = HEX_6;
      4'd7: HEX3 = HEX_7;
      4'd8: HEX3 = HEX_8;
      4'd9: HEX3 = HEX_9;
      4'd10: HEX3 = HEX_10;
      4'd11: HEX3 = HEX_11;
      4'd12: HEX3 = HEX_12;
      4'd13: HEX3 = HEX_13;
      4'd14: HEX3 = HEX_14;
      4'd15: HEX3 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[3]) begin
    case(cout_reg_03[15:12])
      4'd0: HEX3 = HEX_0;
      4'd1: HEX3 = HEX_1;
      4'd2: HEX3 = HEX_2;
      4'd3: HEX3 = HEX_3;
      4'd4: HEX3 = HEX_4;
      4'd5: HEX3 = HEX_5;
      4'd6: HEX3 = HEX_6;
      4'd7: HEX3 = HEX_7;
      4'd8: HEX3 = HEX_8;
      4'd9: HEX3 = HEX_9;
      4'd10: HEX3 = HEX_10;
      4'd11: HEX3 = HEX_11;
      4'd12: HEX3 = HEX_12;
      4'd13: HEX3 = HEX_13;
      4'd14: HEX3 = HEX_14;
      4'd15: HEX3 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[4]) begin
    case(cout_reg_04[15:12])
      4'd0: HEX3 = HEX_0;
      4'd1: HEX3 = HEX_1;
      4'd2: HEX3 = HEX_2;
      4'd3: HEX3 = HEX_3;
      4'd4: HEX3 = HEX_4;
      4'd5: HEX3 = HEX_5;
      4'd6: HEX3 = HEX_6;
      4'd7: HEX3 = HEX_7;
      4'd8: HEX3 = HEX_8;
      4'd9: HEX3 = HEX_9;
      4'd10: HEX3 = HEX_10;
      4'd11: HEX3 = HEX_11;
      4'd12: HEX3 = HEX_12;
      4'd13: HEX3 = HEX_13;
      4'd14: HEX3 = HEX_14;
      4'd15: HEX3 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[5]) begin
    case(cout_reg_05[15:12])
      4'd0: HEX3 = HEX_0;
      4'd1: HEX3 = HEX_1;
      4'd2: HEX3 = HEX_2;
      4'd3: HEX3 = HEX_3;
      4'd4: HEX3 = HEX_4;
      4'd5: HEX3 = HEX_5;
      4'd6: HEX3 = HEX_6;
      4'd7: HEX3 = HEX_7;
      4'd8: HEX3 = HEX_8;
      4'd9: HEX3 = HEX_9;
      4'd10: HEX3 = HEX_10;
      4'd11: HEX3 = HEX_11;
      4'd12: HEX3 = HEX_12;
      4'd13: HEX3 = HEX_13;
      4'd14: HEX3 = HEX_14;
      4'd15: HEX3 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[6]) begin
    case(cout_reg_06[15:12])
      4'd0: HEX3 = HEX_0;
      4'd1: HEX3 = HEX_1;
      4'd2: HEX3 = HEX_2;
      4'd3: HEX3 = HEX_3;
      4'd4: HEX3 = HEX_4;
      4'd5: HEX3 = HEX_5;
      4'd6: HEX3 = HEX_6;
      4'd7: HEX3 = HEX_7;
      4'd8: HEX3 = HEX_8;
      4'd9: HEX3 = HEX_9;
      4'd10: HEX3 = HEX_10;
      4'd11: HEX3 = HEX_11;
      4'd12: HEX3 = HEX_12;
      4'd13: HEX3 = HEX_13;
      4'd14: HEX3 = HEX_14;
      4'd15: HEX3 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[7]) begin
    case(cout_reg_07[15:12])
      4'd0: HEX3 = HEX_0;
      4'd1: HEX3 = HEX_1;
      4'd2: HEX3 = HEX_2;
      4'd3: HEX3 = HEX_3;
      4'd4: HEX3 = HEX_4;
      4'd5: HEX3 = HEX_5;
      4'd6: HEX3 = HEX_6;
      4'd7: HEX3 = HEX_7;
      4'd8: HEX3 = HEX_8;
      4'd9: HEX3 = HEX_9;
      4'd10: HEX3 = HEX_10;
      4'd11: HEX3 = HEX_11;
      4'd12: HEX3 = HEX_12;
      4'd13: HEX3 = HEX_13;
      4'd14: HEX3 = HEX_14;
      4'd15: HEX3 = HEX_15;
    endcase
  end
  else begin
    HEX3 = OFF;
  end
end

always @(*) begin
  if (state == DONE & SW[0]) begin
    case(cout_reg_00[19:16])
      4'd0: HEX4 = HEX_0;
      4'd1: HEX4 = HEX_1;
      4'd2: HEX4 = HEX_2;
      4'd3: HEX4 = HEX_3;
      4'd4: HEX4 = HEX_4;
      4'd5: HEX4 = HEX_5;
      4'd6: HEX4 = HEX_6;
      4'd7: HEX4 = HEX_7;
      4'd8: HEX4 = HEX_8;
      4'd9: HEX4 = HEX_9;
      4'd10: HEX4 = HEX_10;
      4'd11: HEX4 = HEX_11;
      4'd12: HEX4 = HEX_12;
      4'd13: HEX4 = HEX_13;
      4'd14: HEX4 = HEX_14;
      4'd15: HEX4 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[1]) begin
    case(cout_reg_01[19:16])
      4'd0: HEX4 = HEX_0;
      4'd1: HEX4 = HEX_1;
      4'd2: HEX4 = HEX_2;
      4'd3: HEX4 = HEX_3;
      4'd4: HEX4 = HEX_4;
      4'd5: HEX4 = HEX_5;
      4'd6: HEX4 = HEX_6;
      4'd7: HEX4 = HEX_7;
      4'd8: HEX4 = HEX_8;
      4'd9: HEX4 = HEX_9;
      4'd10: HEX4 = HEX_10;
      4'd11: HEX4 = HEX_11;
      4'd12: HEX4 = HEX_12;
      4'd13: HEX4 = HEX_13;
      4'd14: HEX4 = HEX_14;
      4'd15: HEX4 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[2]) begin
    case(cout_reg_02[19:16])
      4'd0: HEX4 = HEX_0;
      4'd1: HEX4 = HEX_1;
      4'd2: HEX4 = HEX_2;
      4'd3: HEX4 = HEX_3;
      4'd4: HEX4 = HEX_4;
      4'd5: HEX4 = HEX_5;
      4'd6: HEX4 = HEX_6;
      4'd7: HEX4 = HEX_7;
      4'd8: HEX4 = HEX_8;
      4'd9: HEX4 = HEX_9;
      4'd10: HEX4 = HEX_10;
      4'd11: HEX4 = HEX_11;
      4'd12: HEX4 = HEX_12;
      4'd13: HEX4 = HEX_13;
      4'd14: HEX4 = HEX_14;
      4'd15: HEX4 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[3]) begin
    case(cout_reg_03[19:16])
      4'd0: HEX4 = HEX_0;
      4'd1: HEX4 = HEX_1;
      4'd2: HEX4 = HEX_2;
      4'd3: HEX4 = HEX_3;
      4'd4: HEX4 = HEX_4;
      4'd5: HEX4 = HEX_5;
      4'd6: HEX4 = HEX_6;
      4'd7: HEX4 = HEX_7;
      4'd8: HEX4 = HEX_8;
      4'd9: HEX4 = HEX_9;
      4'd10: HEX4 = HEX_10;
      4'd11: HEX4 = HEX_11;
      4'd12: HEX4 = HEX_12;
      4'd13: HEX4 = HEX_13;
      4'd14: HEX4 = HEX_14;
      4'd15: HEX4 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[4]) begin
    case(cout_reg_04[19:16])
      4'd0: HEX4 = HEX_0;
      4'd1: HEX4 = HEX_1;
      4'd2: HEX4 = HEX_2;
      4'd3: HEX4 = HEX_3;
      4'd4: HEX4 = HEX_4;
      4'd5: HEX4 = HEX_5;
      4'd6: HEX4 = HEX_6;
      4'd7: HEX4 = HEX_7;
      4'd8: HEX4 = HEX_8;
      4'd9: HEX4 = HEX_9;
      4'd10: HEX4 = HEX_10;
      4'd11: HEX4 = HEX_11;
      4'd12: HEX4 = HEX_12;
      4'd13: HEX4 = HEX_13;
      4'd14: HEX4 = HEX_14;
      4'd15: HEX4 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[5]) begin
    case(cout_reg_05[19:16])
      4'd0: HEX4 = HEX_0;
      4'd1: HEX4 = HEX_1;
      4'd2: HEX4 = HEX_2;
      4'd3: HEX4 = HEX_3;
      4'd4: HEX4 = HEX_4;
      4'd5: HEX4 = HEX_5;
      4'd6: HEX4 = HEX_6;
      4'd7: HEX4 = HEX_7;
      4'd8: HEX4 = HEX_8;
      4'd9: HEX4 = HEX_9;
      4'd10: HEX4 = HEX_10;
      4'd11: HEX4 = HEX_11;
      4'd12: HEX4 = HEX_12;
      4'd13: HEX4 = HEX_13;
      4'd14: HEX4 = HEX_14;
      4'd15: HEX4 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[6]) begin
    case(cout_reg_06[19:16])
      4'd0: HEX4 = HEX_0;
      4'd1: HEX4 = HEX_1;
      4'd2: HEX4 = HEX_2;
      4'd3: HEX4 = HEX_3;
      4'd4: HEX4 = HEX_4;
      4'd5: HEX4 = HEX_5;
      4'd6: HEX4 = HEX_6;
      4'd7: HEX4 = HEX_7;
      4'd8: HEX4 = HEX_8;
      4'd9: HEX4 = HEX_9;
      4'd10: HEX4 = HEX_10;
      4'd11: HEX4 = HEX_11;
      4'd12: HEX4 = HEX_12;
      4'd13: HEX4 = HEX_13;
      4'd14: HEX4 = HEX_14;
      4'd15: HEX4 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[7]) begin
    case(cout_reg_07[19:16])
      4'd0: HEX4 = HEX_0;
      4'd1: HEX4 = HEX_1;
      4'd2: HEX4 = HEX_2;
      4'd3: HEX4 = HEX_3;
      4'd4: HEX4 = HEX_4;
      4'd5: HEX4 = HEX_5;
      4'd6: HEX4 = HEX_6;
      4'd7: HEX4 = HEX_7;
      4'd8: HEX4 = HEX_8;
      4'd9: HEX4 = HEX_9;
      4'd10: HEX4 = HEX_10;
      4'd11: HEX4 = HEX_11;
      4'd12: HEX4 = HEX_12;
      4'd13: HEX4 = HEX_13;
      4'd14: HEX4 = HEX_14;
      4'd15: HEX4 = HEX_15;
    endcase
  end
  else begin
    HEX4 = OFF;
  end
end

always @(*) begin
  if (state == DONE & SW[0]) begin
    case(cout_reg_00[23:20])
      4'd0: HEX5 = HEX_0;
	   4'd1: HEX5 = HEX_1;
	   4'd2: HEX5 = HEX_2;
	   4'd3: HEX5 = HEX_3;
	   4'd4: HEX5 = HEX_4;
	   4'd5: HEX5 = HEX_5;
	   4'd6: HEX5 = HEX_6;
	   4'd7: HEX5 = HEX_7;
	   4'd8: HEX5 = HEX_8;
	   4'd9: HEX5 = HEX_9;
	   4'd10: HEX5 = HEX_10;
	   4'd11: HEX5 = HEX_11;
	   4'd12: HEX5 = HEX_12;
	   4'd13: HEX5 = HEX_13;
	   4'd14: HEX5 = HEX_14;
	   4'd15: HEX5 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[1]) begin
    case(cout_reg_01[23:20])
      4'd0: HEX5 = HEX_0;
	   4'd1: HEX5 = HEX_1;
	   4'd2: HEX5 = HEX_2;
	   4'd3: HEX5 = HEX_3;
	   4'd4: HEX5 = HEX_4;
	   4'd5: HEX5 = HEX_5;
	   4'd6: HEX5 = HEX_6;
	   4'd7: HEX5 = HEX_7;
	   4'd8: HEX5 = HEX_8;
	   4'd9: HEX5 = HEX_9;
	   4'd10: HEX5 = HEX_10;
	   4'd11: HEX5 = HEX_11;
	   4'd12: HEX5 = HEX_12;
	   4'd13: HEX5 = HEX_13;
	   4'd14: HEX5 = HEX_14;
	   4'd15: HEX5 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[2]) begin
    case(cout_reg_02[23:20])
      4'd0: HEX5 = HEX_0;
	   4'd1: HEX5 = HEX_1;
	   4'd2: HEX5 = HEX_2;
	   4'd3: HEX5 = HEX_3;
	   4'd4: HEX5 = HEX_4;
	   4'd5: HEX5 = HEX_5;
	   4'd6: HEX5 = HEX_6;
	   4'd7: HEX5 = HEX_7;
	   4'd8: HEX5 = HEX_8;
	   4'd9: HEX5 = HEX_9;
	   4'd10: HEX5 = HEX_10;
	   4'd11: HEX5 = HEX_11;
	   4'd12: HEX5 = HEX_12;
	   4'd13: HEX5 = HEX_13;
	   4'd14: HEX5 = HEX_14;
	   4'd15: HEX5 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[3]) begin
    case(cout_reg_03[23:20])
      4'd0: HEX5 = HEX_0;
	   4'd1: HEX5 = HEX_1;
	   4'd2: HEX5 = HEX_2;
	   4'd3: HEX5 = HEX_3;
	   4'd4: HEX5 = HEX_4;
	   4'd5: HEX5 = HEX_5;
	   4'd6: HEX5 = HEX_6;
	   4'd7: HEX5 = HEX_7;
	   4'd8: HEX5 = HEX_8;
	   4'd9: HEX5 = HEX_9;
	   4'd10: HEX5 = HEX_10;
	   4'd11: HEX5 = HEX_11;
	   4'd12: HEX5 = HEX_12;
	   4'd13: HEX5 = HEX_13;
	   4'd14: HEX5 = HEX_14;
	   4'd15: HEX5 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[4]) begin
    case(cout_reg_04[23:20])
      4'd0: HEX5 = HEX_0;
	   4'd1: HEX5 = HEX_1;
	   4'd2: HEX5 = HEX_2;
	   4'd3: HEX5 = HEX_3;
	   4'd4: HEX5 = HEX_4;
	   4'd5: HEX5 = HEX_5;
	   4'd6: HEX5 = HEX_6;
	   4'd7: HEX5 = HEX_7;
	   4'd8: HEX5 = HEX_8;
	   4'd9: HEX5 = HEX_9;
	   4'd10: HEX5 = HEX_10;
	   4'd11: HEX5 = HEX_11;
	   4'd12: HEX5 = HEX_12;
	   4'd13: HEX5 = HEX_13;
	   4'd14: HEX5 = HEX_14;
	   4'd15: HEX5 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[5]) begin
    case(cout_reg_05[23:20])
      4'd0: HEX5 = HEX_0;
	   4'd1: HEX5 = HEX_1;
	   4'd2: HEX5 = HEX_2;
	   4'd3: HEX5 = HEX_3;
	   4'd4: HEX5 = HEX_4;
	   4'd5: HEX5 = HEX_5;
	   4'd6: HEX5 = HEX_6;
	   4'd7: HEX5 = HEX_7;
	   4'd8: HEX5 = HEX_8;
	   4'd9: HEX5 = HEX_9;
	   4'd10: HEX5 = HEX_10;
	   4'd11: HEX5 = HEX_11;
	   4'd12: HEX5 = HEX_12;
	   4'd13: HEX5 = HEX_13;
	   4'd14: HEX5 = HEX_14;
	   4'd15: HEX5 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[6]) begin
    case(cout_reg_06[23:20])
      4'd0: HEX5 = HEX_0;
	   4'd1: HEX5 = HEX_1;
	   4'd2: HEX5 = HEX_2;
	   4'd3: HEX5 = HEX_3;
	   4'd4: HEX5 = HEX_4;
	   4'd5: HEX5 = HEX_5;
	   4'd6: HEX5 = HEX_6;
	   4'd7: HEX5 = HEX_7;
	   4'd8: HEX5 = HEX_8;
	   4'd9: HEX5 = HEX_9;
	   4'd10: HEX5 = HEX_10;
	   4'd11: HEX5 = HEX_11;
	   4'd12: HEX5 = HEX_12;
	   4'd13: HEX5 = HEX_13;
	   4'd14: HEX5 = HEX_14;
	   4'd15: HEX5 = HEX_15;
    endcase
  end
  else if (state == DONE & SW[7]) begin
    case(cout_reg_07[23:20])
      4'd0: HEX5 = HEX_0;
	   4'd1: HEX5 = HEX_1;
	   4'd2: HEX5 = HEX_2;
	   4'd3: HEX5 = HEX_3;
	   4'd4: HEX5 = HEX_4;
	   4'd5: HEX5 = HEX_5;
	   4'd6: HEX5 = HEX_6;
	   4'd7: HEX5 = HEX_7;
	   4'd8: HEX5 = HEX_8;
	   4'd9: HEX5 = HEX_9;
	   4'd10: HEX5 = HEX_10;
	   4'd11: HEX5 = HEX_11;
	   4'd12: HEX5 = HEX_12;
	   4'd13: HEX5 = HEX_13;
	   4'd14: HEX5 = HEX_14;
	   4'd15: HEX5 = HEX_15;
    endcase
  end
  else begin
    HEX5 = OFF;
  end
end
endmodule