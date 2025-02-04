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

    /* MEMORY */
    reg [31:0] address;
    logic [63:0] readdata;
    logic readdatavalid, read, waitrequest;
    logic [7:0] readdata_byte [7:0];
    assign readdata_byte[0] = readdata[63:56];
    assign readdata_byte[1] = readdata[55:48];
    assign readdata_byte[2] = readdata[47:40];
    assign readdata_byte[3] = readdata[39:32];
    assign readdata_byte[4] = readdata[31:24];
    assign readdata_byte[5] = readdata[23:16];
    assign readdata_byte[6] = readdata[15:8];
    assign readdata_byte[7] = readdata[7:0];

    /* A ARRAY */
    logic [7:0] dataoutA [7:0];                 // Row of the 2D A array stored into the FIFO
    logic allFull;
    logic [7:0] rdenA, wrenA, emptyA, fullA;
    assign allFull = &fullA;

    /* B ARRAY */
    logic [7:0] dataoutB;                       // B array 
    logic rdenB, wrenB, emptyB, fullB;          // Control signals for the B FIFO

    /* MAC */
    logic [7:0] Ain;
    logic En;
    logic [7:0] Bin;
    logic [23:0] Cout;
    reg [23:0] cout_reg [7:0];
    reg [2:0] mac_count;        // TODO: increment the mac_count depending on the step you're on
                                // i.e., first Cout, then second Cout, then third Cout.

    // Propogate enable
    always_ff @(posedge CLOCK_50, negedge rst_n) begin
        if (!rst_n)
            En <= 0;
        else if (|rdenA & rdenB)
            En <= 1;
    end
    
    // Instantiate the MAC module
    mac mac(
        .clk(CLOCK_50),
        .rst_n(rst_n),
        .En(En),    // Enable MAC when reading from exactly any one A FIFO and exactly one B FIFO
        .Clr(Clr),
        .Ain(Ain),
        .Bin(Bin),
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


    // Generate FIFOs for the 8x8 A input
    genvar k;
    generate 
    for (k = 0; k < 8; k++)  
    begin: A_fifo_gen
        fifo A_fifo(
            .aclr(aclr),
            .data(datain),
            .rdclk(CLOCK_50),
            .rdreq(rdenA[k]),
            .wrclk(CLOCK_50),
            .wrreq(wrenA[k]),
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
                .data(datain),
                .rdclk(CLOCK_50),
                .rdreq(rdenB),
                .wrclk(CLOCK_50),
                .wrreq(wrenB),
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
    end

    // Counter for address, to increment rows
    always_ff @(posedge CLOCK_50, negedge rst_n) begin
        if (!rst_n)
            address <= '0;
        else if (nextrow)
            address <= address + 1;
    end

    // Counter for controlling the otput from MAC
    always_ff @(posedge CLOCK_50, negedge rst_n) begin
        if (!rst_n)
            mac_count <= '0;
        else if (next_cout)
            mac_count <= mac_count + 1;
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
        Clr = 0;
        aclr = 0;
        Ain = '0;
        Bin = '0;

        case(state)
            IDLE:
            begin
                next_state = READ;
                rdenA = '0;
                rdenB = 0;
                aclr = 1;
                Clr = 1;
                for (int i = 0; i < 8; i++)
                    cout_reg[i] = '0;
            end
            READ:
            begin
                read = 1'b1;
                // Need to check for data_valid signal from memory before filling the FIFO
                if (readdatavalid & !fullB)             // Fill B first
                    next_state = FILLB;
                else if (readdatavalid & !(allFull))    // Fill A second
                    next_state = FILLA;
                else if (allFull & fullB)               // Start MAC
                    next_state = EXEC;
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
                else
                begin
                    wrenB = 0;
                    next_state = READ;
                    nextrow = 1;
                end
            end
            FILLA:
            begin
                if(!allFull & ~waitrequest)
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
                else if (allFull & ~waitrequest) begin
                    wrenA = '0;
                    next_state = EXEC;
                end
            end
            WAIT:
            begin
                rdenB = 0;
                rdenA = '0;
                next_state = EXEC;
            end
            EXEC:
            begin
                // Need to pop 1 entry from B FIFO and 1 entry from A FIFO
                // Need to refill the B FIFO, A.K.A propogate through
                // Pop the FIFO, and wait till its not full to write back to it, which should be the value popped
                
                // When the current A FIFO is empty, move to the next cout for MAC
                if (emptyA[mac_count]) begin
                    cout_reg[mac_count] = Cout;     // Output from mac is a wire, need to store that output in a register
                    Clr = 1;
                    next_cout = 1;

                    // Still need to do Bin and Ain for the MAC
                    datain = dataoutB;
                    reading = 1;
                    wrenB = 1;
                    Bin = dataoutB;
                    Ain = dataoutA[mac_count];
                    next_state = WAIT;
                end
                else begin
                    if (fullB) begin
                        rdenA |= (1 << (mac_count));    // Activate the ith A FIFO
                        rdenB = 1;
                        next_state = WAIT;
                    end
                    else begin
                        datain = dataoutB;
                        reading = 1;
                        wrenB = 1;
                        Bin = dataoutB;
                        Ain = dataoutA[mac_count];      // Ain input of the MAC from the ith A FIFO
                        next_state = WAIT;
                    end
                end

                // When the A FIFOs are exhausted, stop the MAC unit
                if (&emptyA)
                    next_state = DONE;
            end
            // DONE STATE
            default:
            begin
                // TODO

            end
        endcase
    end
endmodule