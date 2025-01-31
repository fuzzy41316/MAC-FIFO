module minilab1_2(
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
    

    // Internal wires
    /* SHARED */
    logic [7:0] datain 
    logic clk, rst_n, clr;

    /* MEMORY */
    logic [31:0] address;
    logic [63:0] readdata;
    logic readdatavalid, read;

    /* A ARRAY */
    logic [7:0] dataoutA [7:0];                 // Row of the 2D A array stored into the FIFO
    logic [7:0] rdenA, wrenA, emptyA, fullA;    // Control signals for each row inserted into 8 FIFOs

    /* B ARRAY */
    logic [7:0] dataoutB;                 // B array 
    logic rdenB, wrenB, emptyB, fullB;          // Control signals for the B FIFO
    logic emptyB, fullB;


    // Instantiate the memory module
    mem_wrapper memory(
        .clk(CLOCK_50),    
        .reset_n(rst_n),
        .address(address),              // 32-bit address for 8 rows
        .read(read),                    // Read request
        // Outputs
        .readdata(readdata),            // 64-bit read data (one row)
        .readdatavalid(readdatavalid),  // Data valid signal
        .waitrequest()                  // Busy signal to indicate logic is processing
    );


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

    // State and next state logic
    typedef enum reg [1:0] {FILLA, FILLB, WAIT, DONE} state_t;
    state_t state, next_state;

    always_ff @(posedge clk, negedge rst_n) 
    begin
        if (!rst_n)
            state <= FILLA;
        else 
            state <= next_state;
    end

    // Structural coding
    assign rst_n = KEY[0];

    logic [3:0] column;
    logic [3:0] row;
    // State machine
    always_comb
    begin
        // Defaults
        read = '0;
        next_state = state;

        if (!rst_n)
        begin
            // Reset the FIFOs, start filling B first
            next_state = FILLB;
            emptyA = '0;
            emptyB = '0;
            fullA = '0;
            fullB = '0;
            datain = '0;
            addr '0;
            column = '0;
            row = 1'd1;
        end
        else
        begin
            case(state)
                READ:
                begin
                    read = 1'b1;

                    // Need to check for data_valid signal from memory before filling the FIFO
                    if (!data_valid & !fullB)           // Fill B first
                        next_state = FILLB;
                    else if (!data_valid & !(&fullA))   // Fill A
                        next_state = FILLA;
                end
                FILLB:
                begin
                    // As long as B FIFO is not full, keep filling
                    if (!fullB)
                    begin
                        datain = readdata[(8*column) +: 8];
                        column++; 
                        wrenB = 1'b1;
                    end
                    else
                    begin
                        column = '0;
                        wrenB = 1'b0;
                        next_state = READ;
                    end
                end
                FILLA:
                begin
                    if (!fullA[row-1])
                    begin
                        datain = readdata[(8*column) +: 8];
                        column++;
                        wrenA[row-1] = 1'b1;
                    end
                    else if (fullA[row-1] & !(row & 4'b1001))
                    begin
                        row++
                        next_state = read;
                    end
                    else
                        next_state = DONE;
                end
                DONE:
                begin
                    
                end
            endcase
        end

    end
endmodule