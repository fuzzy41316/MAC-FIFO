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
    localparam DATA_WIDTH = 8;

    localparam FILLA = 2'd0;
    localparam FILLB = 2'd1;
    localparam EXEC = 2'd2;
    localparam DONE = 2'd3;

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
    parameter HEX_10 = 7'b0001000;	    // ten
    parameter HEX_11 = 7'b0000011;	    // eleven
    parameter HEX_12 = 7'b1000110;	    // twelve
    parameter HEX_13 = 7'b0100001;	    // thirteen
    parameter HEX_14 = 7'b0000110;	    // fourteen
    parameter HEX_15 = 7'b0001110;	    // fifteen
    parameter OFF   = 7'b1111111;		// all off

    //=======================================================
    //  REG/WIRE declarations
    //======================================================

    reg [1:0] state;
    reg [7:0] datain [0:DATA_WIDTH-1];
    reg [23:0] result;
    

    wire rst_n;
    wire rden, wren;
    wire [7:0] full, empty;        //will read in a left to right (always) can check the whole thing is full/empty using end bits
    wire [7:0] dataout [0:DATA_WIDTH-1];

    // Memory input/output signals to be driven from SM
    wire mem_wait, data_valid, mem_read;
    wire [3:0]addr;                 //addr only goes 0 to 8 -> can represent in 4 bits
    
    //TODO: need to parse this output from mem file into respective FIFOS. controlled from SM
    // otherwise, i have no clue how to do this.
    wire [63:0] readdata;
    // TODO: add declarations for wires used by the FIFO, MAC, and memwrapper

    //=======================================================
    //  Module instantiation
    //=======================================================

    vectored_mac_fifo vectored(
        // Inputs
        .clk(CLOCK_50)
        .rst_n(rst_n)
        .En()
        .Clr(),
        .Ain(),
        .Bin(),
        .Cout(),
        .datain(),
        .rden(rden),
        .wren(wren),
        .full(full),
        .empty(empty),
        // Outputs
        .dataout());

    // TODO: instantiate the memory module, and store values into FIFO in SM below
    // TODO: where do we give the memory module the file to read? -> happens through rom.v file
    mem_wrapper memory(
        // Inputs
        .clk(CLOCK_50),
        .reset_n(rst_n),
        .address({28{1'b0}}, addr),         // 32-bit address for 8 rows going to come from SM
        .read(mem_read),                    // Read request
        // Outputs
        .readdata(readdata),                 // 64-bit read data (one row)
        .readdatavalid(data_valid),          // Data valid signal
        .waitrequest(mem_wait)               // Busy signal to indicate logic is processing
    );  

    //=======================================================
    //  Structural coding
    //=======================================================

    assign rst_n = KEY[0];
    assign wren = '((state == FILL) & rst_n);
    assign rden = '(state == EXEC);

    integer j;

    // State machine
    always @(posedge CLOCK_50, negedge rst_n) begin
        if (~rst_n) begin 
            state <= FILL;
            result <= {(DATA_WIDTH*3){1'b0}};
            // clear empty and full references for all FIFO units
            empty <= {DATA_WIDTH{1'b1}}; 
            full <= {DATA_WIDTH{1'b0}}; 
            for (j=0; j<8; j++) begin 
                datain[j] <= {DATA_WIDTH{1'b0}};
            end 
        end
        else begin
            case(state) 
                FILLB: begin 
                    addr = 4'h0;
                    //TODO: single call to mem module with addr 0
                end
                
                // TODO: find a way to inc addr such that we dont need a seperate state
                FILLA: begin 
                    if (full[7]) begin
                        state <= EXEC;
                    end
                    else begin 
                        //TODO: mutiple calls to mem module starting at addr 1, ending at addr 8
                    end
                end

                EXEC: begin 
                    if (empty[7]) 
                        state <= DONE;
                end

                DONE: begin 

                end
            endcase

        // TODO: Input data to memory from input_mem.mif
            // B address is 0, A address starts at 1 and goes to 8 (hex)


        // TODO: Read data from memory into FIFO

        // TODO: Perform MAC on data from FIFO
        end
    end
endmodule