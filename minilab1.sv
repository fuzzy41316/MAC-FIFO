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

    localparam FILL = 2'd0;
    localparam EXEC = 2'd1;
    localparam DONE = 2'd2;

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
    wire rden, wren, full, empty;
    wire [7:0] dataout [0:DATA_WIDTH-1];

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
        .rden(),
        .wren(),
        .full(),
        .empty(),
        // Outputs
        .dataout());

    // TODO: instantiate the memory module, and store values into FIFO in SM below

    memwrapper memory(
        // Inputs
        .clk(CLOCK_50),
        .reset_n(rst_n),
        .address(),         // 32-bit address for 8 rows
        .read(),            // Read request
        // Outputs
        .readdata(),        // 64-bit read data (one row)
        .readdatavalid(),   // Data valid signal
        .waitrequest()      // Busy signal to indicate logic is processing
    );

    //=======================================================
    //  Structural coding
    //=======================================================

    assign rst_n = KEY[0];
    assign wren = '((state == FILL) & rst_n);
    assign rden = '(state == EXEC);

    integer j;

    // State machine
    always @(posedge CLOCK_50, negedge rst_n) 
    begin
        // TODO: Input data to memory from input_mem.mif

        // TODO: Read data from memory into FIFO

        // TODO: Perform MAC on data from FIFO
    end
endmodule