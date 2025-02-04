module fifo
#(
  parameter DEPTH=8,
  parameter DATA_WIDTH=8
)
(
  input  clk,
  input  rst_n,
  input  rden,
  input  wren,
  input  [DATA_WIDTH-1:0] i_data,
  output reg [DATA_WIDTH-1:0] o_data,
  output reg full,
  output reg empty
);

reg [DATA_WIDTH-1:0] regfile [DEPTH-1:0];
reg [$clog2(DEPTH)-1:0] write_ptr;
reg [$clog2(DEPTH)-1:0] read_ptr;
reg [$clog2(DEPTH)-1:0] write_ptr_plus_one;
reg [$clog2(DEPTH)-1:0] read_ptr_plus_one;

logic [$clog2(DEPTH)-1:0] write_ptr_next;
logic [$clog2(DEPTH)-1:0] read_ptr_next;
logic [DATA_WIDTH-1:0] o_data_next;
logic empty_next;
logic full_next;

integer i;
always_ff @(posedge clk, negedge rst_n) begin
  if (!rst_n) begin
    write_ptr = 0;
    read_ptr = 0;
    o_data = 0;
    empty = 1;
    full = 0;

    for (i = 0; i < DEPTH-1; i++) // Clear the reg file too
      regfile[i] = '0;
  end
  else begin
    if ((wren && !full) || (wren & rden))
      regfile[write_ptr] = i_data;
    write_ptr = write_ptr_next;
    read_ptr = read_ptr_next;
    o_data = o_data_next;
    empty = empty_next;
    full = full_next;
  end
end

assign write_ptr_plus_one = write_ptr + 1;
assign read_ptr_plus_one = read_ptr + 1;

always_comb begin
  if ((wren && !full) || (wren && rden))
    write_ptr_next <= write_ptr + 1;
  else
    write_ptr_next <= write_ptr;

  if (rden && !empty) begin
    read_ptr_next <= read_ptr + 1;
    o_data_next <= regfile[read_ptr];
  end
  else begin
    read_ptr_next <= read_ptr;
    o_data_next <= o_data;
  end

  if (rden && !wren && (read_ptr_plus_one == write_ptr))
    empty_next <= 1;
  else if (wren)
    empty_next <= 0;
  else
    empty_next <= empty;

  if (!rden && wren && (read_ptr == write_ptr_plus_one))
    full_next <= 1;
  else if (rden && !wren)
    full_next <= 0;
  else
    full_next <= full;

end

endmodule