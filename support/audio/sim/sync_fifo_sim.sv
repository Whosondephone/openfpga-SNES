// Behavioural stand-in for the dcfifo-based sync_fifo (simulation only)
module sync_fifo #(parameter WIDTH = 2) (
    input wire clk_write, input wire clk_read, input wire write_en,
    input wire [WIDTH-1:0] data, output reg [WIDTH-1:0] data_s = 0, output reg write_en_s = 0);
  reg [WIDTH-1:0] hold; reg tog = 0; reg [3:0] ts = 0;
  always @(posedge clk_write) if (write_en) begin hold <= data; tog <= ~tog; end
  always @(posedge clk_read) begin
    ts <= {ts[2:0], tog}; write_en_s <= 0;
    if (ts[3] ^ ts[2]) begin data_s <= hold; write_en_s <= 1; end
  end
endmodule
