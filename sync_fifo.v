`default_nettype none
`timescale 1ns/1ps
//------------------------------------------------------------------------------
// sync_fifo: synchronous first-word-fall-through FIFO in block RAM.
//   `count` includes the output register, so it is the true occupancy.
//   The producer must not overfill: hbm_bw_read reserves space with a credit
//   counter before issuing each AR. `overflow` is a sticky error flag that
//   should never assert.
//------------------------------------------------------------------------------
module sync_fifo #(
  parameter integer DATA_WIDTH = 512,
  parameter integer DEPTH      = 1024,
  parameter integer ADDR_BITS  = 10,
  parameter integer CNT_BITS   = 16
) (
  input  wire                  clk,
  input  wire                  areset,
  input  wire                  clear,
  input  wire                  wr_en,
  input  wire [DATA_WIDTH-1:0] wr_data,
  input  wire                  rd_en,
  output wire [DATA_WIDTH-1:0] rd_data,
  output wire                  rd_valid,
  output wire [CNT_BITS-1:0]   count,
  output reg                   overflow
);

(* ram_style = "block" *)
reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

reg  [ADDR_BITS-1:0]  wptr;
reg  [ADDR_BITS-1:0]  rptr;
reg  [CNT_BITS-1:0]   mem_cnt;
reg  [DATA_WIDTH-1:0] dout;
reg                   dout_v;

wire mem_rd = (mem_cnt != {CNT_BITS{1'b0}}) && (!dout_v || rd_en);

always @(posedge clk) begin
  if (areset || clear) begin
    wptr     <= {ADDR_BITS{1'b0}};
    rptr     <= {ADDR_BITS{1'b0}};
    mem_cnt  <= {CNT_BITS{1'b0}};
    dout_v   <= 1'b0;
    overflow <= 1'b0;
  end else begin
    if (wr_en) begin
      mem[wptr] <= wr_data;
      wptr      <= wptr + 1'b1;
      if (mem_cnt == DEPTH[CNT_BITS-1:0]) overflow <= 1'b1;
    end
    if (mem_rd) begin
      dout <= mem[rptr];
      rptr <= rptr + 1'b1;
    end
    mem_cnt <= mem_cnt + (wr_en ? 1'b1 : 1'b0) - (mem_rd ? 1'b1 : 1'b0);

    if (mem_rd)      dout_v <= 1'b1;
    else if (rd_en)  dout_v <= 1'b0;
  end
end

assign rd_data  = dout;
assign rd_valid = dout_v;
assign count    = mem_cnt + (dout_v ? 1'b1 : 1'b0);

endmodule
`default_nettype wire
