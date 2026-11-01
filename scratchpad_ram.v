`default_nettype none
`timescale 1ns/1ps
//------------------------------------------------------------------------------
// scratchpad_ram: on-chip scratchpad (simple dual-port block RAM).
//   Write port: synchronous.  Read port: registered output with read enable,
//   1-cycle latency; rdata holds its value while re is low.
//------------------------------------------------------------------------------
module scratchpad_ram #(
  parameter integer DATA_WIDTH = 512,
  parameter integer DEPTH      = 1024,
  parameter integer ADDR_BITS  = 10
) (
  input  wire                  clk,
  input  wire                  we,
  input  wire [ADDR_BITS-1:0]  waddr,
  input  wire [DATA_WIDTH-1:0] wdata,
  input  wire                  re,
  input  wire [ADDR_BITS-1:0]  raddr,
  output reg  [DATA_WIDTH-1:0] rdata
);

(* ram_style = "block" *)
reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

always @(posedge clk) begin
  if (we)
    mem[waddr] <= wdata;
end

always @(posedge clk) begin
  if (re)
    rdata <= mem[raddr];
end

endmodule
`default_nettype wire
