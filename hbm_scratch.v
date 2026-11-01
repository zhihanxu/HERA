`default_nettype none
`timescale 1ns/1ps
//------------------------------------------------------------------------------
// hbm_scratch: Vitis RTL kernel top level.
//   ap_clk / ap_rst_n     : kernel clock and active-low reset from the platform
//   s_axi_control         : AXI4-Lite control/argument registers (ap_ctrl_hs)
//   m00_axi               : AXI4 master to HBM
//   interrupt             : ap_done interrupt
// Kernel call signature (XRT):  hbm_scratch(uint num_beats, int* in, int* out)
//------------------------------------------------------------------------------
module hbm_scratch #(
  parameter integer C_S_AXI_CONTROL_ADDR_WIDTH = 12,
  parameter integer C_S_AXI_CONTROL_DATA_WIDTH = 32,
  parameter integer C_M00_AXI_ADDR_WIDTH       = 64,
  parameter integer C_M00_AXI_DATA_WIDTH       = 512
) (
  input  wire                                    ap_clk,
  input  wire                                    ap_rst_n,
  // AXI4 master interface m00_axi
  output wire                                    m00_axi_awvalid,
  input  wire                                    m00_axi_awready,
  output wire [C_M00_AXI_ADDR_WIDTH-1:0]         m00_axi_awaddr,
  output wire [7:0]                              m00_axi_awlen,
  output wire                                    m00_axi_wvalid,
  input  wire                                    m00_axi_wready,
  output wire [C_M00_AXI_DATA_WIDTH-1:0]         m00_axi_wdata,
  output wire [C_M00_AXI_DATA_WIDTH/8-1:0]       m00_axi_wstrb,
  output wire                                    m00_axi_wlast,
  input  wire                                    m00_axi_bvalid,
  output wire                                    m00_axi_bready,
  output wire                                    m00_axi_arvalid,
  input  wire                                    m00_axi_arready,
  output wire [C_M00_AXI_ADDR_WIDTH-1:0]         m00_axi_araddr,
  output wire [7:0]                              m00_axi_arlen,
  input  wire                                    m00_axi_rvalid,
  output wire                                    m00_axi_rready,
  input  wire [C_M00_AXI_DATA_WIDTH-1:0]         m00_axi_rdata,
  input  wire                                    m00_axi_rlast,
  // AXI4-Lite slave interface s_axi_control
  input  wire                                    s_axi_control_awvalid,
  output wire                                    s_axi_control_awready,
  input  wire [C_S_AXI_CONTROL_ADDR_WIDTH-1:0]   s_axi_control_awaddr,
  input  wire                                    s_axi_control_wvalid,
  output wire                                    s_axi_control_wready,
  input  wire [C_S_AXI_CONTROL_DATA_WIDTH-1:0]   s_axi_control_wdata,
  input  wire [C_S_AXI_CONTROL_DATA_WIDTH/8-1:0] s_axi_control_wstrb,
  input  wire                                    s_axi_control_arvalid,
  output wire                                    s_axi_control_arready,
  input  wire [C_S_AXI_CONTROL_ADDR_WIDTH-1:0]   s_axi_control_araddr,
  output wire                                    s_axi_control_rvalid,
  input  wire                                    s_axi_control_rready,
  output wire [C_S_AXI_CONTROL_DATA_WIDTH-1:0]   s_axi_control_rdata,
  output wire [1:0]                              s_axi_control_rresp,
  output wire                                    s_axi_control_bvalid,
  input  wire                                    s_axi_control_bready,
  output wire [1:0]                              s_axi_control_bresp,
  output wire                                    interrupt
);

localparam integer SCRATCH_DEPTH = 1024;  // scratchpad depth in beats
localparam integer MAX_BURST     = 64;    // must match MAX_BURST_LENGTH in packaging

reg areset = 1'b1;
always @(posedge ap_clk) begin
  areset <= ~ap_rst_n;
end

wire        ap_start;
wire        ap_idle;
wire        ap_done;
wire        ap_ready;
wire [31:0] num_beats;
wire [63:0] in_ptr;
wire [63:0] out_ptr;

hbm_scratch_ctrl #(
  .C_S_AXI_ADDR_WIDTH ( C_S_AXI_CONTROL_ADDR_WIDTH ),
  .C_S_AXI_DATA_WIDTH ( C_S_AXI_CONTROL_DATA_WIDTH )
) u_ctrl (
  .ACLK      ( ap_clk                ),
  .ARESET    ( areset                ),
  .AWADDR    ( s_axi_control_awaddr  ),
  .AWVALID   ( s_axi_control_awvalid ),
  .AWREADY   ( s_axi_control_awready ),
  .WDATA     ( s_axi_control_wdata   ),
  .WSTRB     ( s_axi_control_wstrb   ),
  .WVALID    ( s_axi_control_wvalid  ),
  .WREADY    ( s_axi_control_wready  ),
  .BRESP     ( s_axi_control_bresp   ),
  .BVALID    ( s_axi_control_bvalid  ),
  .BREADY    ( s_axi_control_bready  ),
  .ARADDR    ( s_axi_control_araddr  ),
  .ARVALID   ( s_axi_control_arvalid ),
  .ARREADY   ( s_axi_control_arready ),
  .RDATA     ( s_axi_control_rdata   ),
  .RRESP     ( s_axi_control_rresp   ),
  .RVALID    ( s_axi_control_rvalid  ),
  .RREADY    ( s_axi_control_rready  ),
  .interrupt ( interrupt             ),
  .ap_start  ( ap_start              ),
  .ap_done   ( ap_done               ),
  .ap_ready  ( ap_ready              ),
  .ap_idle   ( ap_idle               ),
  .num_beats ( num_beats             ),
  .in_ptr    ( in_ptr                ),
  .out_ptr   ( out_ptr               )
);

hbm_scratch_core #(
  .ADDR_WIDTH ( C_M00_AXI_ADDR_WIDTH ),
  .DATA_WIDTH ( C_M00_AXI_DATA_WIDTH ),
  .DEPTH      ( SCRATCH_DEPTH        ),
  .MAX_BURST  ( MAX_BURST            )
) u_core (
  .clk           ( ap_clk          ),
  .areset        ( areset          ),
  .ap_start      ( ap_start        ),
  .ap_idle       ( ap_idle         ),
  .ap_done       ( ap_done         ),
  .ap_ready      ( ap_ready        ),
  .num_beats     ( num_beats       ),
  .in_ptr        ( in_ptr          ),
  .out_ptr       ( out_ptr         ),
  .m_axi_arvalid ( m00_axi_arvalid ),
  .m_axi_arready ( m00_axi_arready ),
  .m_axi_araddr  ( m00_axi_araddr  ),
  .m_axi_arlen   ( m00_axi_arlen   ),
  .m_axi_rvalid  ( m00_axi_rvalid  ),
  .m_axi_rready  ( m00_axi_rready  ),
  .m_axi_rdata   ( m00_axi_rdata   ),
  .m_axi_rlast   ( m00_axi_rlast   ),
  .m_axi_awvalid ( m00_axi_awvalid ),
  .m_axi_awready ( m00_axi_awready ),
  .m_axi_awaddr  ( m00_axi_awaddr  ),
  .m_axi_awlen   ( m00_axi_awlen   ),
  .m_axi_wvalid  ( m00_axi_wvalid  ),
  .m_axi_wready  ( m00_axi_wready  ),
  .m_axi_wdata   ( m00_axi_wdata   ),
  .m_axi_wstrb   ( m00_axi_wstrb   ),
  .m_axi_wlast   ( m00_axi_wlast   ),
  .m_axi_bvalid  ( m00_axi_bvalid  ),
  .m_axi_bready  ( m00_axi_bready  )
);

endmodule
`default_nettype wire
