`default_nettype none
`timescale 1ns/1ps
//------------------------------------------------------------------------------
// hbm_scratch_ctrl: AXI4-Lite control/argument register block, ap_ctrl_hs.
//   Register semantics follow the Vitis HLS s_axilite adapter so XRT can drive
//   the kernel exactly like an HLS kernel.
//
//   0x00 CTRL     bit0 ap_start (R/W, cleared on ap_ready unless auto_restart)
//                 bit1 ap_done  (R, clear on read)   bit2 ap_idle (R)
//                 bit3 ap_ready (R, clear on read)   bit7 auto_restart (R/W)
//                 bit9 interrupt (R)
//   0x04 GIE      bit0 global interrupt enable
//   0x08 IER      bit0 ap_done irq enable, bit1 ap_ready irq enable
//   0x0C ISR      bit0 ap_done irq status, bit1 ap_ready irq status (toggle on write)
//   0x10          num_beats[31:0]            arg 0
//   0x18 / 0x1C   in_ptr[31:0] / [63:32]     arg 1 (m00_axi)
//   0x24 / 0x28   out_ptr[31:0] / [63:32]    arg 2 (m00_axi)
//------------------------------------------------------------------------------
module hbm_scratch_ctrl #(
  parameter integer C_S_AXI_ADDR_WIDTH = 12,
  parameter integer C_S_AXI_DATA_WIDTH = 32
) (
  input  wire                            ACLK,
  input  wire                            ARESET,
  input  wire [C_S_AXI_ADDR_WIDTH-1:0]   AWADDR,
  input  wire                            AWVALID,
  output wire                            AWREADY,
  input  wire [C_S_AXI_DATA_WIDTH-1:0]   WDATA,
  input  wire [C_S_AXI_DATA_WIDTH/8-1:0] WSTRB,
  input  wire                            WVALID,
  output wire                            WREADY,
  output wire [1:0]                      BRESP,
  output wire                            BVALID,
  input  wire                            BREADY,
  input  wire [C_S_AXI_ADDR_WIDTH-1:0]   ARADDR,
  input  wire                            ARVALID,
  output wire                            ARREADY,
  output wire [C_S_AXI_DATA_WIDTH-1:0]   RDATA,
  output wire [1:0]                      RRESP,
  output wire                            RVALID,
  input  wire                            RREADY,
  output wire                            interrupt,
  // kernel handshake
  output wire                            ap_start,
  input  wire                            ap_done,
  input  wire                            ap_ready,
  input  wire                            ap_idle,
  // kernel arguments
  output wire [31:0]                     num_beats,
  output wire [63:0]                     in_ptr,
  output wire [63:0]                     out_ptr
);

localparam integer ADDR_AP_CTRL     = 'h00;
localparam integer ADDR_GIE         = 'h04;
localparam integer ADDR_IER         = 'h08;
localparam integer ADDR_ISR         = 'h0C;
localparam integer ADDR_NUM_BEATS   = 'h10;
localparam integer ADDR_IN_PTR_L    = 'h18;
localparam integer ADDR_IN_PTR_H    = 'h1C;
localparam integer ADDR_OUT_PTR_L   = 'h24;
localparam integer ADDR_OUT_PTR_H   = 'h28;

localparam [1:0] WRIDLE = 2'd0, WRDATA = 2'd1, WRRESP = 2'd2, WRRESET = 2'd3;
localparam [1:0] RDIDLE = 2'd0, RDDATA = 2'd1, RDRESET = 2'd2;

reg  [1:0]                    wstate = WRRESET;
reg  [C_S_AXI_ADDR_WIDTH-1:0] waddr;
reg  [1:0]                    rstate = RDRESET;
reg  [31:0]                   rdata;

reg         int_ap_start        = 1'b0;
reg         int_task_ap_done    = 1'b0;
reg         int_ap_idle         = 1'b0;
reg         int_ap_ready        = 1'b0;
reg         int_auto_restart    = 1'b0;
reg         auto_restart_status = 1'b0;
reg         int_interrupt       = 1'b0;
reg         int_gie             = 1'b0;
reg  [1:0]  int_ier             = 2'b0;
reg  [1:0]  int_isr             = 2'b0;
reg  [31:0] int_num_beats       = 32'b0;
reg  [63:0] int_in_ptr          = 64'b0;
reg  [63:0] int_out_ptr         = 64'b0;

wire        aw_hs = AWVALID & AWREADY;
wire        w_hs  = WVALID & WREADY;
wire        ar_hs = ARVALID & ARREADY;
wire [31:0] wmask = {{8{WSTRB[3]}}, {8{WSTRB[2]}}, {8{WSTRB[1]}}, {8{WSTRB[0]}}};

wire auto_restart_done = auto_restart_status && (ap_idle && !int_ap_idle);
wire task_ap_done      = (ap_done && !auto_restart_status) || auto_restart_done;
wire task_ap_ready     = ap_ready && !int_auto_restart;

//------------------------ AXI4-Lite write channel ------------------------------
assign AWREADY = (wstate == WRIDLE);
assign WREADY  = (wstate == WRDATA);
assign BVALID  = (wstate == WRRESP);
assign BRESP   = 2'b00;

always @(posedge ACLK) begin
  if (ARESET)
    wstate <= WRRESET;
  else begin
    case (wstate)
      WRIDLE:  if (AWVALID) wstate <= WRDATA;
      WRDATA:  if (WVALID)  wstate <= WRRESP;
      WRRESP:  if (BREADY)  wstate <= WRIDLE;
      default: wstate <= WRIDLE;
    endcase
  end
end

always @(posedge ACLK) begin
  if (aw_hs)
    waddr <= AWADDR;
end

//------------------------ AXI4-Lite read channel -------------------------------
assign ARREADY = (rstate == RDIDLE);
assign RVALID  = (rstate == RDDATA);
assign RDATA   = rdata;
assign RRESP   = 2'b00;

always @(posedge ACLK) begin
  if (ARESET)
    rstate <= RDRESET;
  else begin
    case (rstate)
      RDIDLE:  if (ARVALID) rstate <= RDDATA;
      RDDATA:  if (RREADY)  rstate <= RDIDLE;
      default: rstate <= RDIDLE;
    endcase
  end
end

always @(posedge ACLK) begin
  if (ar_hs) begin
    case (ARADDR)
      ADDR_AP_CTRL:   rdata <= {22'b0, int_interrupt, 1'b0, int_auto_restart, 3'b0,
                                int_ap_ready, int_ap_idle, int_task_ap_done, int_ap_start};
      ADDR_GIE:       rdata <= {31'b0, int_gie};
      ADDR_IER:       rdata <= {30'b0, int_ier};
      ADDR_ISR:       rdata <= {30'b0, int_isr};
      ADDR_NUM_BEATS: rdata <= int_num_beats;
      ADDR_IN_PTR_L:  rdata <= int_in_ptr[31:0];
      ADDR_IN_PTR_H:  rdata <= int_in_ptr[63:32];
      ADDR_OUT_PTR_L: rdata <= int_out_ptr[31:0];
      ADDR_OUT_PTR_H: rdata <= int_out_ptr[63:32];
      default:        rdata <= 32'b0;
    endcase
  end
end

//------------------------ Control / status registers ---------------------------
assign ap_start  = int_ap_start;
assign interrupt = int_interrupt;
assign num_beats = int_num_beats;
assign in_ptr    = int_in_ptr;
assign out_ptr   = int_out_ptr;

always @(posedge ACLK) begin
  if (ARESET)
    int_ap_start <= 1'b0;
  else if (w_hs && waddr == ADDR_AP_CTRL && WSTRB[0] && WDATA[0])
    int_ap_start <= 1'b1;
  else if (ap_ready)
    int_ap_start <= int_auto_restart;
end

always @(posedge ACLK) begin
  if (ARESET)
    int_task_ap_done <= 1'b0;
  else if (task_ap_done)
    int_task_ap_done <= 1'b1;
  else if (ar_hs && ARADDR == ADDR_AP_CTRL)
    int_task_ap_done <= 1'b0;
end

always @(posedge ACLK) begin
  if (ARESET)
    int_ap_idle <= 1'b0;
  else
    int_ap_idle <= ap_idle;
end

always @(posedge ACLK) begin
  if (ARESET)
    int_ap_ready <= 1'b0;
  else if (task_ap_ready)
    int_ap_ready <= 1'b1;
  else if (ar_hs && ARADDR == ADDR_AP_CTRL)
    int_ap_ready <= 1'b0;
end

always @(posedge ACLK) begin
  if (ARESET)
    int_auto_restart <= 1'b0;
  else if (w_hs && waddr == ADDR_AP_CTRL && WSTRB[0])
    int_auto_restart <= WDATA[7];
end

always @(posedge ACLK) begin
  if (ARESET)
    auto_restart_status <= 1'b0;
  else if (int_auto_restart)
    auto_restart_status <= 1'b1;
  else if (ap_idle)
    auto_restart_status <= 1'b0;
end

always @(posedge ACLK) begin
  if (ARESET)
    int_gie <= 1'b0;
  else if (w_hs && waddr == ADDR_GIE && WSTRB[0])
    int_gie <= WDATA[0];
end

always @(posedge ACLK) begin
  if (ARESET)
    int_ier <= 2'b0;
  else if (w_hs && waddr == ADDR_IER && WSTRB[0])
    int_ier <= WDATA[1:0];
end

always @(posedge ACLK) begin
  if (ARESET)
    int_isr[0] <= 1'b0;
  else if (int_ier[0] && ap_done)
    int_isr[0] <= 1'b1;
  else if (w_hs && waddr == ADDR_ISR && WSTRB[0])
    int_isr[0] <= int_isr[0] ^ WDATA[0];
end

always @(posedge ACLK) begin
  if (ARESET)
    int_isr[1] <= 1'b0;
  else if (int_ier[1] && ap_ready)
    int_isr[1] <= 1'b1;
  else if (w_hs && waddr == ADDR_ISR && WSTRB[0])
    int_isr[1] <= int_isr[1] ^ WDATA[1];
end

always @(posedge ACLK) begin
  if (ARESET)
    int_interrupt <= 1'b0;
  else
    int_interrupt <= int_gie && (|int_isr);
end

//------------------------ Argument registers -----------------------------------
always @(posedge ACLK) begin
  if (ARESET)
    int_num_beats <= 32'b0;
  else if (w_hs && waddr == ADDR_NUM_BEATS)
    int_num_beats <= (WDATA & wmask) | (int_num_beats & ~wmask);
end

always @(posedge ACLK) begin
  if (ARESET)
    int_in_ptr <= 64'b0;
  else if (w_hs && waddr == ADDR_IN_PTR_L)
    int_in_ptr[31:0] <= (WDATA & wmask) | (int_in_ptr[31:0] & ~wmask);
  else if (w_hs && waddr == ADDR_IN_PTR_H)
    int_in_ptr[63:32] <= (WDATA & wmask) | (int_in_ptr[63:32] & ~wmask);
end

always @(posedge ACLK) begin
  if (ARESET)
    int_out_ptr <= 64'b0;
  else if (w_hs && waddr == ADDR_OUT_PTR_L)
    int_out_ptr[31:0] <= (WDATA & wmask) | (int_out_ptr[31:0] & ~wmask);
  else if (w_hs && waddr == ADDR_OUT_PTR_H)
    int_out_ptr[63:32] <= (WDATA & wmask) | (int_out_ptr[63:32] & ~wmask);
end

endmodule
`default_nettype wire
