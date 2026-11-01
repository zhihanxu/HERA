`default_nettype none
`timescale 1ns/1ps
//------------------------------------------------------------------------------
// hbm_bw_ctrl: AXI4-Lite control block for hbm_bw, ap_ctrl_hs.
//   Control/interrupt semantics match the Vitis HLS adapter so XRT drives the
//   kernel natively. Beyond the six arguments it exposes read-only performance
//   counters, which the host reads with xrt::kernel::read_register() (requires
//   a kernel opened for exclusive access).
//
//   0x00 CTRL    bit0 ap_start, bit1 ap_done (COR), bit2 ap_idle,
//                bit3 ap_ready (COR), bit7 auto_restart, bit9 interrupt
//   0x04 GIER    0x08 IP_IER    0x0C IP_ISR
//   -- arguments --
//   0x10 mode             (0 = read, 1 = write, 2 = copy)
//   0x18/0x1C in_ptr      (m00_axi, read)
//   0x24/0x28 out_ptr     (m01_axi, write)
//   0x30 num_beats
//   0x38 burst_len        (beats per burst, power of two, 1..64)
//   0x40 max_outstanding
//   -- counters (read-only) --
//   0x50 cycles      0x54 r_beats   0x58 w_beats   0x5C ar_count
//   0x60 aw_count    0x64 b_count   0x68 r_stall   0x6C w_stall
//   0x70 w_starve    0x74 lat_first 0x78 checksum  0x7C flags
//   0x80 max_out_seen_r             0x84 max_out_seen_w
//------------------------------------------------------------------------------
module hbm_bw_ctrl #(
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
  // arguments
  output wire [31:0]                     mode,
  output wire [63:0]                     in_ptr,
  output wire [63:0]                     out_ptr,
  output wire [31:0]                     num_beats,
  output wire [31:0]                     burst_len,
  output wire [31:0]                     max_outstanding,
  // counters
  input  wire [31:0]                     cnt_cycles,
  input  wire [31:0]                     cnt_r_beats,
  input  wire [31:0]                     cnt_w_beats,
  input  wire [31:0]                     cnt_ar,
  input  wire [31:0]                     cnt_aw,
  input  wire [31:0]                     cnt_b,
  input  wire [31:0]                     cnt_r_stall,
  input  wire [31:0]                     cnt_w_stall,
  input  wire [31:0]                     cnt_w_starve,
  input  wire [31:0]                     cnt_lat_first,
  input  wire [31:0]                     cnt_checksum,
  input  wire [31:0]                     cnt_flags,
  input  wire [31:0]                     cnt_max_out_r,
  input  wire [31:0]                     cnt_max_out_w
);

localparam integer ADDR_AP_CTRL = 'h00;
localparam integer ADDR_GIE     = 'h04;
localparam integer ADDR_IER     = 'h08;
localparam integer ADDR_ISR     = 'h0C;
localparam integer ADDR_MODE    = 'h10;
localparam integer ADDR_IN_L    = 'h18;
localparam integer ADDR_IN_H    = 'h1C;
localparam integer ADDR_OUT_L   = 'h24;
localparam integer ADDR_OUT_H   = 'h28;
localparam integer ADDR_NBEATS  = 'h30;
localparam integer ADDR_BLEN    = 'h38;
localparam integer ADDR_MAXOUT  = 'h40;

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
reg  [31:0] int_mode            = 32'b0;
reg  [63:0] int_in_ptr          = 64'b0;
reg  [63:0] int_out_ptr         = 64'b0;
reg  [31:0] int_num_beats       = 32'b0;
reg  [31:0] int_burst_len       = 32'b0;
reg  [31:0] int_max_out         = 32'b0;

wire        aw_hs = AWVALID & AWREADY;
wire        w_hs  = WVALID  & WREADY;
wire        ar_hs = ARVALID & ARREADY;
wire [31:0] wmask = {{8{WSTRB[3]}}, {8{WSTRB[2]}}, {8{WSTRB[1]}}, {8{WSTRB[0]}}};

wire auto_restart_done = auto_restart_status && (ap_idle && !int_ap_idle);
wire task_ap_done      = (ap_done && !auto_restart_status) || auto_restart_done;
wire task_ap_ready     = ap_ready && !int_auto_restart;

//------------------------ write channel ---------------------------------------
assign AWREADY = (wstate == WRIDLE);
assign WREADY  = (wstate == WRDATA);
assign BVALID  = (wstate == WRRESP);
assign BRESP   = 2'b00;

always @(posedge ACLK) begin
  if (ARESET) wstate <= WRRESET;
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
  if (aw_hs) waddr <= AWADDR;
end

//------------------------ read channel ----------------------------------------
assign ARREADY = (rstate == RDIDLE);
assign RVALID  = (rstate == RDDATA);
assign RDATA   = rdata;
assign RRESP   = 2'b00;

always @(posedge ACLK) begin
  if (ARESET) rstate <= RDRESET;
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
      ADDR_AP_CTRL: rdata <= {22'b0, int_interrupt, 1'b0, int_auto_restart, 3'b0,
                              int_ap_ready, int_ap_idle, int_task_ap_done, int_ap_start};
      ADDR_GIE:     rdata <= {31'b0, int_gie};
      ADDR_IER:     rdata <= {30'b0, int_ier};
      ADDR_ISR:     rdata <= {30'b0, int_isr};
      ADDR_MODE:    rdata <= int_mode;
      ADDR_IN_L:    rdata <= int_in_ptr[31:0];
      ADDR_IN_H:    rdata <= int_in_ptr[63:32];
      ADDR_OUT_L:   rdata <= int_out_ptr[31:0];
      ADDR_OUT_H:   rdata <= int_out_ptr[63:32];
      ADDR_NBEATS:  rdata <= int_num_beats;
      ADDR_BLEN:    rdata <= int_burst_len;
      ADDR_MAXOUT:  rdata <= int_max_out;
      'h50:         rdata <= cnt_cycles;
      'h54:         rdata <= cnt_r_beats;
      'h58:         rdata <= cnt_w_beats;
      'h5C:         rdata <= cnt_ar;
      'h60:         rdata <= cnt_aw;
      'h64:         rdata <= cnt_b;
      'h68:         rdata <= cnt_r_stall;
      'h6C:         rdata <= cnt_w_stall;
      'h70:         rdata <= cnt_w_starve;
      'h74:         rdata <= cnt_lat_first;
      'h78:         rdata <= cnt_checksum;
      'h7C:         rdata <= cnt_flags;
      'h80:         rdata <= cnt_max_out_r;
      'h84:         rdata <= cnt_max_out_w;
      default:      rdata <= 32'b0;
    endcase
  end
end

//------------------------ control / status ------------------------------------
assign ap_start        = int_ap_start;
assign interrupt       = int_interrupt;
assign mode            = int_mode;
assign in_ptr          = int_in_ptr;
assign out_ptr         = int_out_ptr;
assign num_beats       = int_num_beats;
assign burst_len       = int_burst_len;
assign max_outstanding = int_max_out;

always @(posedge ACLK) begin
  if (ARESET)                                                         int_ap_start <= 1'b0;
  else if (w_hs && waddr == ADDR_AP_CTRL && WSTRB[0] && WDATA[0])     int_ap_start <= 1'b1;
  else if (ap_ready)                                                  int_ap_start <= int_auto_restart;
end

always @(posedge ACLK) begin
  if (ARESET)                                      int_task_ap_done <= 1'b0;
  else if (task_ap_done)                           int_task_ap_done <= 1'b1;
  else if (ar_hs && ARADDR == ADDR_AP_CTRL)        int_task_ap_done <= 1'b0;
end

always @(posedge ACLK) begin
  if (ARESET) int_ap_idle <= 1'b0;
  else        int_ap_idle <= ap_idle;
end

always @(posedge ACLK) begin
  if (ARESET)                                int_ap_ready <= 1'b0;
  else if (task_ap_ready)                    int_ap_ready <= 1'b1;
  else if (ar_hs && ARADDR == ADDR_AP_CTRL)  int_ap_ready <= 1'b0;
end

always @(posedge ACLK) begin
  if (ARESET)                                              int_auto_restart <= 1'b0;
  else if (w_hs && waddr == ADDR_AP_CTRL && WSTRB[0])      int_auto_restart <= WDATA[7];
end

always @(posedge ACLK) begin
  if (ARESET)                 auto_restart_status <= 1'b0;
  else if (int_auto_restart)  auto_restart_status <= 1'b1;
  else if (ap_idle)           auto_restart_status <= 1'b0;
end

always @(posedge ACLK) begin
  if (ARESET)                                      int_gie <= 1'b0;
  else if (w_hs && waddr == ADDR_GIE && WSTRB[0])  int_gie <= WDATA[0];
end

always @(posedge ACLK) begin
  if (ARESET)                                      int_ier <= 2'b0;
  else if (w_hs && waddr == ADDR_IER && WSTRB[0])  int_ier <= WDATA[1:0];
end

always @(posedge ACLK) begin
  if (ARESET)                                      int_isr[0] <= 1'b0;
  else if (int_ier[0] && ap_done)                  int_isr[0] <= 1'b1;
  else if (w_hs && waddr == ADDR_ISR && WSTRB[0])  int_isr[0] <= int_isr[0] ^ WDATA[0];
end

always @(posedge ACLK) begin
  if (ARESET)                                      int_isr[1] <= 1'b0;
  else if (int_ier[1] && ap_ready)                 int_isr[1] <= 1'b1;
  else if (w_hs && waddr == ADDR_ISR && WSTRB[0])  int_isr[1] <= int_isr[1] ^ WDATA[1];
end

always @(posedge ACLK) begin
  if (ARESET) int_interrupt <= 1'b0;
  else        int_interrupt <= int_gie && (|int_isr);
end

//------------------------ argument registers ----------------------------------
always @(posedge ACLK) begin
  if (ARESET) begin
    int_mode      <= 32'b0;
    int_in_ptr    <= 64'b0;
    int_out_ptr   <= 64'b0;
    int_num_beats <= 32'b0;
    int_burst_len <= 32'b0;
    int_max_out   <= 32'b0;
  end else if (w_hs) begin
    case (waddr)
      ADDR_MODE:   int_mode            <= (WDATA & wmask) | (int_mode & ~wmask);
      ADDR_IN_L:   int_in_ptr[31:0]    <= (WDATA & wmask) | (int_in_ptr[31:0] & ~wmask);
      ADDR_IN_H:   int_in_ptr[63:32]   <= (WDATA & wmask) | (int_in_ptr[63:32] & ~wmask);
      ADDR_OUT_L:  int_out_ptr[31:0]   <= (WDATA & wmask) | (int_out_ptr[31:0] & ~wmask);
      ADDR_OUT_H:  int_out_ptr[63:32]  <= (WDATA & wmask) | (int_out_ptr[63:32] & ~wmask);
      ADDR_NBEATS: int_num_beats       <= (WDATA & wmask) | (int_num_beats & ~wmask);
      ADDR_BLEN:   int_burst_len       <= (WDATA & wmask) | (int_burst_len & ~wmask);
      ADDR_MAXOUT: int_max_out         <= (WDATA & wmask) | (int_max_out & ~wmask);
      default:     ;
    endcase
  end
end

endmodule
`default_nettype wire
