`default_nettype none
`timescale 1ns/1ps
//------------------------------------------------------------------------------
// hbm_bw: Vitis RTL kernel for measuring per-pseudo-channel HBM bandwidth.
//
//   m00_axi  read-only master  (in_ptr)   -> map to one HBM pseudo-channel
//   m01_axi  write-only master (out_ptr)  -> map to another pseudo-channel
//
//   Modes:  0 read-only   stream in_ptr, fold into a checksum, discard
//           1 write-only  stream a generated pattern (word j == j) to out_ptr
//           2 copy        in_ptr -> FIFO -> out_ptr, streaming, no reversal
//
//   burst_len (beats, power of two <= MAX_BURST) and max_outstanding are
//   runtime arguments, so one hardware build can sweep the whole space.
//
//   Kernel signature (XRT):
//     hbm_bw(uint mode, int* in_ptr, int* out_ptr,
//            uint num_beats, uint burst_len, uint max_outstanding)
//   Performance counters are read back over s_axi_control (see hbm_bw_ctrl.v).
//------------------------------------------------------------------------------
module hbm_bw #(
  parameter integer C_S_AXI_CONTROL_ADDR_WIDTH = 12,
  parameter integer C_S_AXI_CONTROL_DATA_WIDTH = 32,
  parameter integer C_M00_AXI_ADDR_WIDTH       = 64,
  parameter integer C_M00_AXI_DATA_WIDTH       = 512,
  parameter integer C_M01_AXI_ADDR_WIDTH       = 64,
  parameter integer C_M01_AXI_DATA_WIDTH       = 512
) (
  input  wire                                    ap_clk,
  input  wire                                    ap_rst_n,
  // AXI4 master m00_axi: reads
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
  // AXI4 master m01_axi: writes
  output wire                                    m01_axi_awvalid,
  input  wire                                    m01_axi_awready,
  output wire [C_M01_AXI_ADDR_WIDTH-1:0]         m01_axi_awaddr,
  output wire [7:0]                              m01_axi_awlen,
  output wire                                    m01_axi_wvalid,
  input  wire                                    m01_axi_wready,
  output wire [C_M01_AXI_DATA_WIDTH-1:0]         m01_axi_wdata,
  output wire [C_M01_AXI_DATA_WIDTH/8-1:0]       m01_axi_wstrb,
  output wire                                    m01_axi_wlast,
  input  wire                                    m01_axi_bvalid,
  output wire                                    m01_axi_bready,
  output wire                                    m01_axi_arvalid,
  input  wire                                    m01_axi_arready,
  output wire [C_M01_AXI_ADDR_WIDTH-1:0]         m01_axi_araddr,
  output wire [7:0]                              m01_axi_arlen,
  input  wire                                    m01_axi_rvalid,
  output wire                                    m01_axi_rready,
  input  wire [C_M01_AXI_DATA_WIDTH-1:0]         m01_axi_rdata,
  input  wire                                    m01_axi_rlast,
  // AXI4-Lite control
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

localparam integer MAX_BURST    = 64;    // beats; must match MAX_BURST_LENGTH at packaging
localparam integer MAX_OUT_HW   = 32;    // must match NUM_*_OUTSTANDING at packaging
localparam integer FIFO_DEPTH   = 1024;  // beats; >= MAX_OUT_HW * MAX_BURST is ideal
localparam integer FIFO_ABITS   = 10;

reg areset = 1'b1;
always @(posedge ap_clk) begin
  areset <= ~ap_rst_n;
end

//------------------------ control block ---------------------------------------
wire        ap_start;
wire        ap_idle;
wire        ap_done;
wire        ap_ready;
wire [31:0] mode;
wire [63:0] in_ptr;
wire [63:0] out_ptr;
wire [31:0] num_beats;
wire [31:0] burst_len;
wire [31:0] max_outstanding;

reg  [31:0] cycles;
reg  [31:0] flags;

wire [31:0] r_ar_count, r_beats, r_stall, r_lat_first, r_checksum;
wire [31:0] w_aw_count, w_beats, w_b_count, w_stall, w_starve;
wire [7:0]  r_max_out_seen, w_max_out_seen;

hbm_bw_ctrl #(
  .C_S_AXI_ADDR_WIDTH ( C_S_AXI_CONTROL_ADDR_WIDTH ),
  .C_S_AXI_DATA_WIDTH ( C_S_AXI_CONTROL_DATA_WIDTH )
) u_ctrl (
  .ACLK            ( ap_clk                     ),
  .ARESET          ( areset                     ),
  .AWADDR          ( s_axi_control_awaddr       ),
  .AWVALID         ( s_axi_control_awvalid      ),
  .AWREADY         ( s_axi_control_awready      ),
  .WDATA           ( s_axi_control_wdata        ),
  .WSTRB           ( s_axi_control_wstrb        ),
  .WVALID          ( s_axi_control_wvalid       ),
  .WREADY          ( s_axi_control_wready       ),
  .BRESP           ( s_axi_control_bresp        ),
  .BVALID          ( s_axi_control_bvalid       ),
  .BREADY          ( s_axi_control_bready       ),
  .ARADDR          ( s_axi_control_araddr       ),
  .ARVALID         ( s_axi_control_arvalid      ),
  .ARREADY         ( s_axi_control_arready      ),
  .RDATA           ( s_axi_control_rdata        ),
  .RRESP           ( s_axi_control_rresp        ),
  .RVALID          ( s_axi_control_rvalid       ),
  .RREADY          ( s_axi_control_rready       ),
  .interrupt       ( interrupt                  ),
  .ap_start        ( ap_start                   ),
  .ap_done         ( ap_done                    ),
  .ap_ready        ( ap_ready                   ),
  .ap_idle         ( ap_idle                    ),
  .mode            ( mode                       ),
  .in_ptr          ( in_ptr                     ),
  .out_ptr         ( out_ptr                    ),
  .num_beats       ( num_beats                  ),
  .burst_len       ( burst_len                  ),
  .max_outstanding ( max_outstanding            ),
  .cnt_cycles      ( cycles                     ),
  .cnt_r_beats     ( r_beats                    ),
  .cnt_w_beats     ( w_beats                    ),
  .cnt_ar          ( r_ar_count                 ),
  .cnt_aw          ( w_aw_count                 ),
  .cnt_b           ( w_b_count                  ),
  .cnt_r_stall     ( r_stall                    ),
  .cnt_w_stall     ( w_stall                    ),
  .cnt_w_starve    ( w_starve                   ),
  .cnt_lat_first   ( r_lat_first                ),
  .cnt_checksum    ( r_checksum                 ),
  .cnt_flags       ( flags                      ),
  .cnt_max_out_r   ( {24'b0, r_max_out_seen}    ),
  .cnt_max_out_w   ( {24'b0, w_max_out_seen}    )
);

//------------------------ argument validation ---------------------------------
wire [7:0] blen_eff = burst_len[7:0];
wire [7:0] mout_eff = (max_outstanding > MAX_OUT_HW) ? MAX_OUT_HW[7:0] : max_outstanding[7:0];

// Both are constant for the duration of a run, so latch them once at start.
// Feeding the engines straight from the 32-bit control registers put a compare
// and mux on their issue-gating paths (the 300 MHz critical path).
reg [7:0] blen_r;
reg [7:0] mout_r;

wire bad_blen  = (burst_len == 32'd0) || (burst_len > MAX_BURST) ||
                 ((burst_len & (burst_len - 32'd1)) != 32'd0);
wire bad_beats = (num_beats == 32'd0);
wire bad_mode  = (mode > 32'd2);
wire bad_mout  = (max_outstanding == 32'd0);
wire args_bad  = bad_blen || bad_beats || bad_mode || bad_mout;

wire mode_read  = (mode == 32'd0);
wire mode_write = (mode == 32'd1);
wire mode_copy  = (mode == 32'd2);

//------------------------ run sequencer ---------------------------------------
localparam [1:0] S_IDLE = 2'd0, S_RUN = 2'd1, S_DONE = 2'd2;
reg [1:0] state;
reg       run_start;    // pulses for both engines so all counters clear
reg       en_r;
reg       en_w;

wire busy_r;
wire busy_w;
wire [15:0] fifo_count;
wire        fifo_overflow;

assign ap_idle  = (state == S_IDLE);
assign ap_done  = (state == S_DONE);
assign ap_ready = (state == S_DONE);

wire engines_idle = !busy_r && !busy_w && (fifo_count == 16'd0);

always @(posedge ap_clk) begin
  if (areset) begin
    state     <= S_IDLE;
    run_start <= 1'b0;
    en_r      <= 1'b0;
    en_w      <= 1'b0;
    cycles    <= 32'd0;
    flags     <= 32'd0;
  end else begin
    run_start <= 1'b0;
    case (state)
      S_IDLE: begin
        if (ap_start) begin
          flags  <= {27'b0, bad_mout, bad_mode, bad_beats, bad_blen, 1'b0};
          blen_r <= blen_eff;
          mout_r <= mout_eff;
          if (args_bad) begin
            state <= S_DONE;
          end else begin
            cycles    <= 32'd0;
            run_start <= 1'b1;
            en_r      <= mode_read || mode_copy;
            en_w      <= mode_write || mode_copy;
            state     <= S_RUN;
          end
        end
      end
      S_RUN: begin
        cycles <= cycles + 32'd1;
        if (fifo_overflow) flags[0] <= 1'b1;
        if (engines_idle && !run_start) state <= S_DONE;
      end
      S_DONE: begin
        state <= S_IDLE;
      end
      default: state <= S_IDLE;
    endcase
  end
end

//------------------------ read engine (m00_axi) -------------------------------
wire                                rd_out_valid;
wire [C_M00_AXI_DATA_WIDTH-1:0]     rd_out_data;
wire                                fifo_pop;

hbm_bw_read #(
  .ADDR_WIDTH ( C_M00_AXI_ADDR_WIDTH ),
  .DATA_WIDTH ( C_M00_AXI_DATA_WIDTH ),
  .MAX_BURST  ( MAX_BURST            ),
  .FIFO_DEPTH ( FIFO_DEPTH           )
) u_read (
  .clk          ( ap_clk           ),
  .areset       ( areset           ),
  .start        ( run_start        ),
  .enable       ( en_r             ),
  .base_addr    ( in_ptr[C_M00_AXI_ADDR_WIDTH-1:0] ),
  .num_beats    ( num_beats        ),
  .burst_len    ( blen_r           ),
  .max_out      ( mout_r           ),
  .use_credit   ( mode_copy        ),
  .pop          ( fifo_pop         ),
  .arvalid      ( m00_axi_arvalid  ),
  .arready      ( m00_axi_arready  ),
  .araddr       ( m00_axi_araddr   ),
  .arlen        ( m00_axi_arlen    ),
  .rvalid       ( m00_axi_rvalid   ),
  .rready       ( m00_axi_rready   ),
  .rdata        ( m00_axi_rdata    ),
  .rlast        ( m00_axi_rlast    ),
  .out_valid    ( rd_out_valid     ),
  .out_data     ( rd_out_data      ),
  .busy         ( busy_r           ),
  .ar_count     ( r_ar_count       ),
  .r_beats      ( r_beats          ),
  .r_stall      ( r_stall          ),
  .lat_first    ( r_lat_first      ),
  .checksum     ( r_checksum       ),
  .max_out_seen ( r_max_out_seen   )
);

// m00_axi is read-only
assign m00_axi_awvalid = 1'b0;
assign m00_axi_awaddr  = {C_M00_AXI_ADDR_WIDTH{1'b0}};
assign m00_axi_awlen   = 8'd0;
assign m00_axi_wvalid  = 1'b0;
assign m00_axi_wdata   = {C_M00_AXI_DATA_WIDTH{1'b0}};
assign m00_axi_wstrb   = {(C_M00_AXI_DATA_WIDTH/8){1'b0}};
assign m00_axi_wlast   = 1'b0;
assign m00_axi_bready  = 1'b1;

//------------------------ streaming FIFO (copy mode) --------------------------
wire                            fifo_rd_valid;
wire [C_M00_AXI_DATA_WIDTH-1:0] fifo_rd_data;

sync_fifo #(
  .DATA_WIDTH ( C_M00_AXI_DATA_WIDTH ),
  .DEPTH      ( FIFO_DEPTH           ),
  .ADDR_BITS  ( FIFO_ABITS           ),
  .CNT_BITS   ( 16                   )
) u_fifo (
  .clk      ( ap_clk                      ),
  .areset   ( areset                      ),
  .clear    ( run_start                   ),
  .wr_en    ( rd_out_valid && en_r && en_w ),
  .wr_data  ( rd_out_data                 ),
  .rd_en    ( fifo_pop                    ),
  .rd_data  ( fifo_rd_data                ),
  .rd_valid ( fifo_rd_valid               ),
  .count    ( fifo_count                  ),
  .overflow ( fifo_overflow               )
);

//------------------------ write engine (m01_axi) ------------------------------
hbm_bw_write #(
  .ADDR_WIDTH ( C_M01_AXI_ADDR_WIDTH ),
  .DATA_WIDTH ( C_M01_AXI_DATA_WIDTH ),
  .MAX_BURST  ( MAX_BURST            )
) u_write (
  .clk          ( ap_clk           ),
  .areset       ( areset           ),
  .start        ( run_start        ),
  .enable       ( en_w             ),
  .base_addr    ( out_ptr[C_M01_AXI_ADDR_WIDTH-1:0] ),
  .num_beats    ( num_beats        ),
  .burst_len    ( blen_r           ),
  .max_out      ( mout_r           ),
  .gen_mode     ( mode_write       ),
  .src_valid    ( fifo_rd_valid    ),
  .src_data     ( fifo_rd_data     ),
  .src_count    ( fifo_count       ),
  .src_pop      ( fifo_pop         ),
  .awvalid      ( m01_axi_awvalid  ),
  .awready      ( m01_axi_awready  ),
  .awaddr       ( m01_axi_awaddr   ),
  .awlen        ( m01_axi_awlen    ),
  .wvalid       ( m01_axi_wvalid   ),
  .wready       ( m01_axi_wready   ),
  .wdata        ( m01_axi_wdata    ),
  .wstrb        ( m01_axi_wstrb    ),
  .wlast        ( m01_axi_wlast    ),
  .bvalid       ( m01_axi_bvalid   ),
  .bready       ( m01_axi_bready   ),
  .busy         ( busy_w           ),
  .aw_count     ( w_aw_count       ),
  .w_beats      ( w_beats          ),
  .b_count      ( w_b_count        ),
  .w_stall      ( w_stall          ),
  .w_starve     ( w_starve         ),
  .max_out_seen ( w_max_out_seen   )
);

// m01_axi is write-only
assign m01_axi_arvalid = 1'b0;
assign m01_axi_araddr  = {C_M01_AXI_ADDR_WIDTH{1'b0}};
assign m01_axi_arlen   = 8'd0;
assign m01_axi_rready  = 1'b1;

endmodule
`default_nettype wire
