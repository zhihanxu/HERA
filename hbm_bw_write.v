`default_nettype none
`timescale 1ns/1ps
//------------------------------------------------------------------------------
// hbm_bw_write: pipelined AXI write engine.
//
//   The AW generator runs ahead up to `max_out` outstanding bursts; W data
//   follows in AW order. W beats are registered, so a gap in the data source
//   never violates AXI (VALID is held until READY once asserted).
//
//   Two data sources:
//     gen_mode = 1  pattern generated on the fly; word j of the stream is j,
//                   so the host can verify without a reference buffer.
//     gen_mode = 0  beats from the FIFO (copy mode). A burst is addressed only
//                   once its data is buffered, so the W channel is never held
//                   open waiting for a producer.
//
//   w_credit tracks beats whose AW has handshaked but whose W data has not been
//   loaded yet, which keeps W strictly behind AW.
//------------------------------------------------------------------------------
module hbm_bw_write #(
  parameter integer ADDR_WIDTH = 64,
  parameter integer DATA_WIDTH = 512,
  parameter integer MAX_BURST  = 64
) (
  input  wire                     clk,
  input  wire                     areset,
  input  wire                     start,       // one-cycle pulse: clears counters
  input  wire                     enable,      // sampled at start; 0 = stay idle
  input  wire [ADDR_WIDTH-1:0]    base_addr,
  input  wire [31:0]              num_beats,
  input  wire [7:0]               burst_len,
  input  wire [7:0]               max_out,
  input  wire                     gen_mode,
  // data source (copy mode)
  input  wire                     src_valid,
  input  wire [DATA_WIDTH-1:0]    src_data,
  input  wire [15:0]              src_count,
  output wire                     src_pop,
  // AXI4 write address / data / response
  output reg                      awvalid,
  input  wire                     awready,
  output reg  [ADDR_WIDTH-1:0]    awaddr,
  output reg  [7:0]               awlen,
  output reg                      wvalid,
  input  wire                     wready,
  output reg  [DATA_WIDTH-1:0]    wdata,
  output wire [DATA_WIDTH/8-1:0]  wstrb,
  output reg                      wlast,
  input  wire                     bvalid,
  output wire                     bready,
  // status
  output wire                     busy,
  output reg  [31:0]              aw_count,
  output reg  [31:0]              w_beats,
  output reg  [31:0]              b_count,
  output reg  [31:0]              w_stall,
  output reg  [31:0]              w_starve,
  output reg  [7:0]               max_out_seen
);

localparam integer LANES      = DATA_WIDTH / 32;
localparam integer BEAT_SHIFT = $clog2(DATA_WIDTH / 8);

reg  [31:0]           aw_left;     // beats not yet addressed
reg  [31:0]           w_left;      // beats not yet loaded into the W register
reg  [ADDR_WIDTH-1:0] next_addr;
reg  [8:0]            out_cnt;     // AW accepted, B not yet returned (for reporting)
reg  [8:0]            aw_inflight; // AW loaded, B not yet returned (gates issue)
reg  [15:0]           w_credit;    // beats addressed but not yet loaded
reg  [7:0]            w_in_burst;  // beats still to load in the current burst
reg  [7:0]            aw_len_q;    // length of the next AW burst, precomputed
reg  [7:0]            w_len_q;     // length of the next W burst, precomputed
reg                   aw_left_nz;  // (aw_left != 0) and (w_left != 0), registered:
reg                   w_left_nz;   // keeps 32-bit reductions out of the gating paths
reg  [31:0]           beat_idx;    // stream position, for the generated pattern
reg                   running;

// Burst lengths are held in registers rather than recomputed from burst_len
// every cycle, keeping the 32-bit compare and mux off the data_ready / w_credit
// arithmetic (the 300 MHz critical path). Values are unchanged:
// aw_len_q == min(burst_len, aw_left), w_len_q == min(burst_len, w_left).
wire [7:0]  aw_this     = aw_len_q;
wire [7:0]  w_len_new   = w_len_q;
wire [7:0]  fired_len   = awlen + 8'd1;
wire [31:0] aw_left_nxt = aw_left - {24'b0, aw_len_q};
wire [7:0]  aw_len_nxt  = (aw_left_nxt > {24'b0, burst_len}) ? burst_len : aw_left_nxt[7:0];
wire [31:0] w_left_nxt  = w_left - 32'd1;
wire [7:0]  w_len_nxt   = (w_left_nxt > {24'b0, burst_len}) ? burst_len : w_left_nxt[7:0];
wire [7:0]  len_first   = (num_beats > {24'b0, burst_len}) ? burst_len : num_beats[7:0];

wire aw_fire      = awvalid && awready;
wire aw_slot_free = !awvalid || awready;
// in copy mode, require a full burst buffered beyond what earlier bursts owe
wire data_ready   = gen_mode || (src_count >= (w_credit + {8'b0, aw_this}));
// gate on `aw_inflight` so a request already in the AW output register counts
wire aw_load      = running && aw_left_nz && aw_slot_free &&
                    (aw_inflight < {1'b0, max_out}) && data_ready;

wire w_fire       = wvalid && wready;
wire w_slot_free  = !wvalid || wready;
wire src_ok       = gen_mode || src_valid;
wire w_mid_burst  = (w_in_burst != 8'd0);
wire w_can_start  = (w_in_burst == 8'd0) && (w_credit != 16'd0) && w_left_nz;
wire w_load       = running && w_slot_free && src_ok && (w_mid_burst || w_can_start);

assign src_pop = w_load && !gen_mode;
assign bready  = 1'b1;
assign wstrb   = {(DATA_WIDTH/8){1'b1}};
assign busy    = running;

// generated pattern: lane L of beat n holds (n*16 + L), i.e. the global word index
integer k;
reg [DATA_WIDTH-1:0] gen_data;
always @(*) begin
  for (k = 0; k < LANES; k = k + 1)
    gen_data[32*k +: 32] = {beat_idx[27:0], 4'b0} + k[31:0];
end

always @(posedge clk) begin
  if (areset) begin
    awvalid    <= 1'b0;
    wvalid     <= 1'b0;
    wlast      <= 1'b0;
    running    <= 1'b0;
    aw_left     <= 32'd0;
    w_left      <= 32'd0;
    out_cnt     <= 9'd0;
    aw_inflight <= 9'd0;
    w_credit    <= 16'd0;
    w_in_burst  <= 8'd0;
  end else if (start) begin
    // counters always clear on a run start, even when this engine is not used
    running      <= enable;
    aw_left      <= enable ? num_beats : 32'd0;
    w_left       <= enable ? num_beats : 32'd0;
    aw_left_nz   <= enable && (num_beats != 32'd0);
    w_left_nz    <= enable && (num_beats != 32'd0);
    aw_len_q     <= len_first;
    w_len_q      <= len_first;
    next_addr    <= base_addr;
    out_cnt      <= 9'd0;
    aw_inflight  <= 9'd0;
    w_credit     <= 16'd0;
    w_in_burst   <= 8'd0;
    beat_idx     <= 32'd0;
    awvalid      <= 1'b0;
    wvalid       <= 1'b0;
    wlast        <= 1'b0;
    aw_count     <= 32'd0;
    w_beats      <= 32'd0;
    b_count      <= 32'd0;
    w_stall      <= 32'd0;
    w_starve     <= 32'd0;
    max_out_seen <= 8'd0;
  end else begin
    // ---- AW channel ----
    if (aw_load) begin
      awvalid   <= 1'b1;
      awaddr    <= next_addr;
      awlen     <= aw_this - 8'd1;
      aw_left    <= aw_left_nxt;
      aw_left_nz <= (aw_left_nxt != 32'd0);
      aw_len_q   <= aw_len_nxt;
      next_addr <= next_addr + ({{(ADDR_WIDTH-8){1'b0}}, aw_this} << BEAT_SHIFT);
    end else if (aw_fire) begin
      awvalid <= 1'b0;
    end

    if (aw_fire) aw_count <= aw_count + 32'd1;
    out_cnt     <= out_cnt     + (aw_fire ? 9'd1 : 9'd0) - (bvalid ? 9'd1 : 9'd0);
    aw_inflight <= aw_inflight + (aw_load ? 9'd1 : 9'd0) - (bvalid ? 9'd1 : 9'd0);
    if (out_cnt > {1'b0, max_out_seen}) max_out_seen <= out_cnt[7:0];
    if (bvalid) b_count <= b_count + 32'd1;

    // W data may only follow an accepted AW
    w_credit <= w_credit + (aw_fire ? {8'b0, fired_len} : 16'd0) - (w_load ? 16'd1 : 16'd0);

    // ---- W channel ----
    if (w_load) begin
      wvalid <= 1'b1;
      wdata  <= gen_mode ? gen_data : src_data;
      if (w_in_burst == 8'd0) begin
        w_in_burst <= w_len_new - 8'd1;
        wlast      <= (w_len_new == 8'd1);
        if (w_len_new == 8'd1) w_len_q <= w_len_nxt;   // 1-beat burst ends here
      end else begin
        w_in_burst <= w_in_burst - 8'd1;
        wlast      <= (w_in_burst == 8'd1);
        if (w_in_burst == 8'd1)  w_len_q <= w_len_nxt; // last beat of this burst
      end
      w_left    <= w_left_nxt;
      w_left_nz <= (w_left_nxt != 32'd0);
      beat_idx <= beat_idx + 32'd1;
    end else if (w_fire) begin
      wvalid <= 1'b0;
      wlast  <= 1'b0;
    end

    if (w_fire)              w_beats  <= w_beats + 32'd1;
    if (wvalid && !wready)   w_stall  <= w_stall + 32'd1;
    if (running && !gen_mode && !src_valid && (w_mid_burst || w_can_start))
      w_starve <= w_starve + 32'd1;   // write side waiting on the producer

    if (running && !aw_left_nz && !w_left_nz && !wvalid &&
        !awvalid && (out_cnt == 9'd0))
      running <= 1'b0;
  end
end

endmodule
`default_nettype wire
