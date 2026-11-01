`default_nettype none
`timescale 1ns/1ps
//------------------------------------------------------------------------------
// hbm_bw_read: pipelined AXI read engine.
//
//   Address issue is decoupled from data consumption: the AR generator runs
//   ahead up to `max_out` outstanding bursts while the R channel drains
//   independently. This is the difference from hbm_scratch, which waited for
//   rlast before issuing the next AR and so could never hide memory latency.
//
//   `rready` is tied high so the memory is never backpressured. In copy mode a
//   credit counter reserves FIFO space before each AR is issued, which makes
//   overflow impossible without ever stalling R.
//
//   Bursts are `burst_len` beats (a power of two, <= MAX_BURST), so with a
//   page-aligned base address no burst crosses a 4 KB boundary.
//------------------------------------------------------------------------------
module hbm_bw_read #(
  parameter integer ADDR_WIDTH = 64,
  parameter integer DATA_WIDTH = 512,
  parameter integer MAX_BURST  = 64,
  parameter integer FIFO_DEPTH = 1024
) (
  input  wire                   clk,
  input  wire                   areset,
  input  wire                   start,        // one-cycle pulse: clears counters
  input  wire                   enable,       // sampled at start; 0 = stay idle
  input  wire [ADDR_WIDTH-1:0]  base_addr,
  input  wire [31:0]            num_beats,
  input  wire [7:0]             burst_len,
  input  wire [7:0]             max_out,
  input  wire                   use_credit,   // 1 in copy mode (FIFO downstream)
  input  wire                   pop,          // a beat left the FIFO this cycle
  // AXI4 read address / data
  output reg                    arvalid,
  input  wire                   arready,
  output reg  [ADDR_WIDTH-1:0]  araddr,
  output reg  [7:0]             arlen,
  input  wire                   rvalid,
  output wire                   rready,
  input  wire [DATA_WIDTH-1:0]  rdata,
  input  wire                   rlast,
  // data to the FIFO
  output wire                   out_valid,
  output wire [DATA_WIDTH-1:0]  out_data,
  // status
  output wire                   busy,
  output reg  [31:0]            ar_count,
  output reg  [31:0]            r_beats,
  output reg  [31:0]            r_stall,
  output reg  [31:0]            lat_first,
  output reg  [31:0]            checksum,
  output reg  [7:0]             max_out_seen
);

localparam integer LANES      = DATA_WIDTH / 32;
localparam integer BEAT_SHIFT = $clog2(DATA_WIDTH / 8);

reg  [31:0]           ar_left;
reg  [ADDR_WIDTH-1:0] next_addr;
reg  [8:0]            out_cnt;    // AR accepted, burst not finished (for reporting)
reg  [8:0]            inflight;   // AR loaded, burst not finished (gates issue)
reg  [15:0]           credit;
reg  [7:0]            len_q;      // length of the next burst, precomputed
reg                   ar_left_nz; // (ar_left != 0), registered: keeps a 32-bit
                                  // reduction out of the AR gating path
reg                   running;
reg                   lat_timing;
reg  [31:0]           lat_cnt;

// The next burst length is held in a register rather than recomputed from
// burst_len every cycle. That keeps the 32-bit compare and mux off the credit
// arithmetic path, which was 11 logic levels deep and the critical path at
// 300 MHz. Behaviour is unchanged: len_q always equals min(burst_len, ar_left).
wire [7:0]  this_len   = len_q;
wire [31:0] left_next  = ar_left - {24'b0, len_q};
wire [7:0]  len_next   = (left_next > {24'b0, burst_len}) ? burst_len : left_next[7:0];
wire [7:0]  len_first  = (num_beats > {24'b0, burst_len}) ? burst_len : num_beats[7:0];
wire       ar_fire     = arvalid && arready;
wire       burst_end   = rvalid && rready && rlast;
wire       slot_free   = !arvalid || arready;
wire       credit_ok   = !use_credit || (credit >= {8'b0, this_len});
// gate on `inflight`, not `out_cnt`: a request already sitting in the AR output
// register is committed, so counting only accepted ones overshoots max_out by 1
wire       ar_load     = running && ar_left_nz && slot_free &&
                         (inflight < {1'b0, max_out}) && credit_ok;

assign rready    = 1'b1;
assign out_valid = rvalid;
assign out_data  = rdata;
assign busy      = running;

// XOR-fold each beat to 32 bits, then accumulate. Order independent, so it is
// insensitive to burst scheduling, and cheap compared with summing every lane.
integer k;
reg [31:0] fold;
always @(*) begin
  fold = 32'd0;
  for (k = 0; k < LANES; k = k + 1)
    fold = fold ^ rdata[32*k +: 32];
end

always @(posedge clk) begin
  if (areset) begin
    arvalid    <= 1'b0;
    running    <= 1'b0;
    ar_left    <= 32'd0;
    out_cnt    <= 9'd0;
    inflight   <= 9'd0;
    credit     <= FIFO_DEPTH[15:0];
    lat_timing <= 1'b0;
  end else if (start) begin
    // counters always clear on a run start, even when this engine is not used,
    // so the host never reads a stale value from the previous run
    running      <= enable;
    ar_left      <= enable ? num_beats : 32'd0;
    ar_left_nz   <= enable && (num_beats != 32'd0);
    len_q        <= len_first;
    next_addr    <= base_addr;
    out_cnt      <= 9'd0;
    inflight     <= 9'd0;
    credit       <= FIFO_DEPTH[15:0];
    arvalid      <= 1'b0;
    ar_count     <= 32'd0;
    r_beats      <= 32'd0;
    r_stall      <= 32'd0;
    lat_first    <= 32'd0;
    lat_cnt      <= 32'd0;
    lat_timing   <= 1'b0;
    checksum     <= 32'd0;
    max_out_seen <= 8'd0;
  end else begin
    // ---- AR channel ----
    if (ar_load) begin
      arvalid   <= 1'b1;
      araddr    <= next_addr;
      arlen     <= this_len - 8'd1;
      ar_left    <= left_next;
      ar_left_nz <= (left_next != 32'd0);
      len_q      <= len_next;
      next_addr <= next_addr + ({{(ADDR_WIDTH-8){1'b0}}, this_len} << BEAT_SHIFT);
    end else if (ar_fire) begin
      arvalid <= 1'b0;
    end

    if (use_credit)
      credit <= credit - (ar_load ? {8'b0, this_len} : 16'd0) + (pop ? 16'd1 : 16'd0);

    if (ar_fire) ar_count <= ar_count + 32'd1;
    out_cnt  <= out_cnt  + (ar_fire ? 9'd1 : 9'd0) - (burst_end ? 9'd1 : 9'd0);
    inflight <= inflight + (ar_load ? 9'd1 : 9'd0) - (burst_end ? 9'd1 : 9'd0);
    if (out_cnt > {1'b0, max_out_seen}) max_out_seen <= out_cnt[7:0];

    // ---- first-read latency: AR handshake -> first R beat ----
    if (ar_fire && !lat_timing && (r_beats == 32'd0) && (lat_first == 32'd0))
      lat_timing <= 1'b1;
    if (lat_timing) begin
      if (rvalid) begin
        lat_timing <= 1'b0;
        lat_first  <= lat_cnt;
      end else begin
        lat_cnt <= lat_cnt + 32'd1;
      end
    end

    // ---- R channel ----
    if (rvalid) begin
      r_beats  <= r_beats + 32'd1;
      checksum <= checksum + fold;
    end else if (running && (out_cnt != 9'd0)) begin
      r_stall <= r_stall + 32'd1;   // requests in flight but no data: memory-bound
    end

    if (running && !ar_left_nz && !arvalid && (out_cnt == 9'd0))
      running <= 1'b0;
  end
end

endmodule
`default_nettype wire
