`default_nettype none
`timescale 1ns/1ps
//------------------------------------------------------------------------------
// hbm_scratch_core: read engine -> on-chip scratchpad -> write engine.
//
//   On ap_start, reads num_beats AXI beats from HBM at in_ptr into the
//   scratchpad, then writes them back to HBM at out_ptr in reversed uint32
//   order:  out[i] = in[W-1-i],  W = num_beats * DATA_WIDTH/32.
//   (beat order is reversed and the 32-bit lanes inside each beat are swapped)
//
//   - num_beats == 0 or > DEPTH: completes immediately without bus traffic.
//   - Bursts are <= MAX_BURST beats and start at multiples of MAX_BURST beats
//     from the page-aligned buffer base, so they never cross a 4 KB boundary
//     (requires MAX_BURST * DATA_WIDTH/8 to divide 4096).
//   - One outstanding transaction at a time: AR -> R, then AW -> W -> B.
//------------------------------------------------------------------------------
module hbm_scratch_core #(
  parameter integer ADDR_WIDTH = 64,
  parameter integer DATA_WIDTH = 512,   // multiple of 32
  parameter integer DEPTH      = 1024,  // scratchpad depth in beats (>= MAX_BURST)
  parameter integer MAX_BURST  = 64     // beats per burst
) (
  input  wire                    clk,
  input  wire                    areset,
  // ap_ctrl_hs handshake
  input  wire                    ap_start,
  output wire                    ap_idle,
  output wire                    ap_done,
  output wire                    ap_ready,
  // arguments, sampled when ap_start is seen in IDLE
  input  wire [31:0]             num_beats,
  input  wire [63:0]             in_ptr,
  input  wire [63:0]             out_ptr,
  // AXI4 master to HBM (minimal Vitis signal subset)
  output reg                     m_axi_arvalid,
  input  wire                    m_axi_arready,
  output reg  [ADDR_WIDTH-1:0]   m_axi_araddr,
  output reg  [7:0]              m_axi_arlen,
  input  wire                    m_axi_rvalid,
  output reg                     m_axi_rready,
  input  wire [DATA_WIDTH-1:0]   m_axi_rdata,
  input  wire                    m_axi_rlast,   // unused: burst end tracked by beat count
  output reg                     m_axi_awvalid,
  input  wire                    m_axi_awready,
  output reg  [ADDR_WIDTH-1:0]   m_axi_awaddr,
  output reg  [7:0]              m_axi_awlen,
  output reg                     m_axi_wvalid,
  input  wire                    m_axi_wready,
  output wire [DATA_WIDTH-1:0]   m_axi_wdata,
  output wire [DATA_WIDTH/8-1:0] m_axi_wstrb,
  output reg                     m_axi_wlast,
  input  wire                    m_axi_bvalid,
  output reg                     m_axi_bready
);

localparam integer LANES      = DATA_WIDTH / 32;
localparam integer BEAT_SHIFT = $clog2(DATA_WIDTH / 8);
localparam integer RAM_ABITS  = $clog2(DEPTH);
localparam integer IDX_BITS   = $clog2(DEPTH + 1);
localparam integer LEN_BITS   = $clog2(MAX_BURST + 1);
localparam [LEN_BITS-1:0] MAX_LEN = MAX_BURST;

localparam [3:0] S_IDLE    = 4'd0,
                 S_RD_PREP = 4'd1,
                 S_RD_ADDR = 4'd2,
                 S_RD_DATA = 4'd3,
                 S_WR_PREP = 4'd4,
                 S_WR_ADDR = 4'd5,
                 S_WR_DATA = 4'd6,
                 S_WR_RESP = 4'd7,
                 S_DONE    = 4'd8;

reg [3:0]            state;
reg [IDX_BITS-1:0]   n_beats;     // beats in this run
reg [ADDR_WIDTH-1:0] base_in;
reg [ADDR_WIDTH-1:0] base_out;
reg [IDX_BITS-1:0]   idx;         // beats covered by completed bursts
reg [LEN_BITS-1:0]   burst_len;   // beats in the current burst
reg [LEN_BITS-1:0]   beat_cnt;    // beats received (read) / fetched (write) in this burst
reg [RAM_ABITS-1:0]  scr_wptr;    // scratchpad write pointer, counts up (read phase)
reg [RAM_ABITS-1:0]  scr_rptr;    // scratchpad read pointer, counts down (write phase)

wire [IDX_BITS-1:0]   remaining = n_beats - idx;
wire [LEN_BITS-1:0]   next_len  = (remaining > MAX_BURST) ? MAX_LEN : remaining[LEN_BITS-1:0];
wire [ADDR_WIDTH-1:0] idx_ext   = idx;
wire [ADDR_WIDTH-1:0] offset    = idx_ext << BEAT_SHIFT;
wire [IDX_BITS-1:0]   idx_next  = idx + burst_len;
wire                  last_beat = (beat_cnt == burst_len - 1'b1);

assign ap_idle  = (state == S_IDLE);
assign ap_done  = (state == S_DONE);
assign ap_ready = (state == S_DONE);

//------------------------ Scratchpad -------------------------------------------
// Read engine: R beats are registered once before the scratchpad write.
reg                  pipe_we;
reg [RAM_ABITS-1:0]  pipe_addr;
reg [DATA_WIDTH-1:0] pipe_data;

always @(posedge clk) begin
  if (areset)
    pipe_we <= 1'b0;
  else
    pipe_we <= (state == S_RD_DATA) && m_axi_rvalid && m_axi_rready;
  pipe_addr <= scr_wptr;
  pipe_data <= m_axi_rdata;
end

// Write engine: the scratchpad output register holds the W beat. A new beat is
// fetched when the W register is empty or its beat is being accepted, so wdata
// stays stable under backpressure.
wire                  w_hs  = m_axi_wvalid & m_axi_wready;
wire                  fetch = (state == S_WR_DATA) && (beat_cnt != burst_len) &&
                              (!m_axi_wvalid || m_axi_wready);
wire [DATA_WIDTH-1:0] scr_rdata;

scratchpad_ram #(
  .DATA_WIDTH ( DATA_WIDTH ),
  .DEPTH      ( DEPTH      ),
  .ADDR_BITS  ( RAM_ABITS  )
) u_scratchpad (
  .clk   ( clk       ),
  .we    ( pipe_we   ),
  .waddr ( pipe_addr ),
  .wdata ( pipe_data ),
  .re    ( fetch     ),
  .raddr ( scr_rptr  ),
  .rdata ( scr_rdata )
);

genvar g;
generate
  for (g = 0; g < LANES; g = g + 1) begin : g_lane_reverse
    assign m_axi_wdata[g*32 +: 32] = scr_rdata[(LANES-1-g)*32 +: 32];
  end
endgenerate

assign m_axi_wstrb = {(DATA_WIDTH/8){1'b1}};

//------------------------ Control FSM ------------------------------------------
always @(posedge clk) begin
  if (areset) begin
    state         <= S_IDLE;
    m_axi_arvalid <= 1'b0;
    m_axi_rready  <= 1'b0;
    m_axi_awvalid <= 1'b0;
    m_axi_wvalid  <= 1'b0;
    m_axi_wlast   <= 1'b0;
    m_axi_bready  <= 1'b0;
  end else begin
    case (state)
      S_IDLE: begin
        if (ap_start) begin
          n_beats  <= num_beats[IDX_BITS-1:0];
          base_in  <= in_ptr[ADDR_WIDTH-1:0];
          base_out <= out_ptr[ADDR_WIDTH-1:0];
          idx      <= {IDX_BITS{1'b0}};
          scr_wptr <= {RAM_ABITS{1'b0}};
          state    <= (num_beats == 0 || num_beats > DEPTH) ? S_DONE : S_RD_PREP;
        end
      end

      // ---- read engine: HBM -> scratchpad ----
      S_RD_PREP: begin
        m_axi_araddr  <= base_in + offset;
        m_axi_arlen   <= next_len - 1'b1;
        m_axi_arvalid <= 1'b1;
        burst_len     <= next_len;
        beat_cnt      <= {LEN_BITS{1'b0}};
        state         <= S_RD_ADDR;
      end

      S_RD_ADDR: begin
        if (m_axi_arready) begin
          m_axi_arvalid <= 1'b0;
          m_axi_rready  <= 1'b1;
          state         <= S_RD_DATA;
        end
      end

      S_RD_DATA: begin
        if (m_axi_rvalid) begin
          scr_wptr <= scr_wptr + 1'b1;
          beat_cnt <= beat_cnt + 1'b1;
          if (last_beat) begin
            m_axi_rready <= 1'b0;
            if (idx_next == n_beats) begin
              idx      <= {IDX_BITS{1'b0}};
              scr_rptr <= n_beats - 1'b1;
              state    <= S_WR_PREP;
            end else begin
              idx      <= idx_next;
              state    <= S_RD_PREP;
            end
          end
        end
      end

      // ---- write engine: scratchpad (reversed) -> HBM ----
      S_WR_PREP: begin
        m_axi_awaddr  <= base_out + offset;
        m_axi_awlen   <= next_len - 1'b1;
        m_axi_awvalid <= 1'b1;
        burst_len     <= next_len;
        beat_cnt      <= {LEN_BITS{1'b0}};
        state         <= S_WR_ADDR;
      end

      S_WR_ADDR: begin
        if (m_axi_awready) begin
          m_axi_awvalid <= 1'b0;
          state         <= S_WR_DATA;
        end
      end

      S_WR_DATA: begin
        if (fetch) begin
          m_axi_wvalid <= 1'b1;
          m_axi_wlast  <= last_beat;
          beat_cnt     <= beat_cnt + 1'b1;
          scr_rptr     <= scr_rptr - 1'b1;
        end else if (w_hs) begin
          m_axi_wvalid <= 1'b0;
          m_axi_wlast  <= 1'b0;
        end
        if (w_hs && m_axi_wlast) begin
          m_axi_bready <= 1'b1;
          state        <= S_WR_RESP;
        end
      end

      S_WR_RESP: begin
        if (m_axi_bvalid) begin
          m_axi_bready <= 1'b0;
          idx          <= idx_next;
          state        <= (idx_next == n_beats) ? S_DONE : S_WR_PREP;
        end
      end

      S_DONE: begin
        state <= S_IDLE;
      end

      default: begin
        state <= S_IDLE;
      end
    endcase
  end
end

endmodule
`default_nettype wire
