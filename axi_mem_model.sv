`timescale 1ns/1ps
//------------------------------------------------------------------------------
// axi_mem_model: AXI4 slave memory standing in for HBM in simulation.
//   - Sparse byte-addressed 64-bit memory (little-endian beats).
//   - Random ready/valid stalls, controlled by stall_pct (0..100).
//   - Protocol checks: burst length <= MAX_BURST, no 4 KB crossing, WLAST
//     position, WSTRB, one outstanding transaction per direction, and
//     VALID/payload stability until handshake.
//   - Statistics (n_ar, n_aw, n_rbeats, n_wbeats) and error count for the TB.
//------------------------------------------------------------------------------
module axi_mem_model #(
  parameter int ADDR_WIDTH = 64,
  parameter int DATA_WIDTH = 512,
  parameter int MAX_BURST  = 64
) (
  input  logic                    clk,
  input  logic                    rst_n,
  // write address / data / response
  input  logic                    awvalid,
  output logic                    awready,
  input  logic [ADDR_WIDTH-1:0]   awaddr,
  input  logic [7:0]              awlen,
  input  logic                    wvalid,
  output logic                    wready,
  input  logic [DATA_WIDTH-1:0]   wdata,
  input  logic [DATA_WIDTH/8-1:0] wstrb,
  input  logic                    wlast,
  output logic                    bvalid,
  input  logic                    bready,
  // read address / data
  input  logic                    arvalid,
  output logic                    arready,
  input  logic [ADDR_WIDTH-1:0]   araddr,
  input  logic [7:0]              arlen,
  output logic                    rvalid,
  input  logic                    rready,
  output logic [DATA_WIDTH-1:0]   rdata,
  output logic                    rlast
);

  localparam int NB   = DATA_WIDTH / 8;
  localparam int PAGE = 4096;

  bit [7:0] mem [bit [63:0]];

  int stall_pct = 0;
  int errors    = 0;
  int n_ar      = 0;
  int n_aw      = 0;
  int n_rbeats  = 0;
  int n_wbeats  = 0;

  //---------------------------- helpers ---------------------------------------
  function automatic void model_error(string msg);
    errors++;
    $display("ERROR: [%0t] [HBM model] %s", $time, msg);
  endfunction

  function automatic void poke32(bit [63:0] addr, logic [31:0] v);
    for (int b = 0; b < 4; b++) mem[addr + b] = v[8*b +: 8];
  endfunction

  function automatic logic [31:0] peek32(bit [63:0] addr);
    logic [31:0] v;
    for (int b = 0; b < 4; b++) v[8*b +: 8] = mem.exists(addr + b) ? mem[addr + b] : 8'hxx;
    return v;
  endfunction

  function automatic logic [DATA_WIDTH-1:0] get_beat(bit [63:0] addr);
    logic [DATA_WIDTH-1:0] d;
    for (int b = 0; b < NB; b++) d[8*b +: 8] = mem.exists(addr + b) ? mem[addr + b] : 8'hxx;
    return d;
  endfunction

  function automatic void put_beat(bit [63:0] addr, logic [DATA_WIDTH-1:0] d, logic [NB-1:0] strb);
    for (int b = 0; b < NB; b++) if (strb[b]) mem[addr + b] = d[8*b +: 8];
  endfunction

  function automatic void clear_mem();
    mem.delete();
  endfunction

  function automatic void clear_stats();
    n_ar = 0; n_aw = 0; n_rbeats = 0; n_wbeats = 0;
  endfunction

  function automatic bit go();
    return ($urandom_range(99, 0) >= stall_pct);
  endfunction

  function automatic void check_burst(string ch, bit [63:0] addr, logic [7:0] len);
    int beats = int'(len) + 1;
    if (beats > MAX_BURST)
      model_error($sformatf("%s burst of %0d beats exceeds MAX_BURST=%0d", ch, beats, MAX_BURST));
    if ((addr % NB) != 0)
      model_error($sformatf("%s addr 0x%016h is not beat aligned", ch, addr));
    if ((addr % PAGE) + beats * NB > PAGE)
      model_error($sformatf("%s burst addr=0x%016h beats=%0d crosses a 4 KB boundary", ch, addr, beats));
  endfunction

  //---------------------------- write channel ---------------------------------
  typedef enum logic [1:0] {W_IDLE, W_DATA, W_RESP} w_state_t;
  w_state_t  w_st;
  bit [63:0] w_addr;
  int        w_len;
  int        w_cnt;

  always @(posedge clk) begin
    if (!rst_n) begin
      w_st    <= W_IDLE;
      awready <= 1'b0;
      wready  <= 1'b0;
      bvalid  <= 1'b0;
    end else begin
      case (w_st)
        W_IDLE: begin
          if (wvalid) model_error("WVALID before AW handshake");
          awready <= go();
          if (awvalid && awready) begin
            n_aw++;
            check_burst("AW", awaddr, awlen);
            w_addr  <= awaddr;
            w_len   <= int'(awlen) + 1;
            w_cnt   <= 0;
            awready <= 1'b0;
            wready  <= go();
            w_st    <= W_DATA;
          end
        end
        W_DATA: begin
          if (awvalid) model_error("AWVALID while a write burst is outstanding");
          wready <= go();
          if (wvalid && wready) begin
            n_wbeats++;
            if (wstrb !== '1) model_error($sformatf("WSTRB=0x%h is not all ones", wstrb));
            if (wlast !== (w_cnt == w_len - 1))
              model_error($sformatf("WLAST=%0b on beat %0d of a %0d-beat burst", wlast, w_cnt, w_len));
            put_beat(w_addr + w_cnt * NB, wdata, wstrb);
            if (w_cnt == w_len - 1) begin
              wready <= 1'b0;
              w_st   <= W_RESP;
            end
            w_cnt <= w_cnt + 1;
          end
        end
        W_RESP: begin
          if (awvalid) model_error("AWVALID before B handshake");
          if (wvalid)  model_error("WVALID after the last beat of a burst");
          if (bvalid && bready) begin
            bvalid <= 1'b0;
            w_st   <= W_IDLE;
          end else if (!bvalid) begin
            bvalid <= go();
          end
        end
        default: w_st <= W_IDLE;
      endcase
    end
  end

  //---------------------------- read channel ----------------------------------
  typedef enum logic {R_IDLE, R_DATA} r_state_t;
  r_state_t  r_st;
  bit [63:0] r_addr;
  int        r_len;
  int        r_cnt;
  int        r_nxt;

  always @(posedge clk) begin
    if (!rst_n) begin
      r_st    <= R_IDLE;
      arready <= 1'b0;
      rvalid  <= 1'b0;
      rlast   <= 1'b0;
      rdata   <= '0;
    end else begin
      case (r_st)
        R_IDLE: begin
          arready <= go();
          if (arvalid && arready) begin
            n_ar++;
            check_burst("AR", araddr, arlen);
            r_addr  <= araddr;
            r_len   <= int'(arlen) + 1;
            r_cnt   <= 0;
            arready <= 1'b0;
            r_st    <= R_DATA;
          end
        end
        R_DATA: begin
          if (arvalid) model_error("ARVALID while a read burst is outstanding");
          r_nxt = r_cnt;
          if (rvalid && rready) begin
            n_rbeats++;
            r_nxt = r_cnt + 1;
          end
          if (r_nxt == r_len) begin
            rvalid <= 1'b0;
            rlast  <= 1'b0;
            r_st   <= R_IDLE;
          end else if (!rvalid || rready) begin
            if (go()) begin
              rvalid <= 1'b1;
              rdata  <= get_beat(r_addr + r_nxt * NB);
              rlast  <= (r_nxt == r_len - 1);
            end else begin
              rvalid <= 1'b0;
              rlast  <= 1'b0;
            end
          end
          r_cnt <= r_nxt;
        end
        default: r_st <= R_IDLE;
      endcase
    end
  end

  //---------------------------- master-side stability checks -----------------
  logic                  awvalid_q, awready_q, arvalid_q, arready_q, wvalid_q, wready_q, wlast_q;
  logic [ADDR_WIDTH-1:0] awaddr_q, araddr_q;
  logic [7:0]            awlen_q, arlen_q;
  logic [DATA_WIDTH-1:0] wdata_q;

  always @(posedge clk) begin
    if (!rst_n) begin
      awvalid_q <= 1'b0;
      arvalid_q <= 1'b0;
      wvalid_q  <= 1'b0;
    end else begin
      if ($isunknown({awvalid, wvalid, bready, arvalid, rready}))
        model_error("X/Z on an AXI master control signal");
      if (awvalid_q && !awready_q && (!awvalid || awaddr !== awaddr_q || awlen !== awlen_q))
        model_error("AW channel changed before handshake");
      if (arvalid_q && !arready_q && (!arvalid || araddr !== araddr_q || arlen !== arlen_q))
        model_error("AR channel changed before handshake");
      if (wvalid_q && !wready_q && (!wvalid || wdata !== wdata_q || wlast !== wlast_q))
        model_error("W channel changed before handshake");
      awvalid_q <= awvalid; awready_q <= awready; awaddr_q <= awaddr; awlen_q <= awlen;
      arvalid_q <= arvalid; arready_q <= arready; araddr_q <= araddr; arlen_q <= arlen;
      wvalid_q  <= wvalid;  wready_q  <= wready;  wdata_q  <= wdata;  wlast_q <= wlast;
    end
  end

endmodule
