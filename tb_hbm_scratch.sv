`timescale 1ns/1ps
//------------------------------------------------------------------------------
// tb_hbm_scratch: self-checking testbench for the hbm_scratch RTL kernel.
//   The AXI4-Lite tasks drive the kernel the way XRT does for ap_ctrl_hs:
//   write arguments, set ap_start, then poll ap_done (or wait for the interrupt).
//   axi_mem_model stands in for HBM. Every test checks out[i] == in[W-1-i],
//   guard words around the output buffer, the untouched input buffer and the
//   AXI burst/beat counts.
//------------------------------------------------------------------------------
module tb_hbm_scratch;

  parameter int DATA_WIDTH = 512;

  localparam int ADDR_WIDTH   = 64;
  localparam int WPB          = DATA_WIDTH / 32;   // uint32 words per beat
  localparam int DEPTH        = 1024;              // must match hbm_scratch SCRATCH_DEPTH
  localparam int MAX_BURST    = 64;                // must match hbm_scratch MAX_BURST
  localparam int POLL_GAP     = 16;                // cycles between ap_done polls
  localparam int MAX_POLLS    = 20000;
  localparam logic [31:0] SENTINEL = 32'hDEADBEEF;

  localparam logic [11:0] REG_CTRL      = 12'h000;
  localparam logic [11:0] REG_GIE       = 12'h004;
  localparam logic [11:0] REG_IER       = 12'h008;
  localparam logic [11:0] REG_ISR       = 12'h00C;
  localparam logic [11:0] REG_NUM_BEATS = 12'h010;
  localparam logic [11:0] REG_IN_PTR_L  = 12'h018;
  localparam logic [11:0] REG_IN_PTR_H  = 12'h01C;
  localparam logic [11:0] REG_OUT_PTR_L = 12'h024;
  localparam logic [11:0] REG_OUT_PTR_H = 12'h028;

  //---------------------------- clock / reset ---------------------------------
  logic ap_clk   = 1'b0;
  logic ap_rst_n = 1'b0;
  always #1.667 ap_clk = ~ap_clk;   // ~300 MHz

  //---------------------------- AXI4-Lite control ------------------------------
  logic        s_awvalid = 1'b0;
  logic        s_awready;
  logic [11:0] s_awaddr  = '0;
  logic        s_wvalid  = 1'b0;
  logic        s_wready;
  logic [31:0] s_wdata   = '0;
  logic [3:0]  s_wstrb   = '0;
  logic        s_bvalid;
  logic        s_bready  = 1'b0;
  logic [1:0]  s_bresp;
  logic        s_arvalid = 1'b0;
  logic        s_arready;
  logic [11:0] s_araddr  = '0;
  logic        s_rvalid;
  logic        s_rready  = 1'b0;
  logic [31:0] s_rdata;
  logic [1:0]  s_rresp;
  logic        irq;

  //---------------------------- AXI4 master (HBM) -----------------------------
  logic                    m_awvalid, m_awready;
  logic [ADDR_WIDTH-1:0]   m_awaddr;
  logic [7:0]              m_awlen;
  logic                    m_wvalid, m_wready, m_wlast;
  logic [DATA_WIDTH-1:0]   m_wdata;
  logic [DATA_WIDTH/8-1:0] m_wstrb;
  logic                    m_bvalid, m_bready;
  logic                    m_arvalid, m_arready;
  logic [ADDR_WIDTH-1:0]   m_araddr;
  logic [7:0]              m_arlen;
  logic                    m_rvalid, m_rready, m_rlast;
  logic [DATA_WIDTH-1:0]   m_rdata;

  hbm_scratch #(
    .C_S_AXI_CONTROL_ADDR_WIDTH ( 12         ),
    .C_S_AXI_CONTROL_DATA_WIDTH ( 32         ),
    .C_M00_AXI_ADDR_WIDTH       ( ADDR_WIDTH ),
    .C_M00_AXI_DATA_WIDTH       ( DATA_WIDTH )
  ) dut (
    .ap_clk                ( ap_clk    ),
    .ap_rst_n              ( ap_rst_n  ),
    .m00_axi_awvalid       ( m_awvalid ),
    .m00_axi_awready       ( m_awready ),
    .m00_axi_awaddr        ( m_awaddr  ),
    .m00_axi_awlen         ( m_awlen   ),
    .m00_axi_wvalid        ( m_wvalid  ),
    .m00_axi_wready        ( m_wready  ),
    .m00_axi_wdata         ( m_wdata   ),
    .m00_axi_wstrb         ( m_wstrb   ),
    .m00_axi_wlast         ( m_wlast   ),
    .m00_axi_bvalid        ( m_bvalid  ),
    .m00_axi_bready        ( m_bready  ),
    .m00_axi_arvalid       ( m_arvalid ),
    .m00_axi_arready       ( m_arready ),
    .m00_axi_araddr        ( m_araddr  ),
    .m00_axi_arlen         ( m_arlen   ),
    .m00_axi_rvalid        ( m_rvalid  ),
    .m00_axi_rready        ( m_rready  ),
    .m00_axi_rdata         ( m_rdata   ),
    .m00_axi_rlast         ( m_rlast   ),
    .s_axi_control_awvalid ( s_awvalid ),
    .s_axi_control_awready ( s_awready ),
    .s_axi_control_awaddr  ( s_awaddr  ),
    .s_axi_control_wvalid  ( s_wvalid  ),
    .s_axi_control_wready  ( s_wready  ),
    .s_axi_control_wdata   ( s_wdata   ),
    .s_axi_control_wstrb   ( s_wstrb   ),
    .s_axi_control_arvalid ( s_arvalid ),
    .s_axi_control_arready ( s_arready ),
    .s_axi_control_araddr  ( s_araddr  ),
    .s_axi_control_rvalid  ( s_rvalid  ),
    .s_axi_control_rready  ( s_rready  ),
    .s_axi_control_rdata   ( s_rdata   ),
    .s_axi_control_rresp   ( s_rresp   ),
    .s_axi_control_bvalid  ( s_bvalid  ),
    .s_axi_control_bready  ( s_bready  ),
    .s_axi_control_bresp   ( s_bresp   ),
    .interrupt             ( irq       )
  );

  axi_mem_model #(
    .ADDR_WIDTH ( ADDR_WIDTH ),
    .DATA_WIDTH ( DATA_WIDTH ),
    .MAX_BURST  ( MAX_BURST  )
  ) hbm (
    .clk     ( ap_clk    ),
    .rst_n   ( ap_rst_n  ),
    .awvalid ( m_awvalid ),
    .awready ( m_awready ),
    .awaddr  ( m_awaddr  ),
    .awlen   ( m_awlen   ),
    .wvalid  ( m_wvalid  ),
    .wready  ( m_wready  ),
    .wdata   ( m_wdata   ),
    .wstrb   ( m_wstrb   ),
    .wlast   ( m_wlast   ),
    .bvalid  ( m_bvalid  ),
    .bready  ( m_bready  ),
    .arvalid ( m_arvalid ),
    .arready ( m_arready ),
    .araddr  ( m_araddr  ),
    .arlen   ( m_arlen   ),
    .rvalid  ( m_rvalid  ),
    .rready  ( m_rready  ),
    .rdata   ( m_rdata   ),
    .rlast   ( m_rlast   )
  );

  //---------------------------- bookkeeping -----------------------------------
  int          errors       = 0;
  int          tests_run    = 0;
  int          tests_failed = 0;
  logic [31:0] in_words[$];

  function automatic void fail(string msg);
    errors++;
    $display("ERROR: [%0t] %s", $time, msg);
  endfunction

  //---------------------------- AXI4-Lite master tasks ------------------------
  task automatic axil_write(input logic [11:0] addr, input logic [31:0] data);
    s_awaddr  <= addr;
    s_wdata   <= data;
    s_wstrb   <= 4'hF;
    fork
      begin
        s_awvalid <= 1'b1;
        do @(posedge ap_clk); while (!s_awready);
        s_awvalid <= 1'b0;
      end
      begin
        s_wvalid <= 1'b1;
        do @(posedge ap_clk); while (!s_wready);
        s_wvalid <= 1'b0;
      end
    join
    s_bready <= 1'b1;
    do @(posedge ap_clk); while (!s_bvalid);
    s_bready <= 1'b0;
  endtask

  task automatic axil_read(input logic [11:0] addr, output logic [31:0] data);
    s_araddr  <= addr;
    s_arvalid <= 1'b1;
    do @(posedge ap_clk); while (!s_arready);
    s_arvalid <= 1'b0;
    s_rready  <= 1'b1;
    do @(posedge ap_clk); while (!s_rvalid);
    data = s_rdata;
    s_rready <= 1'b0;
  endtask

  task automatic expect_reg(input logic [11:0] addr, input logic [31:0] exp, input string what);
    logic [31:0] v;
    axil_read(addr, v);
    if (v !== exp) fail($sformatf("%s readback 0x%08h, expected 0x%08h", what, v, exp));
  endtask

  //---------------------------- kernel invocation (XRT-like) ------------------
  task automatic run_kernel(input int unsigned n, input logic [63:0] ip, input logic [63:0] op,
                            input bit use_irq);
    logic [31:0] v;
    int          polls;
    bit          seen_busy;

    axil_write(REG_NUM_BEATS, n);
    axil_write(REG_IN_PTR_L,  ip[31:0]);
    axil_write(REG_IN_PTR_H,  ip[63:32]);
    axil_write(REG_OUT_PTR_L, op[31:0]);
    axil_write(REG_OUT_PTR_H, op[63:32]);
    expect_reg(REG_NUM_BEATS, n,         "num_beats");
    expect_reg(REG_IN_PTR_L,  ip[31:0],  "in_ptr[31:0]");
    expect_reg(REG_IN_PTR_H,  ip[63:32], "in_ptr[63:32]");
    expect_reg(REG_OUT_PTR_L, op[31:0],  "out_ptr[31:0]");
    expect_reg(REG_OUT_PTR_H, op[63:32], "out_ptr[63:32]");

    axil_read(REG_CTRL, v);
    if (v[0] !== 1'b0 || v[2] !== 1'b1)
      fail($sformatf("before start: CTRL=0x%08h, expected ap_idle=1 ap_start=0", v));

    axil_write(REG_CTRL, 32'h1);   // ap_start

    if (use_irq) begin
      fork
        begin
          fork
            wait (irq === 1'b1);
            begin
              repeat (MAX_POLLS * POLL_GAP) @(posedge ap_clk);
              fail("timeout waiting for interrupt");
            end
          join_any
          disable fork;
        end
      join
      axil_read(REG_ISR, v);
      if (v[0] !== 1'b1) fail($sformatf("ISR=0x%08h, expected ap_done status bit set", v));
      axil_write(REG_ISR, 32'h1);  // toggle-on-write clears the status
      repeat (3) @(posedge ap_clk);
      if (irq !== 1'b0) fail("interrupt still asserted after clearing ISR");
      axil_read(REG_CTRL, v);
      if (v[1] !== 1'b1) fail($sformatf("after interrupt: CTRL=0x%08h, expected ap_done=1", v));
    end else begin
      // ap_idle is registered in the control block (as in HLS), so the very first
      // poll may still read idle=1; a long run must look busy on some poll.
      polls     = 0;
      seen_busy = 1'b0;
      do begin
        axil_read(REG_CTRL, v);
        if (v[2] === 1'b0 && v[0] === 1'b1) seen_busy = 1'b1;
        polls++;
        if (polls > MAX_POLLS) begin
          fail("timeout polling ap_done");
          break;
        end
        if (!v[1]) repeat (POLL_GAP) @(posedge ap_clk);
      end while (!v[1]);
      if (n >= MAX_BURST && n <= DEPTH && !seen_busy)
        fail("never observed ap_start=1 ap_idle=0 while the kernel was running");
    end

    axil_read(REG_CTRL, v);
    if (v[1] !== 1'b0) fail($sformatf("after done: CTRL=0x%08h, ap_done not cleared on read", v));
    if (v[2] !== 1'b1) fail($sformatf("after done: CTRL=0x%08h, kernel not idle", v));
    if (v[0] !== 1'b0) fail($sformatf("after done: CTRL=0x%08h, ap_start not cleared", v));
  endtask

  //---------------------------- buffers and checking --------------------------
  task automatic prep_buffers(input int unsigned n, input logic [63:0] ip, input logic [63:0] op);
    int unsigned W = n * WPB;
    logic [31:0] v;
    hbm.clear_mem();
    hbm.clear_stats();
    in_words.delete();
    for (int i = 0; i < W; i++) begin
      v = $urandom;
      hbm.poke32(ip + 64'(4 * i), v);
      in_words.push_back(v);
    end
    for (int k = 1; k <= WPB; k++) begin
      hbm.poke32(op - 64'(4 * k), SENTINEL);             // guard before
      hbm.poke32(op + 64'(4 * (W + k - 1)), SENTINEL);   // guard after
    end
    for (int i = 0; i < W; i++) hbm.poke32(op + 64'(4 * i), SENTINEL);
  endtask

  task automatic check_output(input string name, input int unsigned n, input logic [63:0] ip,
                              input logic [63:0] op, input bit expect_run);
    int unsigned W      = n * WPB;
    int unsigned bursts = (n + MAX_BURST - 1) / MAX_BURST;
    int          bad    = 0;
    logic [31:0] got;
    logic [31:0] exp;

    for (int j = 0; j < W; j++) begin
      got = hbm.peek32(op + 64'(4 * j));
      exp = expect_run ? in_words[W - 1 - j] : SENTINEL;
      if (got !== exp) begin
        if (bad < 5) fail($sformatf("%s: out[%0d]=0x%08h, expected 0x%08h", name, j, got, exp));
        bad++;
      end
    end
    if (bad > 0) fail($sformatf("%s: %0d of %0d output words wrong", name, bad, W));

    for (int i = 0; i < W; i++) begin
      got = hbm.peek32(ip + 64'(4 * i));
      if (got !== in_words[i]) begin
        fail($sformatf("%s: input word %0d modified (0x%08h)", name, i, got));
        break;
      end
    end

    for (int k = 1; k <= WPB; k++) begin
      if (hbm.peek32(op - 64'(4 * k)) !== SENTINEL)
        fail($sformatf("%s: guard word before output buffer overwritten", name));
      if (hbm.peek32(op + 64'(4 * (W + k - 1))) !== SENTINEL)
        fail($sformatf("%s: guard word after output buffer overwritten", name));
    end

    if (expect_run) begin
      if (hbm.n_ar != bursts || hbm.n_aw != bursts || hbm.n_rbeats != n || hbm.n_wbeats != n)
        fail($sformatf("%s: AXI stats AR=%0d AW=%0d R=%0d W=%0d, expected %0d/%0d/%0d/%0d",
                       name, hbm.n_ar, hbm.n_aw, hbm.n_rbeats, hbm.n_wbeats, bursts, bursts, n, n));
    end else begin
      if (hbm.n_ar != 0 || hbm.n_aw != 0)
        fail($sformatf("%s: expected no AXI traffic, saw AR=%0d AW=%0d", name, hbm.n_ar, hbm.n_aw));
    end
  endtask

  task automatic do_test(input string name, input int unsigned n, input logic [63:0] ip,
                         input logic [63:0] op, input int stall, input bit use_irq = 1'b0);
    int e0 = errors;
    int m0 = hbm.errors;
    bit valid = (n != 0) && (n <= DEPTH);
    time t0;

    hbm.stall_pct = stall;
    prep_buffers(n, ip, op);
    t0 = $time;
    run_kernel(n, ip, op, use_irq);
    check_output(name, n, ip, op, valid);
    tests_run++;
    if (errors == e0 && hbm.errors == m0)
      $display("[TEST] %-26s N=%-5d stall=%2d%% in=0x%016h out=0x%016h  PASS (%0t)",
               name, n, stall, ip, op, $time - t0);
    else begin
      tests_failed++;
      $display("[TEST] %-26s N=%-5d stall=%2d%% in=0x%016h out=0x%016h  FAIL",
               name, n, stall, ip, op);
    end
  endtask

  //---------------------------- test sequence ---------------------------------
  initial begin
    int unsigned n;
    logic [63:0] ip;
    logic [63:0] op;

    $display("tb_hbm_scratch: DATA_WIDTH=%0d words/beat=%0d DEPTH=%0d MAX_BURST=%0d",
             DATA_WIDTH, WPB, DEPTH, MAX_BURST);
    ap_rst_n = 1'b0;
    repeat (10) @(posedge ap_clk);
    ap_rst_n <= 1'b1;
    repeat (5) @(posedge ap_clk);

    // All tests run back to back without a reset in between.
    do_test("single beat",          1,       64'h0000_0001_0000_0000, 64'h0000_0002_0000_0000, 0);
    do_test("one full burst",       64,      64'h0000_0001_0004_0000, 64'h0000_0002_0010_0000, 30);
    do_test("burst + 1",            65,      64'h0000_0001_0008_0000, 64'h0000_0002_0020_0000, 30);
    do_test("partial last burst",   1000,    64'h8000_0012_3456_7000, 64'h0000_00AB_CDEF_0000, 30);
    do_test("full scratchpad",      DEPTH,   64'h0000_0004_0000_0000, 64'h0000_0005_0000_0000, 50);
    do_test("full, no backpressure",DEPTH,   64'h0000_0006_0000_0000, 64'h0000_0007_0000_0000, 0);
    do_test("N=0 no-op",            0,       64'h0000_0001_0000_0000, 64'h0000_0002_0000_0000, 0);
    do_test("N=DEPTH+1 rejected",   DEPTH+1, 64'h0000_0001_0000_0000, 64'h0000_0002_0000_0000, 0);
    do_test("back-to-back #1",      129,     64'h0000_0003_0000_1000, 64'h0000_0003_1000_0000, 20);
    do_test("back-to-back #2",      129,     64'h0000_0003_1000_0000, 64'h0000_0003_0000_1000, 20);

    axil_write(REG_GIE, 32'h1);
    axil_write(REG_IER, 32'h1);
    do_test("interrupt completion", 200,     64'h0000_0008_0000_0000, 64'h0000_0009_0000_0000, 10, 1'b1);
    axil_write(REG_IER, 32'h0);
    axil_write(REG_GIE, 32'h0);

    for (int r = 0; r < 6; r++) begin
      n  = $urandom_range(DEPTH, 1);
      ip = 64'h0000_0010_0000_0000 + (64'($urandom_range(32'hFFFFF, 0)) << 12);
      op = 64'h0000_0020_0000_0000 + (64'($urandom_range(32'hFFFFF, 0)) << 12);
      do_test($sformatf("random #%0d", r), n, ip, op, $urandom_range(70, 0));
    end

    $display("------------------------------------------------------------");
    $display("tests run: %0d  failed: %0d  tb errors: %0d  model errors: %0d",
             tests_run, tests_failed, errors, hbm.errors);
    if (tests_failed == 0 && errors == 0 && hbm.errors == 0)
      $display("ALL TESTS PASSED (DATA_WIDTH=%0d)", DATA_WIDTH);
    else
      $display("TEST FAILED (DATA_WIDTH=%0d)", DATA_WIDTH);
    $finish;
  end

  initial begin
    #100_000_000;   // 100 ms of simulated time
    $display("ERROR: global watchdog timeout");
    $display("TEST FAILED (DATA_WIDTH=%0d)", DATA_WIDTH);
    $finish;
  end

endmodule
