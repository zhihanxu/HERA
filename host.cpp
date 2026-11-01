// hbm_host: XRT host application for the hbm_scratch RTL kernel on Alveo U280.
//
// Flow: host memory -> HBM (input buffer) -> kernel reads it into the on-chip
// scratchpad and writes the reversed uint32 stream back to HBM (output buffer)
// -> host memory. The host then checks out[i] == in[W-1-i] for all W words and
// that the input buffer read back from HBM matches what was sent.
//
// Usage: hbm_host -x <xclbin> [-d <bdf>] [-n <num_beats>] [-s <seed>] [-t <timeout_s>]
//   With XCL_EMULATION_MODE set (hw_emu), the emulated device index 0 is used.

#include <xrt/xrt_bo.h>
#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>

#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr const char* kKernelName   = "hbm_scratch";
constexpr uint32_t    kWordsPerBeat = 16;    // 512-bit AXI beat = 16 uint32 words
constexpr uint32_t    kBytesPerBeat = 64;
constexpr uint32_t    kMaxBeats     = 1024;  // on-chip scratchpad depth
constexpr uint32_t    kSentinel     = 0xDEADBEEF;

using Clock = std::chrono::steady_clock;

double ms_since(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

std::string hex(uint64_t v, int width) {
  std::ostringstream os;
  os << "0x" << std::hex << std::setw(width) << std::setfill('0') << v;
  return os.str();
}

void usage(const char* prog) {
  std::cerr << "Usage: " << prog << " -x <xclbin> [-d <bdf>] [-n <num_beats>] [-s <seed>] [-t <timeout_s>]\n"
            << "  -x  xclbin file (required)\n"
            << "  -d  device BDF (default 0000:3b:00.1; ignored under XCL_EMULATION_MODE)\n"
            << "  -n  number of 512-bit beats, 1.." << kMaxBeats << " (default 1000)\n"
            << "  -s  random seed for the input pattern (default 1)\n"
            << "  -t  kernel timeout in seconds (default 30, or 3600 in emulation)\n";
}

}  // namespace

int main(int argc, char** argv) {
  const char* emu_mode = std::getenv("XCL_EMULATION_MODE");
  std::string xclbin;
  std::string bdf       = "0000:3b:00.1";
  unsigned long beats   = 1000;
  unsigned long seed    = 1;
  unsigned long timeout = emu_mode ? 3600 : 30;

  int opt;
  while ((opt = getopt(argc, argv, "x:d:n:s:t:h")) != -1) {
    switch (opt) {
      case 'x': xclbin  = optarg; break;
      case 'd': bdf     = optarg; break;
      case 'n': beats   = std::strtoul(optarg, nullptr, 0); break;
      case 's': seed    = std::strtoul(optarg, nullptr, 0); break;
      case 't': timeout = std::strtoul(optarg, nullptr, 0); break;
      default:  usage(argv[0]); return 1;
    }
  }
  if (xclbin.empty() || beats == 0 || beats > kMaxBeats) {
    usage(argv[0]);
    return 1;
  }

  const uint32_t num_beats = static_cast<uint32_t>(beats);
  const size_t   n_words   = size_t{num_beats} * kWordsPerBeat;
  const size_t   n_bytes   = size_t{num_beats} * kBytesPerBeat;

  try {
    // ---- device, xclbin, kernel ----
    auto t0 = Clock::now();
    xrt::device device = emu_mode ? xrt::device(0) : xrt::device(bdf);
    std::cout << "Device        : " << (emu_mode ? std::string("index 0 (") + emu_mode + ")" : bdf) << "\n";
    auto uuid   = device.load_xclbin(xclbin);
    auto kernel = xrt::kernel(device, uuid, kKernelName);
    std::cout << "xclbin        : " << xclbin << " (loaded in " << std::fixed << std::setprecision(1)
              << ms_since(t0) << " ms)\n";
    std::cout << "Kernel        : " << kKernelName << ", mem groups in_ptr=" << kernel.group_id(1)
              << " out_ptr=" << kernel.group_id(2) << "\n";
    std::cout << "Transfer      : " << num_beats << " beats = " << n_words << " uint32 words = "
              << n_bytes << " bytes per buffer\n";

    // ---- buffers in HBM ----
    auto bo_in  = xrt::bo(device, n_bytes, xrt::bo::flags::normal, kernel.group_id(1));
    auto bo_out = xrt::bo(device, n_bytes, xrt::bo::flags::normal, kernel.group_id(2));
    auto in     = bo_in.map<uint32_t*>();
    auto out    = bo_out.map<uint32_t*>();
    std::cout << "Buffers       : in @ " << hex(bo_in.address(), 16) << ", out @ " << hex(bo_out.address(), 16)
              << " (device addresses)\n";
    if ((bo_in.address() & 0xFFF) || (bo_out.address() & 0xFFF))
      std::cout << "WARNING       : buffer not page aligned; kernel bursts may cross 4 KB boundaries\n";

    std::mt19937 gen(static_cast<uint32_t>(seed));
    std::vector<uint32_t> golden(n_words);
    for (size_t i = 0; i < n_words; ++i) {
      golden[i] = gen();
      in[i]     = golden[i];
      out[i]    = kSentinel;
    }

    // ---- host -> HBM ----
    auto t_h2d = Clock::now();
    bo_in.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    bo_out.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    const double ms_h2d = ms_since(t_h2d);

    // ---- run kernel: HBM -> scratchpad -> reversed -> HBM ----
    // Wait in 1 s slices: the hw_emu shim sleeps for the whole timeout given to
    // wait() before it sees completion, while hardware returns as soon as done.
    auto t_run    = Clock::now();
    auto run      = kernel(num_beats, bo_in, bo_out);
    auto deadline = t_run + std::chrono::seconds(timeout);
    auto state    = run.wait(std::chrono::milliseconds(1000));
    while (state == ERT_CMD_STATE_TIMEOUT && Clock::now() < deadline)
      state = run.wait(std::chrono::milliseconds(1000));
    const double ms_run = ms_since(t_run);
    if (state == ERT_CMD_STATE_TIMEOUT) {
      std::cerr << "ERROR         : kernel did not complete within " << timeout << " s\n";
      return 2;
    }
    if (state != ERT_CMD_STATE_COMPLETED) {
      std::cerr << "ERROR         : kernel run ended in state " << static_cast<int>(state) << "\n";
      return 2;
    }

    // ---- HBM -> host (output, and input read back to check the host->HBM path) ----
    for (size_t i = 0; i < n_words; ++i) in[i] = 0;
    auto t_d2h = Clock::now();
    bo_out.sync(XCL_BO_SYNC_BO_FROM_DEVICE);
    bo_in.sync(XCL_BO_SYNC_BO_FROM_DEVICE);
    const double ms_d2h = ms_since(t_d2h);

    // ---- verify ----
    size_t in_bad = 0, out_bad = 0, sentinel_left = 0;
    for (size_t i = 0; i < n_words; ++i) {
      if (in[i] != golden[i]) ++in_bad;
      const uint32_t expect = golden[n_words - 1 - i];
      if (out[i] == kSentinel && expect != kSentinel) ++sentinel_left;
      if (out[i] != expect) {
        if (out_bad < 8)
          std::cout << "MISMATCH      : out[" << i << "]=" << hex(out[i], 8) << " expected in["
                    << (n_words - 1 - i) << "]=" << hex(expect, 8) << "\n";
        ++out_bad;
      }
    }

    std::cout << "Sample        : in[0..3]  =";
    for (size_t i = 0; i < 4 && i < n_words; ++i) std::cout << " " << hex(golden[i], 8);
    std::cout << "\n                in[W-4..W-1]=";
    for (size_t i = n_words >= 4 ? n_words - 4 : 0; i < n_words; ++i) std::cout << " " << hex(golden[i], 8);
    std::cout << "\n                out[0..3] =";
    for (size_t i = 0; i < 4 && i < n_words; ++i) std::cout << " " << hex(out[i], 8);
    std::cout << "\n";

    std::cout << std::fixed << std::setprecision(1)
              << "Timing        : host->HBM " << ms_h2d << " ms, kernel " << ms_run << " ms, HBM->host "
              << ms_d2h << " ms\n"
              << "Check         : input readback mismatches " << in_bad << "/" << n_words
              << ", output mismatches " << out_bad << "/" << n_words << ", sentinels left " << sentinel_left
              << "\n";

    const bool pass = (in_bad == 0 && out_bad == 0 && sentinel_left == 0);
    std::cout << (pass ? "TEST PASSED" : "TEST FAILED") << "\n";
    return pass ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "ERROR         : " << e.what() << "\n";
    return 3;
  }
}
