// hbm_bw_host: XRT host for the hbm_bw bandwidth kernel on Alveo U280.
//
// Measures per-pseudo-channel HBM bandwidth using the kernel's own cycle
// counters, which is exact, rather than host wall-clock time, which is swamped
// by XRT command overhead for short kernels (see method.md).
//
//   mode read   stream in_ptr (HBM[0]) and fold it into a checksum
//   mode write  stream a generated pattern to out_ptr (HBM[1])
//   mode copy   in_ptr -> on-chip FIFO -> out_ptr, streaming
//
// The counters live in the kernel's register space but are not kernel
// arguments, so the CU must be opened for exclusive access for read_register().
//
// Usage:
//   hbm_bw_host -x <xclbin> [-d <bdf>] [-m read|write|copy|all] [-n <beats>]
//               [-b <burst_beats>] [-o <outstanding>] [-f <kernel_MHz>] [-S] [-q]

#include <xrt/xrt_bo.h>
#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>

#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr const char* kKernelName   = "hbm_bw";
constexpr uint32_t    kWordsPerBeat = 16;     // 512-bit beat
constexpr uint32_t    kBytesPerBeat = 64;
constexpr uint32_t    kMaxBurst     = 64;     // beats; matches MAX_BURST in hbm_bw.v
constexpr uint32_t    kMaxOut       = 32;     // matches MAX_OUT_HW in hbm_bw.v
constexpr uint32_t    kFifoDepth    = 1024;   // beats; matches FIFO_DEPTH in hbm_bw.v

// kernel register offsets (hbm_bw_ctrl.v)
enum : uint32_t {
  R_CYCLES = 0x50, R_RBEATS = 0x54, R_WBEATS = 0x58, R_AR = 0x5C,
  R_AW     = 0x60, R_B      = 0x64, R_RSTALL = 0x68, R_WSTALL = 0x6C,
  R_WSTARV = 0x70, R_LATF   = 0x74, R_CKSUM  = 0x78, R_FLAGS  = 0x7C,
  R_MOUT_R = 0x80, R_MOUT_W = 0x84
};

enum Mode : uint32_t { MODE_READ = 0, MODE_WRITE = 1, MODE_COPY = 2 };

struct Counters {
  uint32_t cycles, r_beats, w_beats, ar, aw, b;
  uint32_t r_stall, w_stall, w_starve, lat_first, checksum, flags;
  uint32_t max_out_r, max_out_w;
};

// Must match the kernel's generated pattern and the testbench: the input buffer
// holds a hash of the word index, never the index itself. An affine pattern
// would make the kernel's XOR-fold checksum identically zero.
inline uint32_t pattern(uint32_t word_index) { return word_index * 0x9E3779B1u; }

Counters read_counters(const xrt::kernel& k) {
  Counters c{};
  c.cycles    = k.read_register(R_CYCLES);
  c.r_beats   = k.read_register(R_RBEATS);
  c.w_beats   = k.read_register(R_WBEATS);
  c.ar        = k.read_register(R_AR);
  c.aw        = k.read_register(R_AW);
  c.b         = k.read_register(R_B);
  c.r_stall   = k.read_register(R_RSTALL);
  c.w_stall   = k.read_register(R_WSTALL);
  c.w_starve  = k.read_register(R_WSTARV);
  c.lat_first = k.read_register(R_LATF);
  c.checksum  = k.read_register(R_CKSUM);
  c.flags     = k.read_register(R_FLAGS);
  c.max_out_r = k.read_register(R_MOUT_R);
  c.max_out_w = k.read_register(R_MOUT_W);
  return c;
}

double gbps(uint32_t beats, uint32_t cycles, double mhz) {
  if (cycles == 0) return 0.0;
  return (static_cast<double>(beats) * kBytesPerBeat * mhz) / (static_cast<double>(cycles) * 1000.0);
}

uint32_t expected_checksum(uint32_t num_beats) {
  uint32_t sum = 0;
  for (uint32_t n = 0; n < num_beats; ++n) {
    uint32_t fold = 0;
    for (uint32_t l = 0; l < kWordsPerBeat; ++l) fold ^= pattern(n * kWordsPerBeat + l);
    sum += fold;
  }
  return sum;
}

const char* mode_name(uint32_t m) {
  return m == MODE_READ ? "read" : (m == MODE_WRITE ? "write" : "copy");
}

void usage(const char* prog) {
  std::cerr << "Usage: " << prog << " -x <xclbin> [options]\n"
            << "  -x  xclbin (required)\n"
            << "  -d  device BDF (default 0000:3b:00.1; ignored under XCL_EMULATION_MODE)\n"
            << "  -m  mode: read | write | copy | all (default read)\n"
            << "  -n  transfer length in 512-bit beats (default 65536 = 4 MB)\n"
            << "  -b  beats per burst, power of two 1.." << kMaxBurst << " (default 64)\n"
            << "  -o  outstanding bursts, 1.." << kMaxOut << " (default 32)\n"
            << "  -f  kernel clock in MHz for the GB/s conversion (default 300)\n"
            << "  -S  sweep burst size x outstanding in read mode\n"
            << "  -q  quiet: one line per run\n";
}

}  // namespace

int main(int argc, char** argv) {
  const char* emu = std::getenv("XCL_EMULATION_MODE");
  std::string xclbin;
  std::string bdf       = "0000:3b:00.1";
  std::string mode_str  = "read";
  uint32_t    beats     = 65536;
  uint32_t    burst     = 64;
  uint32_t    out       = 32;
  double      mhz       = 300.0;
  bool        sweep     = false;
  bool        quiet     = false;

  int opt;
  while ((opt = getopt(argc, argv, "x:d:m:n:b:o:f:Sqh")) != -1) {
    switch (opt) {
      case 'x': xclbin   = optarg; break;
      case 'd': bdf      = optarg; break;
      case 'm': mode_str = optarg; break;
      case 'n': beats    = static_cast<uint32_t>(std::strtoul(optarg, nullptr, 0)); break;
      case 'b': burst    = static_cast<uint32_t>(std::strtoul(optarg, nullptr, 0)); break;
      case 'o': out      = static_cast<uint32_t>(std::strtoul(optarg, nullptr, 0)); break;
      case 'f': mhz      = std::strtod(optarg, nullptr); break;
      case 'S': sweep    = true; break;
      case 'q': quiet    = true; break;
      default:  usage(argv[0]); return 1;
    }
  }
  if (xclbin.empty() || beats == 0 || burst == 0 || burst > kMaxBurst ||
      (burst & (burst - 1)) != 0 || out == 0 || out > kMaxOut) {
    usage(argv[0]);
    return 1;
  }

  std::vector<uint32_t> modes;
  if (mode_str == "read")       modes = {MODE_READ};
  else if (mode_str == "write") modes = {MODE_WRITE};
  else if (mode_str == "copy")  modes = {MODE_COPY};
  else if (mode_str == "all")   modes = {MODE_READ, MODE_WRITE, MODE_COPY};
  else { usage(argv[0]); return 1; }

  const size_t n_words = static_cast<size_t>(beats) * kWordsPerBeat;
  const size_t n_bytes = static_cast<size_t>(beats) * kBytesPerBeat;
  int failures = 0;

  try {
    xrt::device device = emu ? xrt::device(0) : xrt::device(bdf);
    auto uuid = device.load_xclbin(xclbin);
    // exclusive access: required by read_register() for the counters
    auto kernel = xrt::kernel(device, uuid, kKernelName, xrt::kernel::cu_access_mode::exclusive);

    std::cout << "Device      : " << (emu ? std::string("index 0 (") + emu + ")" : bdf) << "\n"
              << "xclbin      : " << xclbin << "\n"
              << "Kernel      : " << kKernelName << " (exclusive), in_ptr group "
              << kernel.group_id(1) << ", out_ptr group " << kernel.group_id(2) << "\n"
              << "Transfer    : " << beats << " beats = " << n_bytes / 1024.0 << " KiB per buffer\n"
              << "Clock       : " << mhz << " MHz assumed for GB/s (-f to change)\n";

    auto bo_in  = xrt::bo(device, n_bytes, xrt::bo::flags::normal, kernel.group_id(1));
    auto bo_out = xrt::bo(device, n_bytes, xrt::bo::flags::normal, kernel.group_id(2));
    auto in     = bo_in.map<uint32_t*>();
    auto outp   = bo_out.map<uint32_t*>();

    for (size_t j = 0; j < n_words; ++j) in[j] = pattern(static_cast<uint32_t>(j));
    bo_in.sync(XCL_BO_SYNC_BO_TO_DEVICE);
    std::cout << "Buffers     : in @ 0x" << std::hex << bo_in.address()
              << ", out @ 0x" << bo_out.address() << std::dec << "\n";
    if ((bo_in.address() & 0xFFF) || (bo_out.address() & 0xFFF))
      std::cout << "WARNING     : buffer not 4 KB aligned; bursts may cross page boundaries\n";

    const uint32_t exp_cksum = expected_checksum(beats);

    auto do_run = [&](uint32_t mode, uint32_t blen, uint32_t mout, bool verify) -> Counters {
      if (mode != MODE_READ) {
        std::memset(outp, 0xA5, n_bytes);
        bo_out.sync(XCL_BO_SYNC_BO_TO_DEVICE);
      }
      auto run   = kernel(mode, bo_in, bo_out, beats, blen, mout);
      auto state = run.wait(std::chrono::milliseconds(1000));
      while (state == ERT_CMD_STATE_TIMEOUT) state = run.wait(std::chrono::milliseconds(1000));
      if (state != ERT_CMD_STATE_COMPLETED)
        throw std::runtime_error("kernel ended in state " + std::to_string(static_cast<int>(state)));

      Counters c = read_counters(kernel);
      const uint32_t exp_bursts = (beats + blen - 1) / blen;
      std::vector<std::string> errs;

      if (c.flags != 0) errs.push_back("flags=0x" + std::to_string(c.flags));
      if (mode == MODE_READ || mode == MODE_COPY) {
        if (c.r_beats != beats)   errs.push_back("r_beats=" + std::to_string(c.r_beats));
        if (c.ar != exp_bursts)   errs.push_back("ar=" + std::to_string(c.ar));
        if (c.max_out_r > mout)   errs.push_back("max_out_r=" + std::to_string(c.max_out_r));
      }
      if (mode == MODE_WRITE || mode == MODE_COPY) {
        if (c.w_beats != beats)   errs.push_back("w_beats=" + std::to_string(c.w_beats));
        if (c.aw != exp_bursts)   errs.push_back("aw=" + std::to_string(c.aw));
        if (c.b != exp_bursts)    errs.push_back("b=" + std::to_string(c.b));
        if (c.max_out_w > mout)   errs.push_back("max_out_w=" + std::to_string(c.max_out_w));
      }
      if (mode == MODE_READ && c.checksum != exp_cksum)
        errs.push_back("checksum=0x" + std::to_string(c.checksum));

      if (verify && mode != MODE_READ) {
        bo_out.sync(XCL_BO_SYNC_BO_FROM_DEVICE);
        size_t bad = 0;
        for (size_t j = 0; j < n_words; ++j) {
          const uint32_t want = (mode == MODE_WRITE) ? static_cast<uint32_t>(j)
                                                     : pattern(static_cast<uint32_t>(j));
          if (outp[j] != want) {
            if (bad < 4)
              std::cout << "  MISMATCH out[" << j << "]=0x" << std::hex << outp[j]
                        << " expected 0x" << want << std::dec << "\n";
            ++bad;
          }
        }
        if (bad) errs.push_back(std::to_string(bad) + "/" + std::to_string(n_words) + " words wrong");
      }

      if (!errs.empty()) {
        ++failures;
        std::cout << "  CHECK FAILED:";
        for (const auto& e : errs) std::cout << " " << e;
        std::cout << "\n";
      }
      return c;
    };

    if (sweep) {
      std::cout << "\nRead bandwidth sweep (m00_axi on one pseudo-channel)\n"
                << "  burst    outstanding   cycles     GB/s   beats/cyc  r_stall   lat(cyc)  lat(ns)  max_seen\n";
      for (uint32_t blen = 1; blen <= kMaxBurst; blen *= 2) {
        for (uint32_t mout = 1; mout <= kMaxOut; mout *= 2) {
          Counters c = do_run(MODE_READ, blen, mout, false);
          std::cout << "  " << std::setw(5) << blen * kBytesPerBeat << " B"
                    << std::setw(10) << mout
                    << std::setw(11) << c.cycles
                    << std::fixed << std::setprecision(2) << std::setw(9) << gbps(c.r_beats, c.cycles, mhz)
                    << std::setprecision(3) << std::setw(11)
                    << static_cast<double>(c.r_beats) / static_cast<double>(c.cycles)
                    << std::setw(10) << c.r_stall
                    << std::setw(10) << c.lat_first
                    << std::setprecision(1) << std::setw(9) << c.lat_first * 1000.0 / mhz
                    << std::setw(9) << c.max_out_r << "\n";
        }
      }
    }

    for (uint32_t mode : modes) {
      Counters c = do_run(mode, burst, out, true);
      const uint32_t moved = (mode == MODE_WRITE) ? c.w_beats : c.r_beats;
      std::cout << "\n[" << mode_name(mode) << "] burst=" << burst * kBytesPerBeat
                << " B outstanding=" << out << "\n"
                << "  cycles      : " << c.cycles << " (" << std::fixed << std::setprecision(1)
                << c.cycles * 1000.0 / mhz << " ns at " << mhz << " MHz)\n"
                << "  bandwidth   : " << std::setprecision(2) << gbps(moved, c.cycles, mhz)
                << " GB/s (" << std::setprecision(3)
                << static_cast<double>(moved) / static_cast<double>(c.cycles) << " beats/cycle)\n";
      if (!quiet && mode != MODE_WRITE)
        std::cout << "  read        : " << c.r_beats << " beats, " << c.ar << " bursts, stall "
                  << c.r_stall << " cyc, first-read latency " << c.lat_first << " cyc ("
                  << std::setprecision(1) << c.lat_first * 1000.0 / mhz << " ns), max outstanding "
                  << c.max_out_r << "\n";
      if (!quiet && mode != MODE_READ)
        std::cout << "  write       : " << c.w_beats << " beats, " << c.aw << " bursts, " << c.b
                  << " responses, stall " << c.w_stall << " cyc, starve " << c.w_starve
                  << " cyc, max outstanding " << c.max_out_w << "\n";
      if (!quiet && mode == MODE_READ)
        std::cout << "  checksum    : 0x" << std::hex << c.checksum << " (expected 0x" << exp_cksum
                  << ")" << std::dec << "\n";
    }

    std::cout << "\n" << (failures == 0 ? "TEST PASSED" : "TEST FAILED") << "\n";
    return failures == 0 ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "ERROR       : " << e.what() << "\n";
    return 3;
  }
}
