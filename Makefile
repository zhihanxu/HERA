# HBM RTL kernel test on Alveo U280
#   M1: make sim
#   M2: make xo synth_check
#   M3: make host hw_emu run_emu
#   M4: make hw run_hw
#   M7: make skel_sim skel_synth_check skel_xo skel_run_hw  (see skel/README.md)
SHELL    := /bin/bash
ROOT     := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
BUILD    := $(ROOT)/build

KERNEL   := hbm_scratch
PART     := xcu280-fsvh2892-2L-e
PLATFORM := xilinx_u280_gen3x16_xdma_1_202211_1
XPFM     := /opt/xilinx/platforms/$(PLATFORM)/$(PLATFORM).xpfm
CFG      := $(ROOT)/cfg/u280_hbm.cfg
XO       := $(BUILD)/$(KERNEL).xo
VIVADO   := vivado -mode batch -notrace

HOST     := $(BUILD)/host/hbm_host
EMU_DIR  := $(BUILD)/hw_emu
EMU_XCLBIN := $(EMU_DIR)/$(KERNEL).xclbin
EMU_N    ?= 100

HW_DIR     := $(BUILD)/hw
HW_XCLBIN  := $(HW_DIR)/$(KERNEL).xclbin
DEVICE_BDF ?= 0000:3b:00.1
HW_N       ?= 1000
VPP_JOBS   ?= 16

RTL_SRCS := $(wildcard $(ROOT)/rtl/*.v)

export TMPDIR := $(BUILD)/tmp

XO_BW := $(BUILD)/hbm_bw.xo
RTL_BW_SRCS := $(wildcard $(ROOT)/rtl/bw/*.v)

XO_RW       := $(BUILD)/hbm_rw.xo
RTL_RW_SRCS := $(ROOT)/rtl/rw/hbm_rw.v $(ROOT)/rtl/bw/hbm_bw_ctrl.v \
               $(ROOT)/rtl/bw/hbm_bw_read.v $(ROOT)/rtl/bw/hbm_bw_write.v \
               $(ROOT)/rtl/bw/sync_fifo.v

CFG_RW32      := $(ROOT)/cfg/u280_hbm_rw32.cfg
CFG_RW4       := $(ROOT)/cfg/u280_hbm_rw4.cfg
HOST_RW       := $(BUILD)/host/hbm_rw_host
EMU_RW_DIR    := $(BUILD)/hw_emu_rw4
EMU_RW_XCLBIN := $(EMU_RW_DIR)/hbm_rw.xclbin
EMU_RW_N      ?= 256
HW_RW_DIR     := $(BUILD)/hw_rw32
HW_RW_XCLBIN  := $(HW_RW_DIR)/hbm_rw.xclbin
HW_RW_N       ?= 524288
HW_RW_CU      ?= 32
HW_RW_B       ?= 64
# 8, not 32: the M6.4 build sets MAX_OUT_HW=8 to shrink the platform's per-port
# buffers, and M5.4 showed 4 KB bursts saturate a channel at 2-4 outstanding.
HW_RW_O       ?= 8
# read, not all: the M6.4 build sets ENABLE_COPY=0, so mode 2 (same-channel copy)
# is deliberately rejected by the RTL. Use `make run_hw_rw32 HW_RW_M=write` for
# the write direction; copy needs an ENABLE_COPY=1 build.
HW_RW_M       ?= read
HW_RW_F       ?= 300

CFG_BW        := $(ROOT)/cfg/u280_hbm_bw.cfg
HOST_BW       := $(BUILD)/host/hbm_bw_host
EMU_BW_DIR    := $(BUILD)/hw_emu_bw
EMU_BW_XCLBIN := $(EMU_BW_DIR)/hbm_bw.xclbin
EMU_BW_N      ?= 256
HW_BW_DIR     := $(BUILD)/hw_bw
HW_BW_XCLBIN  := $(HW_BW_DIR)/hbm_bw.xclbin
HW_BW_N       ?= 65536
HW_BW_B       ?= 64
HW_BW_O       ?= 32
HW_BW_F       ?= 300

.PHONY: sim xo synth_check host hw_emu run_emu hw run_hw sim_bw xo_bw synth_check_bw \
        host_bw hw_emu_bw run_emu_bw hw_bw run_hw_bw sim_rw xo_rw synth_check_rw host_rw hw_emu_rw4 run_emu_rw4 hw_rw32 run_hw_rw32 clean

## M1: RTL simulation (xvlog/xelab/xsim, batch)
sim:
	@mkdir -p $(TMPDIR)
	bash $(ROOT)/scripts/sim.sh

## M6.1: single-master hbm_rw kernel simulation (one channel, both directions)
sim_rw:
	@mkdir -p $(TMPDIR)
	bash $(ROOT)/scripts/sim_rw.sh

## M5.1: hbm_bw bandwidth kernel simulation + sweep
sim_bw:
	@mkdir -p $(TMPDIR)
	bash $(ROOT)/scripts/sim_bw.sh

## M6.2: package hbm_rw (single master) as a Vitis kernel object
xo_rw: $(XO_RW)

$(XO_RW): $(RTL_RW_SRCS) $(ROOT)/scripts/package_rw.tcl $(ROOT)/scripts/kernel_rw.xml
	@mkdir -p $(TMPDIR) $(BUILD)/pkg_rw
	cd $(BUILD)/pkg_rw && $(VIVADO) -source $(ROOT)/scripts/package_rw.tcl \
		-log package_rw.log -journal package_rw.jou -tclargs $(PART)

## M6.2: out-of-context timing check for ONE hbm_rw CU at 300 MHz
synth_check_rw:
	@mkdir -p $(TMPDIR) $(BUILD)/synth_rw
	cd $(BUILD)/synth_rw && $(VIVADO) -source $(ROOT)/scripts/synth_check_rw.tcl \
		-log synth_check_rw.log -journal synth_check_rw.jou -tclargs $(PART)

## M5.2: package hbm_bw as a Vitis kernel object
xo_bw: $(XO_BW)

$(XO_BW): $(RTL_BW_SRCS) $(ROOT)/scripts/package_bw.tcl $(ROOT)/scripts/kernel_bw.xml
	@mkdir -p $(TMPDIR) $(BUILD)/pkg_bw
	cd $(BUILD)/pkg_bw && $(VIVADO) -source $(ROOT)/scripts/package_bw.tcl \
		-log package_bw.log -journal package_bw.jou -tclargs $(PART)

## M5.2: out-of-context synth/place/route timing check for hbm_bw at 300 MHz
synth_check_bw:
	@mkdir -p $(TMPDIR) $(BUILD)/synth_bw
	cd $(BUILD)/synth_bw && $(VIVADO) -source $(ROOT)/scripts/synth_check_bw.tcl \
		-log synth_check_bw.log -journal synth_check_bw.jou -tclargs $(PART)

## M2: package the RTL as a Vitis kernel object
xo: $(XO)

$(XO): $(RTL_SRCS) $(ROOT)/scripts/package_kernel.tcl $(ROOT)/scripts/kernel.xml
	@mkdir -p $(TMPDIR) $(BUILD)/pkg
	cd $(BUILD)/pkg && $(VIVADO) -source $(ROOT)/scripts/package_kernel.tcl \
		-log package_kernel.log -journal package_kernel.jou -tclargs $(PART)

## M2: out-of-context synth/place/route timing check at 300 MHz
synth_check:
	@mkdir -p $(TMPDIR) $(BUILD)/synth
	cd $(BUILD)/synth && $(VIVADO) -source $(ROOT)/scripts/synth_check.tcl \
		-log synth_check.log -journal synth_check.jou -tclargs $(PART)

## M3: XRT host application
host: $(HOST)

$(HOST): $(ROOT)/host/host.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 -O2 -Wall -Wextra -o $@ $< \
		-I$(XILINX_XRT)/include -L$(XILINX_XRT)/lib -lxrt_coreutil -pthread

## M3: hardware-emulation xclbin + emulation config
hw_emu: $(EMU_XCLBIN) $(EMU_DIR)/emconfig.json

$(EMU_XCLBIN): $(XO) $(CFG)
	@mkdir -p $(TMPDIR) $(EMU_DIR)
	cd $(EMU_DIR) && v++ -l -t hw_emu --platform $(XPFM) --config $(CFG) \
		--temp_dir $(EMU_DIR)/_x --log_dir $(EMU_DIR)/logs --report_dir $(EMU_DIR)/reports \
		-o $@ $(XO)

$(EMU_DIR)/emconfig.json:
	@mkdir -p $(EMU_DIR)
	cd $(EMU_DIR) && emconfigutil --platform $(XPFM) --od $(EMU_DIR)

## M3: run the host against hardware emulation
run_emu: host hw_emu
	cd $(EMU_DIR) && EMCONFIG_PATH=$(EMU_DIR) XCL_EMULATION_MODE=hw_emu $(HOST) -x $(EMU_XCLBIN) -n $(EMU_N)

## M4: hardware xclbin (full synthesis + implementation, ~2 h)
hw: $(HW_XCLBIN)

$(HW_XCLBIN): $(XO) $(CFG)
	@mkdir -p $(TMPDIR) $(HW_DIR)
	cd $(HW_DIR) && v++ -l -t hw --platform $(XPFM) --config $(CFG) \
		--vivado.synth.jobs $(VPP_JOBS) --vivado.impl.jobs $(VPP_JOBS) \
		--temp_dir $(HW_DIR)/_x --log_dir $(HW_DIR)/logs --report_dir $(HW_DIR)/reports \
		-o $@ $(XO)

## M4: run the host on the U280: host memory -> HBM -> kernel -> HBM -> host memory
run_hw: host hw
	cd $(HW_DIR) && $(HOST) -x $(HW_XCLBIN) -d $(DEVICE_BDF) -n $(HW_N)

## M5.3: XRT host for hbm_bw (reads performance counters)
host_bw: $(HOST_BW)

$(HOST_BW): $(ROOT)/host/host_bw.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 -O2 -Wall -Wextra -o $@ $< \
		-I$(XILINX_XRT)/include -L$(XILINX_XRT)/lib -lxrt_coreutil -pthread

## M5.3: hardware-emulation xclbin for hbm_bw
hw_emu_bw: $(EMU_BW_XCLBIN) $(EMU_BW_DIR)/emconfig.json

$(EMU_BW_XCLBIN): $(XO_BW) $(CFG_BW)
	@mkdir -p $(TMPDIR) $(EMU_BW_DIR)
	cd $(EMU_BW_DIR) && v++ -l -t hw_emu --platform $(XPFM) --config $(CFG_BW) \
		--temp_dir $(EMU_BW_DIR)/_x --log_dir $(EMU_BW_DIR)/logs --report_dir $(EMU_BW_DIR)/reports \
		-o $@ $(XO_BW)

$(EMU_BW_DIR)/emconfig.json:
	@mkdir -p $(EMU_BW_DIR)
	cd $(EMU_BW_DIR) && emconfigutil --platform $(XPFM) --od $(EMU_BW_DIR)

## M5.3: run hbm_bw under hardware emulation (small transfer)
run_emu_bw: host_bw hw_emu_bw
	cd $(EMU_BW_DIR) && EMCONFIG_PATH=$(EMU_BW_DIR) XCL_EMULATION_MODE=hw_emu \
		$(HOST_BW) -x $(EMU_BW_XCLBIN) -n $(EMU_BW_N) -b 16 -o 8 -m all

## M5.4: hardware xclbin for hbm_bw (~2 h)
hw_bw: $(HW_BW_XCLBIN)

$(HW_BW_XCLBIN): $(XO_BW) $(CFG_BW)
	@mkdir -p $(TMPDIR) $(HW_BW_DIR)
	cd $(HW_BW_DIR) && v++ -l -t hw --platform $(XPFM) --config $(CFG_BW) \
		--vivado.synth.jobs $(VPP_JOBS) --vivado.impl.jobs $(VPP_JOBS) \
		--temp_dir $(HW_BW_DIR)/_x --log_dir $(HW_BW_DIR)/logs --report_dir $(HW_BW_DIR)/reports \
		-o $@ $(XO_BW)

## M5.4: per-pseudo-channel bandwidth measurement on the U280
run_hw_bw: host_bw hw_bw
	cd $(HW_BW_DIR) && $(HOST_BW) -x $(HW_BW_XCLBIN) -d $(DEVICE_BDF) \
		-n $(HW_BW_N) -b $(HW_BW_B) -o $(HW_BW_O) -f $(HW_BW_F) -m all -S

## M6.3: multi-CU XRT host
host_rw: $(HOST_RW)

$(HOST_RW): $(ROOT)/host/host_rw.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 -O2 -Wall -Wextra -o $@ $< \
		-I$(XILINX_XRT)/include -L$(XILINX_XRT)/lib -lxrt_coreutil -pthread

## M6.3: 4-CU hardware-emulation xclbin (validates the multi-CU host)
hw_emu_rw4: $(EMU_RW_XCLBIN) $(EMU_RW_DIR)/emconfig.json

$(EMU_RW_XCLBIN): $(XO_RW) $(CFG_RW4)
	@mkdir -p $(TMPDIR) $(EMU_RW_DIR)
	cd $(EMU_RW_DIR) && v++ -l -t hw_emu --platform $(XPFM) --config $(CFG_RW4) \
		--temp_dir $(EMU_RW_DIR)/_x --log_dir $(EMU_RW_DIR)/logs --report_dir $(EMU_RW_DIR)/reports \
		-o $@ $(XO_RW)

$(EMU_RW_DIR)/emconfig.json:
	@mkdir -p $(EMU_RW_DIR)
	cd $(EMU_RW_DIR) && emconfigutil --platform $(XPFM) --od $(EMU_RW_DIR)

run_emu_rw4: host_rw hw_emu_rw4
	cd $(EMU_RW_DIR) && EMCONFIG_PATH=$(EMU_RW_DIR) XCL_EMULATION_MODE=hw_emu \
		$(HOST_RW) -x $(EMU_RW_XCLBIN) -c 4 -n $(EMU_RW_N) -b 16 -o 8 -m all

## M6.4: 32-CU hardware xclbin, one CU per HBM pseudo-channel (3-5 h)
hw_rw32: $(HW_RW_XCLBIN)

$(HW_RW_XCLBIN): $(XO_RW) $(CFG_RW32)
	@mkdir -p $(TMPDIR) $(HW_RW_DIR)
	cd $(HW_RW_DIR) && v++ -l -t hw --platform $(XPFM) --config $(CFG_RW32) \
		--vivado.synth.jobs $(VPP_JOBS) --vivado.impl.jobs $(VPP_JOBS) \
		--temp_dir $(HW_RW_DIR)/_x --log_dir $(HW_RW_DIR)/logs --report_dir $(HW_RW_DIR)/reports \
		-o $@ $(XO_RW)

## M6.4: all-32-channel bandwidth measurement on the U280
run_hw_rw32: host_rw hw_rw32
	cd $(HW_RW_DIR) && $(HOST_RW) -x $(HW_RW_XCLBIN) -d $(DEVICE_BDF) -c $(HW_RW_CU) \
		-n $(HW_RW_N) -b $(HW_RW_B) -o $(HW_RW_O) -f $(HW_RW_F) -m $(HW_RW_M)

## M7: reusable all-channel HBM skeleton.
#
# skel/ is deliberately self-contained -- its own Makefile, config.mk, RTL,
# testbench and scripts -- because a skeleton gets copied wholesale into a
# future project and has to work there without this file. These targets only
# forward to it, so the project root stays a single entry point:
#
#   make skel_config        print the resolved skeleton configuration
#   make skel_sim           xsim the seam, 6 configurations
#   make skel_synth_check   one CU: timing + what the user logic costs
#   make skel_xo            package one CU
#   make skel_hw_emu        link the 4-CU emulation xclbin
#   make skel_hw            link the 32-CU hardware xclbin (hours)
#   make skel_host          build the skeleton host
#   make skel_run_emu       run the host against the emulation xclbin
#   make skel_run_hw        run the host on the card
#   make skel_sim_multi     xsim the experimental multi-master variant
#   make skel ARGS="xo USER_LOGIC=examples/user_logic_gen.v"
SKEL_DIR := $(ROOT)/skel

.PHONY: skel
skel:
	$(MAKE) -C $(SKEL_DIR) $(ARGS)

skel_%:
	$(MAKE) -C $(SKEL_DIR) $*

clean:
	rm -rf $(BUILD)
