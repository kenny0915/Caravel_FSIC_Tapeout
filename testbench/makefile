
#.PHONY: run_edge_rtl
#run_edge_rtl:
#	$(MAKE) rtlcom -C sim_vcs_edge
#	cp sim_vcs_edge/compile.log sim_vcs_edge/compile.log.rtl
	
#.PHONY: run_edge_post
#run_edge_post:
#	$(MAKE) postcom -C sim_vcs_edge
#	cp sim_vcs_edge/compile.log sim_vcs_edge/compile.log.post
	
#.PHONY: run_edge
#run_edge: run_edge_rtl run_edge_post

.PHONY: clean
clean:
	rm -rf sim_vcs_system_test001

	rm -rf sim_vcs_system_test002

	rm -rf sim_vcs_system_test003

	rm -rf sim_vcs_system_test004

	rm -rf sim_vcs_system_test005

	rm -rf sim_vcs_system_test006
	
	rm -rf sim_vcs_system_test007
	
	rm -rf sim_vcs_system_test008

# $(MAKE) clean -C sim_vcs_edge

TARGETS = test001 test002 test003 test004 test005 test006 test007 test008

# Define variables for clarity (assuming sim_vcs_system is the common path)
SIM_DIR = sim_vcs_system

.PHONY: dir_rtl

dir_rtl:
	for dir in $(TARGETS); do \
		$(MAKE) rtlcom -C $(SIM_DIR)_$$dir; \
		cp  $(SIM_DIR)_$$dir/compile.log $(SIM_DIR)_$$dir/compile.log.rtl; \
	done

.PHONY: dir_post
dir_post:
	for dir in $(TARGETS); do \
		$(MAKE) postcom -C $(SIM_DIR)_$$dir; \
		cp  $(SIM_DIR)_$$dir/compile.log $(SIM_DIR)_$$dir/compile.log.post; \
	done

pre_sim: dir_rtl dir_post #run_edge youwei comment delete edge part

SED_CMD = sed -e 's/^\/\/`define\s*SYSTEM_$$dir\s*1/`define SYSTEM_$$dir 1/' testbench/top_bench.sv 

.PHONY: setup_sed
setup_sed:

	for dir in $(TARGETS); do \
		$(SED_CMD) > $(SIM_DIR)_$$dir/top_bench.sv; \
	done

.PHONY: setup_new
setup_new:

	for dir in $(TARGETS); do \
		rm -rf $(SIM_DIR)_$$dir; \
		cp -r system $(SIM_DIR)_$$dir; \
		sed -e 's/^\/\/`define\s*SYSTEM_$$dir\s*1/`define SYSTEM_$$dir 1/' testbench/top_bench.sv > $(SIM_DIR)_$$dir/top_bench.sv; \
		cp system/hex/system_$$dir.hex $(SIM_DIR)_$$dir/riscv.hex; \
	done

.PHONY: setup
setup:
	rm -rf sim_vcs_system_test001
	cp -r system sim_vcs_system_test001
	sed -e 's/^\/\/`define\s*SYSTEM_test001\s*1/`define SYSTEM_test001 1/' testbench/top_bench.sv > sim_vcs_system_test001/top_bench.sv

	rm -rf sim_vcs_system_test002
	cp -r system sim_vcs_system_test002
	sed -e 's/^\/\/`define\s*SYSTEM_test002\s*1/`define SYSTEM_test002 1/' testbench/top_bench.sv > sim_vcs_system_test002/top_bench.sv

	rm -rf sim_vcs_system_test003
	cp -r system sim_vcs_system_test003
	sed -e 's/^\/\/`define\s*SYSTEM_test003\s*1/`define SYSTEM_test003 1/' testbench/top_bench.sv > sim_vcs_system_test003/top_bench.sv

	rm -rf sim_vcs_system_test004
	cp -r system sim_vcs_system_test004
	sed -e 's/^\/\/`define\s*SYSTEM_test004\s*1/`define SYSTEM_test004 1/' testbench/top_bench.sv > sim_vcs_system_test004/top_bench.sv

	rm -rf sim_vcs_system_test005
	cp -r system sim_vcs_system_test005
	sed -e 's/^\/\/`define\s*SYSTEM_test005\s*1/`define SYSTEM_test005 1/' testbench/top_bench.sv > sim_vcs_system_test005/top_bench.sv

	rm -rf sim_vcs_system_test006
	cp -r system sim_vcs_system_test006
	sed -e 's/^\/\/`define\s*SYSTEM_test006\s*1/`define SYSTEM_test006 1/' testbench/top_bench.sv > sim_vcs_system_test006/top_bench.sv
	
	rm -rf sim_vcs_system_test007
	cp -r system sim_vcs_system_test007
	sed -e 's/^\/\/`define\s*SYSTEM_test007\s*1/`define SYSTEM_test007 1/' testbench/top_bench.sv > sim_vcs_system_test007/top_bench.sv

	rm -rf sim_vcs_system_test008
	cp -r system sim_vcs_system_test008
	sed -e 's/^\/\/`define\s*SYSTEM_test008\s*1/`define SYSTEM_test008 1/' testbench/top_bench.sv > sim_vcs_system_test008/top_bench.sv

	
