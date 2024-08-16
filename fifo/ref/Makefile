sim_test0:    ##wirte to full test full, and then w/r, and then r only.
	cp ./fifo_tb.v tb.v
	@sed -i 's|// *#(Simulation_time)// *test0|#(Simulation_time)|' tb.v
	@sed -i 's|// *start_test = 3;// *test0|start_test = 3;|' tb.v
	@sed -i 's|// *test0|test0|' tb.v
	xvlog -f ./file_list tb.v
	xelab -top fifo_tb -debug all -snapshot fifo_tb_elab
	xsim fifo_tb_elab -R | tee sim.log
	make check_log

sim_test1:     ##wirte to full test full, and then r only.
	cp ./fifo_tb.v tb.v
	@sed -i 's|// *test1|test1|' tb.v
	xvlog -f ./file_list tb.v
	xelab -top fifo_tb -debug all -snapshot fifo_tb_elab
	xsim fifo_tb_elab -R | tee sim.log
	make check_log
sim_test2:          ##  w/r, and then r only
	cp ./fifo_tb.v tb.v
	@sed -i 's|// *test2|test2|' tb.v
	xvlog -f ./file_list tb.v
	xelab -top fifo_tb -debug all -snapshot fifo_tb_elab
	xsim fifo_tb_elab -R | tee sim.log
	make check_log
sim_test3:           ## random test
	cp ./fifo_tb.v tb.v
	@sed -i "s/reg \[DATA_WIDTH-1:0\] seed=1;/reg \[DATA_WIDTH-1:0\] seed=$$(awk 'BEGIN{srand(); print int(1+rand()*100)}');/" tb.v
	@sed -i 's|// *test3|test3|' tb.v
	xvlog -f ./file_list tb.v
	xelab -top fifo_tb -debug all -snapshot fifo_tb_elab
	xsim fifo_tb_elab -R | tee sim.log
	make check_log
sim_test4:      ## self_defined test
	cp ./fifo_tb.v tb.v
	xvlog -f ./file_list tb.v
	xelab -top fifo_tb -debug all -snapshot fifo_tb_elab
	xsim fifo_tb_elab -R | tee sim.log
	make check_log

check_log:
	grep 'x' sim.log 

clean:
	## -rf: remove recursively and forcefully
	rm -rf xsim.dir/ *.log *.pb *.jou *.wdb *.vcd tb.v
