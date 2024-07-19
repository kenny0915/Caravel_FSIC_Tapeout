`include "fsic_defines.v"

`default_nettype none

`timescale 1 ns / 1 ps

module top_bench #(
    parameter BITS=32,
    parameter pUSER_PROJECT_SIDEBAND_WIDTH = 5,
`ifdef USER_PROJECT_SIDEBAND_SUPPORT
    parameter pSERIALIO_WIDTH = 13,
`else
    parameter pSERIALIO_WIDTH = 12,
`endif
	parameter pADDR_WIDTH = 15,
	parameter pDATA_WIDTH = 32,
	parameter IOCLK_Period = 10,
	parameter DLYCLK_Period	= 1,
	parameter SHIFT_DEPTH = 5,
	parameter pRxFIFO_DEPTH = 5,
	parameter pCLK_RATIO = 4
	)
(
);
//tony_debug  `include "bench_ini.svh"
	localparam CoreClkPhaseLoop	= 1;
	localparam UP_BASE=32'h3000_0000;
	localparam AA_BASE=32'h3000_2000;
	localparam IS_BASE=32'h3000_3000;

	localparam SOC_to_FPGA_MailBox_Base=28'h000_2000;
	localparam FPGA_to_SOC_AA_BASE=28'h000_2000;
	localparam FPGA_to_SOC_IS_BASE=28'h000_3000;
		
	localparam AA_MailBox_Reg_Offset=12'h000;
	localparam AA_Internal_Reg_Offset=12'h100;
		
	localparam TUSER_AXIS = 2'b00;
	localparam TUSER_AXILITE_WRITE = 2'b01;
	localparam TUSER_AXILITE_READ_REQ = 2'b10;
	localparam TUSER_AXILITE_READ_CPL = 2'b11;

	localparam TID_DN_UP = 2'b00;
	localparam TID_DN_AA = 2'b01;
	localparam TID_UP_UP = 2'b00;
	localparam TID_UP_AA = 2'b01;
	localparam TID_UP_LA = 2'b10;
    
    localparam BASE_OFFSET = 8;
    localparam RXD_OFFSET = BASE_OFFSET;
    localparam RXCLK_OFFSET = RXD_OFFSET + pSERIALIO_WIDTH;
    localparam TXD_OFFSET = RXCLK_OFFSET + 1;
    localparam TXCLK_OFFSET = TXD_OFFSET + pSERIALIO_WIDTH;
    localparam IOCLK_OFFSET = TXCLK_OFFSET + 1;
    localparam TXRX_WIDTH = IOCLK_OFFSET - BASE_OFFSET + 1;

	reg fpga_rst;
	reg fpga_resetb;	//POR reset	
	wire fpga_coreclk;

	//write addr channel
	reg fpga_axi_awvalid;
	reg [pADDR_WIDTH-1:0] fpga_axi_awaddr;
	wire fpga_axi_awready;
	
	//write data channel
	reg 	fpga_axi_wvalid;
	reg 	[pDATA_WIDTH-1:0] fpga_axi_wdata;
	reg 	[3:0] fpga_axi_wstrb;
	wire	fpga_axi_wready;
	
	//read addr channel
	reg 	fpga_axi_arvalid;
	reg 	[pADDR_WIDTH-1:0] fpga_axi_araddr;
	wire 	fpga_axi_arready;
	
	//read data channel
	wire 	fpga_axi_rvalid;
	wire 	[pDATA_WIDTH-1:0] fpga_axi_rdata;
	reg 	fpga_axi_rready;
	
	reg 	fpga_cc_is_enable;		//axi_lite enable
	
	reg [pDATA_WIDTH-1:0] fpga_as_is_tdata;
`ifdef USER_PROJECT_SIDEBAND_SUPPORT
    reg [pUSER_PROJECT_SIDEBAND_WIDTH-1:0] fpga_as_is_tupsb;
`endif
	reg [3:0] fpga_as_is_tstrb;
	reg [3:0] fpga_as_is_tkeep;
	reg fpga_as_is_tlast;
	reg [1:0] fpga_as_is_tid;
	reg fpga_as_is_tvalid;
	reg [1:0] fpga_as_is_tuser;
	reg fpga_as_is_tready;		//when local side axis switch Rxfifo size <= threshold then as_is_tready=0; this flow control mechanism is for notify remote side do not provide data with is_as_tvalid=1

	wire [pDATA_WIDTH-1:0] fpga_is_as_tdata;
`ifdef USER_PROJECT_SIDEBAND_SUPPORT
    wire [pUSER_PROJECT_SIDEBAND_WIDTH-1:0] fpga_is_as_tupsb;
`endif
	wire [3:0] fpga_is_as_tstrb;
	wire [3:0] fpga_is_as_tkeep;
	wire fpga_is_as_tlast;
	wire [1:0] fpga_is_as_tid;
	wire fpga_is_as_tvalid;
	wire [1:0] fpga_is_as_tuser;
	wire fpga_is_as_tready;		//when remote side axis switch Rxfifo size <= threshold then is_as_tready=0, this flow control mechanism is for notify local side do not provide data with as_is_tvalid=1

    reg [27:0] fpga_axilite_write_addr;

	reg[27:0] soc_to_fpga_mailbox_write_addr_expect_value;
	reg[3:0] soc_to_fpga_mailbox_write_addr_BE_expect_value;
	reg[31:0] soc_to_fpga_mailbox_write_data_expect_value;
	reg [31:0] soc_to_fpga_mailbox_write_addr_captured;
	reg [31:0] soc_to_fpga_mailbox_write_data_captured;
	event soc_to_fpga_mailbox_write_event;
    reg stream_data_addr_or_data; //0: address, 1: data, use to identify the write transaction from AA.

	reg [31:0] soc_to_fpga_axilite_read_cpl_expect_value;
	reg [31:0] soc_to_fpga_axilite_read_cpl_captured;
	event soc_to_fpga_axilite_read_cpl_event;

	reg [31:0] error_cnt;
	reg [31:0] check_cnt;
    reg finish_flag;
  
    wire [11:0] checkbits;

    assign checkbits = mprj_io_out[31:21];

    reg         power1;  // 3.3V
    reg         power2;  // 1.8V

    // External clock is used by default.  Make this artificially fast for the
    // simulation.  Normally this would be a slow clock and the digital PLL
    // would be the fast clock.
    //
    reg clock;
    reg io_clk;   // generated from FPGA in real system

    localparam pSOC_FREQ = 10.0;                 // 10 MHz
    localparam pIOS_FREQ = (pSOC_FREQ * 4.0);    // 40 MHz

    // Timing Order
    // POWER ==> CLOCK ==> RESET

    initial begin
        clock  = 0;
        wait(power2);
        #100;
        forever begin
            #(500.0 / pSOC_FREQ);
            clock = ~clock;
        end
    end

    initial begin
        io_clk  = 0;
        wait(power2);
        #100;
        forever begin
            #(500.0 / pIOS_FREQ);
        io_clk = ~io_clk;
        end
    end

    wire [11:0] rx_dat;
    wire        rx_clk;

    assign #1 rx_clk = io_clk;
    assign #2 rx_dat = 12'h000;

    // MPRJ_IO PIN PLANNING when pSERIALIO_WIDTH=13
    // --------------------------------
    // [20: 8]  I   RXD
    // [   21]  I   RXCLK

    // --------------------------------
    // [34:22]  O   TXD
    // [   35]  O   TXCLK

    // --------------------------------
    // [   36]  I   IO_CLK

    // MPRJ_IO PIN PLANNING when pSERIALIO_WIDTH=12
    // --------------------------------
    // [19: 8]  I   RXD
    // [   20]  I   RXCLK

    // --------------------------------
    // [32:21]  O   TXD
    // [   33]  O   TXCLK

    // --------------------------------
    // [   34]  I   IO_CLK

	wire [pSERIALIO_WIDTH-1:0] soc_serial_txd;
	wire soc_txclk;

    wire [pSERIALIO_WIDTH-1:0] fpga_serial_txd;
	wire fpga_txclk;

    assign mprj_io[IOCLK_OFFSET] = io_clk;

    assign mprj_io[RXCLK_OFFSET] = fpga_txclk;
    assign mprj_io[RXD_OFFSET +: pSERIALIO_WIDTH] = fpga_serial_txd;

    assign soc_txclk = mprj_io[TXCLK_OFFSET];
    assign soc_serial_txd = mprj_io[TXD_OFFSET +: pSERIALIO_WIDTH];

    caravel_top uut (
    `ifdef USE_POWER_PINS
        .vddio     (VDD3V3),
        .vddio_2   (VDD3V3),
        .vssio     (VSS),
        .vssio_2   (VSS),
        .vdda      (VDD3V3),
        .vssa      (VSS),
        .vccd      (VDD1V8),
        .vssd      (VSS),
        .vdda1     (VDD3V3),
        .vdda1_2   (VDD3V3),
        .vdda2     (VDD3V3),
        .vssa1     (VSS),
        .vssa1_2   (VSS),
        .vssa2     (VSS),
        .vccd1     (VDD1V8),
        .vccd2     (VDD1V8),
        .vssd1     (VSS),
        .vssd2     (VSS),
    `endif //USE_POWER_PINS
        .clock     (clock),
        .gpio      (gpio),
        .mprj_io   (mprj_io),
        .flash_csb (flash_csb),
        .flash_clk (flash_clk),
        .flash_io0 (flash_io0),
        .flash_io1 (flash_io1),
        .resetb    (RSTB) );

    spiflash #(.FILENAME("riscv.hex")) spiflash( 
        .csb(flash_csb),
        .clk(flash_clk),
        .io0(flash_io0),
        .io1(flash_io1),
        .io2(),          // not used
        .io3() );        // not used

	fsic_clock_div fpga_clock_div (
	    .resetb(fpga_resetb),
	    .in(ioclk),
	    .out(fpga_coreclk)
	);

    fpga  #(
		.pUSER_PROJECT_SIDEBAND_WIDTH(pUSER_PROJECT_SIDEBAND_WIDTH),
		.pSERIALIO_WIDTH(pSERIALIO_WIDTH),
		.pADDR_WIDTH(pADDR_WIDTH),
		.pDATA_WIDTH(pDATA_WIDTH),
		.pRxFIFO_DEPTH(pRxFIFO_DEPTH),
		.pCLK_RATIO(pCLK_RATIO)
	)
	fpga_fsic(
		.axis_rst_n(~fpga_rst),
		.axi_reset_n(~fpga_rst),
		.serial_tclk(fpga_txclk),
		.serial_rclk(soc_txclk),
		.ioclk(ioclk),
		.axis_clk(fpga_coreclk),
		.axi_clk(fpga_coreclk),
		
		//write addr channel
		.axi_awvalid_s_awvalid(fpga_axi_awvalid),
		.axi_awaddr_s_awaddr(fpga_axi_awaddr),
		.axi_awready_axi_awready3(fpga_axi_awready),

		//write data channel
		.axi_wvalid_s_wvalid(fpga_axi_wvalid),
		.axi_wdata_s_wdata(fpga_axi_wdata),
		.axi_wstrb_s_wstrb(fpga_axi_wstrb),
		.axi_wready_axi_wready3(fpga_axi_wready),

		//read addr channel
		.axi_arvalid_s_arvalid(fpga_axi_arvalid),
		.axi_araddr_s_araddr(fpga_axi_araddr),
		.axi_arready_axi_arready3(fpga_axi_arready),
		
		//read data channel
		.axi_rvalid_axi_rvalid3(fpga_axi_rvalid),
		.axi_rdata_axi_rdata3(fpga_axi_rdata),
		.axi_rready_s_rready(fpga_axi_rready),
		
		.cc_is_enable(fpga_cc_is_enable),


		.as_is_tdata(fpga_as_is_tdata),
    `ifdef USER_PROJECT_SIDEBAND_SUPPORT
        .as_is_tupsb  (fpga_as_is_tupsb),
    `endif
		.as_is_tstrb(fpga_as_is_tstrb),
		.as_is_tkeep(fpga_as_is_tkeep),
		.as_is_tlast(fpga_as_is_tlast),
		.as_is_tid(fpga_as_is_tid),
		.as_is_tvalid(fpga_as_is_tvalid),
		.as_is_tuser(fpga_as_is_tuser),
		.as_is_tready(fpga_as_is_tready),


		.serial_txd(fpga_serial_txd),
		.serial_rxd(soc_serial_txd),


		.is_as_tdata(fpga_is_as_tdata),
    `ifdef USER_PROJECT_SIDEBAND_SUPPORT
        .is_as_tupsb  (fpga_is_as_tupsb),
    `endif
		.is_as_tstrb(fpga_is_as_tstrb),
		.is_as_tkeep(fpga_is_as_tkeep),
		.is_as_tlast(fpga_is_as_tlast),
		.is_as_tid(fpga_is_as_tid),
		.is_as_tvalid(fpga_is_as_tvalid),
		.is_as_tuser(fpga_is_as_tuser),
		.is_as_tready(fpga_is_as_tready)
	);
    
    task fpga_apply_reset;
		input real delta1;		// for POR De-Assert
		input real delta2;		// for reset De-Assert
		begin
			#(40);
			$display($time, "=> fpga POR Assert"); 
			fpga_resetb = 0;
			$display($time, "=> fpga reset Assert"); 
			fpga_rst = 1;
			#(delta1);

			$display($time, "=> fpga POR De-Assert"); 
			fpga_resetb = 1;

			#(delta2);
			$display($time, "=> fpga reset De-Assert"); 
			fpga_rst = 0;
		end
	endtask

    task init_fpga_as;
    begin
		#40;

		fpga_as_to_is_init();
				
		//soc_cc_is_enable=1;
		fpga_cc_is_enable=1;

		#400;
		$display($time, "=> wait uut.mprj.u_fsic.U_IO_SERDES0.rxen");
        wait(uut.mprj.u_fsic.U_IO_SERDES0.rxen);
		$display($time, "=> detect uut.mprj.u_fsic.U_IO_SERDES0.rxen=1");

		fpga_cfg_write(0,1,1,0);
		$display($time, "=> fpga rxen_ctl=1");
        
		repeat(4) @(posedge fpga_coreclk);
		fork 
			//soc_is_cfg_write(0, 4'b0001, 3);				//ioserdes txen
			fpga_cfg_write(0,3,1,0);
		join
		//$display($time, "=> soc txen_ctl=1");
		$display($time, "=> fpga txen_ctl=1");

		#200;
		fpga_as_is_tdata = 32'h5a5a5a5a;
    `ifdef USER_PROJECT_SIDEBAND_SUPPORT
        fpga_as_is_tupsb = 5'h00;
    `endif
		#40;
		#200;
    end
	endtask

    // fpga_as_to_is_init

    // fpga_axilite_write -> fpga write to soc cfg
    // fpga_axilite_read_req -> fpga read soc cfg
    // fpga_axis_req -> write to user project
    // fpga_cfg_write -> write to fpga fsic cfg(tx enable control and rx enable control)
    // wait_and_check_soc_to_fpga_mailbox_write_event

    // downstream: to caravel
    // updtream: to fpga 
    initial begin		//get upstream soc_to_fpga_axilite_read_completion
		while (1) begin
			@(posedge fpga_coreclk);
			if (fpga_is_as_tvalid == 1 && fpga_is_as_tid == TID_UP_AA && fpga_is_as_tuser == TUSER_AXILITE_READ_CPL) begin
				$display($time, "=> get soc_to_fpga_axilite_read_cpl_captured be : soc_to_fpga_axilite_read_cpl_captured =%x, fpga_is_as_tdata=%x", soc_to_fpga_axilite_read_cpl_captured, fpga_is_as_tdata);
				soc_to_fpga_axilite_read_cpl_captured = fpga_is_as_tdata ;		//use block assignment
				$display($time, "=> get soc_to_fpga_axilite_read_cpl_captured af : soc_to_fpga_axilite_read_cpl_captured =%x, fpga_is_as_tdata=%x", soc_to_fpga_axilite_read_cpl_captured, fpga_is_as_tdata);
				->> soc_to_fpga_axilite_read_cpl_event;
				$display($time, "=> soc_to_fpga_axilite_read_cpl_captured : send soc_to_fpga_axilite_read_cpl_event");
			end	
		end
	end

    initial begin		//when soc cfg write to AA, then AA in soc generate soc_to_fpga_mailbox_write, 
        stream_data_addr_or_data = 0;
		while (1) begin
			@(posedge fpga_coreclk);
			//New AA version, all stream data with last = 1.  
            if (fpga_is_as_tvalid == 1 && fpga_is_as_tid == TID_UP_AA && fpga_is_as_tuser == TUSER_AXILITE_WRITE && fpga_is_as_tlast == 1) begin
                if(stream_data_addr_or_data == 1'b0) begin
                    //Address
                    $display($time, "=> get soc_to_fpga_mailbox_write_addr_captured be : soc_to_fpga_mailbox_write_addr_captured =%x, fpga_is_as_tdata=%x", soc_to_fpga_mailbox_write_addr_captured, fpga_is_as_tdata);
                    soc_to_fpga_mailbox_write_addr_captured = fpga_is_as_tdata ;		//use block assignment
                    $display($time, "=> get soc_to_fpga_mailbox_write_addr_captured af : soc_to_fpga_mailbox_write_addr_captured =%x, fpga_is_as_tdata=%x", soc_to_fpga_mailbox_write_addr_captured, fpga_is_as_tdata);
                    //Next should be data
                    stream_data_addr_or_data = 1; 
                end else begin
                    //Data
                    $display($time, "=> get soc_to_fpga_mailbox_write_data_captured be : soc_to_fpga_mailbox_write_data_captured =%x, fpga_is_as_tdata=%x", soc_to_fpga_mailbox_write_data_captured, fpga_is_as_tdata);
                    soc_to_fpga_mailbox_write_data_captured = fpga_is_as_tdata ;		//use block assignment
                    $display($time, "=> get soc_to_fpga_mailbox_write_data_captured af : soc_to_fpga_mailbox_write_data_captured =%x, fpga_is_as_tdata=%x", soc_to_fpga_mailbox_write_data_captured, fpga_is_as_tdata);
                    #0 -> soc_to_fpga_mailbox_write_event;
                    $display($time, "=> soc_to_fpga_mailbox_write_data_captured : send soc_to_fpga_mailbox_write_event");                    
                    //Next should be address
                    stream_data_addr_or_data = 0;
                end
			end	
		end
	end

    // -----------------------------------

    // SOC read FSIC modules configuration


    // FPGA read SOC FSIC module configuration
  endmodule