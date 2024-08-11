
/*--------------------------------------------------------------------------------*/
/* MIT License                                                                    */
/*                                                                                */
/* Copyright (c) 2023 VIA NEXT Technologies, Inc - <Hurry Lin>                    */
/*                                                                                */
/* Permission is hereby granted, free of charge, to any person obtaining a copy   */
/* of this software and associated documentation files (the "Software"), to deal  */
/* in the Software without restriction, including without limitation the rights   */
/* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      */
/* copies of the Software, and to permit persons to whom the Software is          */
/* furnished to do so, subject to the following conditions:                       */
/*                                                                                */
/* The above copyright notice and this permission notice shall be included in all */
/* copies or substantial portions of the Software.                                */
/*                                                                                */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     */
/* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       */
/* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    */
/* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         */
/* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  */
/* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  */
/* SOFTWARE.                                                                      */
/*--------------------------------------------------------------------------------*/
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/06/17 22:39:23
// Design Name: 
// Module Name: arbiter_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module fifo_tb();
parameter   VALID_WS_LEN = 2;

localparam DATA_WIDTH=45;
localparam Simulation_time=20000;

reg o_clk, o_rst_n;
reg [DATA_WIDTH-1:0] data_in;
reg [DATA_WIDTH-1:0] r_data[1000:0];
reg [DATA_WIDTH-1:0] w_data[1000:0];
wire [DATA_WIDTH-1:0] data_out;
reg w_vld,r_rdy;
wire w_rdy,r_vld;
reg [1:0]start_test;
reg [DATA_WIDTH-1:0]i;
reg [DATA_WIDTH-1:0] j;
reg [31:0]cnt;



initial
begin
    i=0;
    j=0;
    w_vld=0;
    r_rdy=0;
    data_in=0;
	cnt=1;
    start_test = 0;    
	o_clk = 0;
	o_rst_n = 1'b0;
	#150 o_rst_n = 1;
end
reg [DATA_WIDTH-1:0] seed=1;

task test0;   //wirte to full test full 
    if(start_test==1) begin
		w_vld <=1;
        r_rdy<= 0;
    end
    else  if(start_test==2) begin 
        w_vld<= 1;   
        r_rdy<= 1;   
    end
    else if(start_test==3)begin
        w_vld<= 0;   
        r_rdy<= 1;   
    end
endtask


task test1;   //wirte to full test full 
    if(start_test==1) begin
		w_vld <=1;
        r_rdy<= 0;
    end
    else  if(start_test==2) begin 
        w_vld<= 0;   
        r_rdy<= 1;   
    end
endtask

task test2;  // always can read an write
    if(start_test==1) begin
		w_vld <=1;
        r_rdy<= 1;
    end
    else  if(start_test==2) begin 
        w_vld<= 0;   
        r_rdy<= 1;   
    end
endtask

task test3;// random cycle
    if(start_test==1) begin
		w_vld <= $random(seed)%2;
        r_rdy <= $random(seed)%2;
    end
    else  if(start_test==2) begin 
        w_vld<= 0;   
        r_rdy<= 1;   
    end
endtask

always @(posedge o_clk) begin
	begin
        test0;
        //test1;
        //test2;
        //test3;
    end  
end


// read protocol
always @(posedge o_clk) begin
	if(r_rdy && r_vld) begin
        r_data[i]<=data_out;
        i<=i+1;           
	end
end

// write protocol
always @  (posedge o_clk) begin
    if(w_vld && w_rdy) begin
        w_data[j]<=data_in;
        j=j+1;
        data_in<=data_in+1;           
	end
end
reg [15:0]index;
reg error;
initial
begin
    error=0;
    #200
    start_test = 1;  


    #(Simulation_time)
    start_test = 2;
    #(Simulation_time)
    start_test = 3;
    wait (i==j);
    for (index = 0; index < i; index=index+1) begin
        $display("Write data at iteration %d: %d", index + 1, w_data[index]);
        $display("Read  data at iteration %d: %d", index + 1, r_data[index]);
        if(w_data[index]!=r_data[index])begin
            error=1;
        end
    end
    $display("======================Result=========================");
    if(error)  $display("fail");
    else $display("succeed");
    $display("======================End============================");
    $finish;
end

fifo #(.WIDTH(DATA_WIDTH)) fifo
    (
        .axis_clk(o_clk),
        .axi_reset_n(o_rst_n),
        .w_vld(w_vld),
        .w_rdy(w_rdy),
        .data_in(data_in),
        .r_vld(r_vld),
        .r_rdy(r_rdy),
        .data_out(data_out)
    ); 
always	#50 o_clk = ~o_clk;
initial begin
    $dumpfile("fifo_tb.vcd");
    $dumpvars(0,fifo_tb);
end


endmodule
