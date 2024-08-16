`timescale 1ns/100fs


//
// Company:  
// Engineer: youwei
// 
// Create Date: 2024/07/20 18:36:41
// Design Name: 
// Module Name: 
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 


 
module fifo
#( parameter WIDTH = 45
)
(
    input axis_clk ,
    input axi_reset_n ,
    // write protocol
    input   w_vld,
    output  w_rdy,
    input [WIDTH-1:0] data_in,
    // read protocol
    input r_rdy,
    output  r_vld,
    output [WIDTH-1:0] data_out
    );

`define numAddr 6
`define numWords 64
`define halfwordlength 64
`define wordLength 128

wire [`numAddr-1:0] sram_addr;
wire [`wordLength-1:0] sram_din;
wire [`wordLength-1:0] sram_dout;
wire sram_rd;

reg [1:0] state, next_state;

reg r_cyc_cnt, w_cyc_cnt;
wire r_cyc_cnt_next, w_cyc_cnt_next;

reg [`numAddr-1:0] r_ptr, w_ptr;
wire [`numAddr-1:0] next_r_ptr, next_w_ptr;

wire r_cyc, w_cyc;

assign r_cyc = r_rdy & r_vld;
assign w_cyc = w_rdy & w_vld;

// FSM state = 00 empty, 01 sram empty but w_reg has value, 10 not empty, 11 full

always@(*) begin
    case(state)
        2'b00:  if (w_cyc) next_state = 2'b01;
                else next_state = 2'b00;
        2'b01:  if (w_cyc && !r_cyc) next_state = 2'b10;
                else if (r_cyc && !w_cyc) next_state = 2'b00;
                else next_state = 2'b01;
        2'b10:  if ((w_ptr == r_ptr) && r_cyc && !w_cyc)       next_state = 2'b01;
                else if ( ((r_ptr - w_ptr == 6'b1) || (w_ptr - r_ptr == `numWords - 1) ) && sram_we && !sram_rd)   next_state = 2'b11;
                else next_state = 2'b10;
        2'b11:  if (r_cyc) next_state = 2'b10;
                else next_state = 2'b11;
        default:next_state = 2'b00;
    endcase
end

always@(posedge axis_clk)
    if(~axi_reset_n)
        state <= 2'b00;
    else
        state <= next_state;

// r w cnt

assign r_cyc_cnt_next = (state != 2'b01 && r_cyc)      ? (!r_cyc_cnt) : r_cyc_cnt;
assign w_cyc_cnt_next = (!(state == 2'b01 && next_state == 2'b01) && w_cyc) ? (!w_cyc_cnt) : w_cyc_cnt;
// next_state = 2'10 or 2'b11 we can write data to sram.

always@(posedge axis_clk)
    if(~axi_reset_n) begin
        r_cyc_cnt <= 0;
        w_cyc_cnt <= 0;
    end else begin
        r_cyc_cnt <= r_cyc_cnt_next;
        w_cyc_cnt <= w_cyc_cnt_next;
    end

assign sram_we = (w_cyc_cnt == 1) && (w_cyc_cnt_next == 0);
assign sram_rd = (r_cyc_cnt == 0) && (r_cyc_cnt_next == 1);

// rw pointer
assign next_w_ptr = (sram_we) ? w_ptr + 6'b1 : w_ptr;
assign next_r_ptr = (sram_rd) ? r_ptr + 6'b1 : r_ptr;

always@(posedge axis_clk)
    if(~axi_reset_n) begin
        w_ptr <= 6'b0;
        r_ptr <= 6'b0;
    end else begin
        w_ptr <= next_w_ptr;
        r_ptr <= next_r_ptr;
    end

// latch
reg [`halfwordlength-1:0] r_reg;
reg [`halfwordlength-1:0] w_reg;

always@(posedge axis_clk)
    if(~axi_reset_n)
        w_reg <= {`halfwordlength{1'b0}};
    else if (w_cyc)
        w_reg <= {{`halfwordlength - WIDTH{1'b0}} , data_in};
    else
        w_reg <= w_reg;

always@(posedge axis_clk)
    if(~axi_reset_n)
        r_reg <= {`halfwordlength{1'b0}};
    else if (r_cyc)
        r_reg <= {{`halfwordlength - WIDTH{1'b0}},sram_dout[WIDTH - 1:0]};
    else
        r_reg <= r_reg;

// data out mux
wire [WIDTH - 1:0] data_out_mux;

assign data_out_mux = (state == 2'b01 && next_state == 2'b01 || next_state == 2'b00) ? w_reg : ((r_cyc_cnt == 1) ? r_reg : sram_dout[`halfwordlength + WIDTH - 1: `halfwordlength]);

// sram

assign sram_addr = (sram_we) ? w_ptr : r_ptr;
assign sram_din = {w_reg, {`halfwordlength - WIDTH{1'b0}}, data_in};
assign data_out = data_out_mux;

// axis protocal
assign w_rdy = (state != 2'b11);
assign r_vld = (state != 2'b00);


SRAM1RW64x128 SRAM1RW64x64(
    .CE(axis_clk),
    .WEB(~sram_we),// WEB =1 is read WEB=0 is w_vldite
    .OEB(1'b0),
    .CSB(1'b0),
    .A(sram_addr),
    .I(sram_din),
    .O(sram_dout)
);


endmodule
