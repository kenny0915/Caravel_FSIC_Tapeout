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


// normal fifo

reg [1:0] state;
reg [1:0] next_state;

reg [`numAddr-1:0] r_ptr, next_r_ptr;
reg [`numAddr-1:0] w_ptr, next_w_ptr;

wire r_cyc;
wire w_cyc;

assign r_cyc = r_rdy & r_vld;
assign w_cyc = w_rdy & w_vld;

// FSM state = {full, empty}
always@(*) begin
    case(state)
        2'b01: if (sram_we) next_state = 2'b00;
                else next_state = 2'b01;
        2'b00: if ( ( (w_ptr - r_ptr == 6'b1) || (r_ptr - w_ptr == `numWords - 1) ) && sram_rd && !sram_we)       next_state = 2'b01;
                else if ( ((r_ptr - w_ptr == 6'b1) || (w_ptr - r_ptr == `numWords - 1) ) && sram_we && !sram_rd)   next_state = 2'b10;
                else next_state = 2'b00;
        2'b10: if (sram_rd) next_state = 2'b00;
                else next_state = 2'b10;
        default: next_state = 2'b01;

    endcase
end

always@(posedge axis_clk)
    if(~axi_reset_n)
        state <= 2'b01;
    else
        state <= next_state;

// rw pointer
always @(*) begin
    if(sram_we) next_w_ptr = w_ptr + 6'b1;
    else        next_w_ptr = w_ptr;
end

always@(posedge axis_clk)
    if(~axi_reset_n)
        w_ptr <= 6'b0;
    else
        w_ptr <= next_w_ptr;


always @(*) begin
    if (state[0])   next_r_ptr = r_ptr;
    else begin
        if(sram_rd) next_r_ptr = r_ptr + 6'd1;
        else        next_r_ptr = r_ptr;
    end    
end

always@(posedge axis_clk)
    if(~axi_reset_n)
        r_ptr <= 6'b0;
    else
        r_ptr <= next_r_ptr;

// latch
reg [`halfwordlength-1:0] r_reg;
reg [`halfwordlength-1:0] w_reg;

always@(posedge axis_clk)
    if(~axi_reset_n)
        w_reg <= {`halfwordlength{1'b0}};
    else
        w_reg <= {{`halfwordlength - WIDTH{1'b0}} , data_in};

always@(posedge axis_clk)
    if(~axi_reset_n)
        r_reg <= {`halfwordlength{1'b0}};
    else
        r_reg <= {{`halfwordlength - WIDTH{1'b0}},sram_dout[WIDTH - 1:0]};

// sram we
reg w_cyc_cnt;
reg w_cyc_cnt_next;

always @(*) begin
    case (w_cyc_cnt)
        0: 
            if (w_cyc)  w_cyc_cnt_next = 1;
            else        w_cyc_cnt_next = 0;
        1: 
            if (w_cyc)  w_cyc_cnt_next = 0;
            else        w_cyc_cnt_next = 1;
        default: 
                        w_cyc_cnt_next = 0;
    endcase 
end

always@(posedge axis_clk)
    if(~axi_reset_n)
        w_cyc_cnt <= 0;
    else
        w_cyc_cnt <= w_cyc_cnt_next;

// sram re
reg r_cyc_cnt;
reg r_cyc_cnt_next;

always @(*) begin
    case (r_cyc_cnt)
        0: 
            if (r_cyc)  r_cyc_cnt_next = 1;
            else        r_cyc_cnt_next = 0;
        1: 
            if (r_cyc)  r_cyc_cnt_next = 0;
            else        r_cyc_cnt_next = 1;
        default: 
                        r_cyc_cnt_next = 0;
    endcase 
end

always@(posedge axis_clk)
    if(~axi_reset_n)
        r_cyc_cnt <= 0;
    else
        r_cyc_cnt <= r_cyc_cnt_next;

// data out mux
reg [WIDTH - 1:0] data_out_mux;
reg [WIDTH - 1:0] data_out_mux_next;

always @(*) begin
    if(state[0] && (w_cyc_cnt == 1))               data_out_mux = w_reg;
    else if (r_cyc_cnt == 1)                       data_out_mux= r_reg;
    else                                           data_out_mux = sram_dout[`halfwordlength + WIDTH - 1: `halfwordlength];
end

// sram

assign sram_addr = (sram_we) ? w_ptr : r_ptr;
assign sram_we = (w_cyc_cnt == 1) && w_cyc;
assign sram_rd = ((r_cyc_cnt == 0) && r_cyc);
assign sram_din = {w_reg, {`halfwordlength - WIDTH{1'b0}}, data_in};
assign data_out = data_out_mux;

// axis protocal
assign w_rdy = ~state[1];
assign r_vld = ~state[0];

// change: finite state machine full empty

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



