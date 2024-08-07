// This code snippet was auto generated by xls2vlog.py from source file: ./user_project_wrapper.xlsx
// User: josh
// Date: Sep-22-23



module USER_PRJ1 #( parameter pUSER_PROJECT_SIDEBAND_WIDTH   = 5,
          parameter pADDR_WIDTH   = 12,
                   parameter pDATA_WIDTH   = 32
                 )
(
  output wire                        awready,
  output wire                        arready,
  output wire                        wready,
  output wire                        rvalid,
  output wire  [(pDATA_WIDTH-1) : 0] rdata,
  input  wire                        awvalid,
  input  wire                [11: 0] awaddr,
  input  wire                        arvalid,
  input  wire                [11: 0] araddr,
  input  wire                        wvalid,
  input  wire                 [3: 0] wstrb,
  input  wire  [(pDATA_WIDTH-1) : 0] wdata,
  input  wire                        rready,
  input  wire                        ss_tvalid,
  input  wire  [(pDATA_WIDTH-1) : 0] ss_tdata,
  input  wire                 [1: 0] ss_tuser,
    `ifdef USER_PROJECT_SIDEBAND_SUPPORT
  input  wire                 [pUSER_PROJECT_SIDEBAND_WIDTH-1: 0] ss_tupsb,
  `endif
  input  wire                 [3: 0] ss_tstrb,
  input  wire                 [3: 0] ss_tkeep,
  input  wire                        ss_tlast,
  input  wire                        sm_tready,
  output wire                        ss_tready,
  output wire                        sm_tvalid,
  output wire  [(pDATA_WIDTH-1) : 0] sm_tdata,
  output wire                 [2: 0] sm_tid,
  `ifdef USER_PROJECT_SIDEBAND_SUPPORT
  output  wire                 [pUSER_PROJECT_SIDEBAND_WIDTH-1: 0] sm_tupsb,
  `endif
  output wire                 [3: 0] sm_tstrb,
  output wire                 [3: 0] sm_tkeep,
  output wire                        sm_tlast,
  output wire                        low__pri_irq,
  output wire                        High_pri_req,
  output wire                [23: 0] la_data_o,
  input  wire                        axi_clk,
  input  wire                        axis_clk,
  input  wire                        axi_reset_n,
  input  wire                        axis_rst_n,
  input  wire                        user_clock2,
  input  wire                        uck2_rst_n
);


assign awready       = 1'b1;
assign wready        = 1'b1;

assign arready       = 1'b1;
assign rvalid        = 1'b1;

//assign ss_tready     = 1'b1;


reg [31:0] ss_tdata_reg;
reg ss_tvalid_reg;

assign sm_tvalid     = 0;
assign sm_tdata      = 0;
assign sm_tid        = 3'b0;
`ifdef USER_PROJECT_SIDEBAND_SUPPORT
  assign sm_tupsb      = 5'b0;
`endif
assign sm_tstrb      = ss_tstrb;
assign sm_tkeep      = ss_tkeep;
assign sm_tlast      = ss_tlast;

assign low__pri_irq  = 1'b0;
assign High_pri_req  = 1'b0;
assign la_data_o     = 24'b0;

reg [7:0] cfg_reg;
reg [7:0] next_cfg_reg;

reg [4:0] cnt;
wire [4:0] cnt_next;

assign cnt_next = (cnt == 5'd25) ? cnt : cnt + 1'b1;

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n || cfg_reg[0])
        cnt <= 1'b0;
    else
        cnt <= cnt_next;
end

assign ss_tready = (cnt == 5'd25);

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n)
        ss_tdata_reg <= 32'h0000_0000;
    else
        ss_tdata_reg <= ss_tdata;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n)
        ss_tvalid_reg <= 1'b0;
    else
        ss_tvalid_reg <= ss_tvalid;
end

wire done;

assign done = (ss_tdata == 32'd39);

always @* begin
    case(cfg_reg)
        8'd0:
            if(awaddr==12'h00 && awvalid && wvalid) next_cfg_reg = wdata[7:0];
            else next_cfg_reg = cfg_reg;
        8'd1:
            next_cfg_reg = 8'd2;
        8'd2:
            if(done) next_cfg_reg = 8'd4;
            else next_cfg_reg = cfg_reg;
        8'd4:
            next_cfg_reg = cfg_reg;
        default: next_cfg_reg = 8'd0;
    endcase
end

always@(posedge axi_clk or negedge axi_reset_n) begin
    if (~axi_reset_n)
        cfg_reg <= 8'h00;
    else
        cfg_reg <= next_cfg_reg;        
end

assign rdata = (arvalid && rready && (araddr==12'h00)) ? (cfg_reg) : ({pDATA_WIDTH{1'b0}});


endmodule // USER_PRJ1
