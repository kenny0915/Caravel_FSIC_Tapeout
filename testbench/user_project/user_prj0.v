// This code snippet was auto generated by xls2vlog.py from source file: ./user_project_wrapper.xlsx
// User: josh
// Date: Sep-22-23



module USER_PRJ0 #( parameter pUSER_PROJECT_SIDEBAND_WIDTH   = 5,
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

assign ss_tready     = 1'b1;


reg [31:0] ss_tdata_reg;
reg ss_tlast_reg;
reg ss_tvalid_reg;

assign sm_tvalid     = ss_tvalid_reg || ss_tvalid;
assign sm_tdata      = ss_tdata_reg;
assign sm_tid        = 3'b0;
`ifdef USER_PROJECT_SIDEBAND_SUPPORT
  assign sm_tupsb      = 5'b0;
`endif
assign sm_tstrb      = ss_tstrb;
assign sm_tkeep      = ss_tkeep;
assign sm_tlast      = ss_tlast_reg;

assign low__pri_irq  = 1'b0;
assign High_pri_req  = 1'b0;
assign la_data_o     = 24'b0;

reg [7:0] cfg_reg;
reg [7:0] next_cfg_reg;



always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n) begin
        ss_tdata_reg <= 32'h0000_0000;
        ss_tlast_reg <= 0;
    end else begin
        ss_tdata_reg <= ss_tdata;
        ss_tlast_reg <= ss_tlast;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (~axis_rst_n)
        ss_tvalid_reg <= 1'b0;
    else
        ss_tvalid_reg <= ss_tvalid;
end

always @* begin
    if(awaddr==12'h00 && awvalid && wvalid) begin
        next_cfg_reg = wdata[7:0];
    end else begin
        next_cfg_reg = cfg_reg;
    end
end

always@(posedge axi_clk or negedge axi_reset_n) begin
    if (~axi_reset_n)
        cfg_reg <= 8'h00;
    else
        cfg_reg <= next_cfg_reg;        
end

assign rdata = (arvalid && rready && (araddr==12'h00)) ? (cfg_reg) : ({pDATA_WIDTH{1'b0}});


endmodule // USER_PRJ0
