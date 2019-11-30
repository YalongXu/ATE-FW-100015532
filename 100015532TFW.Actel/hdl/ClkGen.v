`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:59:54 06/23/2009 
// Design Name: 
// Module Name:    ClkGen 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//          40MHz input
//              20MHz output
//              20MHz output
//              1MHz output
//              200kHz output
//              16kHz output
//              10kHz output
//              10Hz output
//              
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps

(* opt_mode="speed", optimize_primitives="yes" *)
`define CLK_10HZ_ADDR		4'h0
`define CLK_2KHZ_ADDR		4'h1
`define CLK_RES_ADDR		4'h2	
`define CLK_DIV_AQ_ADDR     4'h3    /* Aquasition clock divier Register (R/W) */
`define CLK_RS485_ADDR		4'h4
`define CCLK_DIV_ADDR		4'h5	/* Clock Divider for CCLK 6000 */
`define CLK_16KHZ_ADDR		4'h6
`define CLK_DIV_SD_ADDR		4'h7	/* Serial Data Clock divier Register (R/W) */
`define ENET_DCM			4'h8    
`define OPB_DCM				4'h9

module ClkGen(
	output  [31:0]  OPB_DO,
	input   [15:0]  OPB_DI,
	input   [3:0]   OPB_ADDR,
	input           OPB_RE,
	input           OPB_WE,
	input           OPB_CLK,				
	input           OPB_RST,
	input           SYSCLK,					
	
	output          CLK_10Hz,
	output          CLK_RESOLVER,
	output          CLK_AQ,
	output          CLK_DATA,
	output          CLK_CAN,	
	output          CLK_16KHz,
	output          CLK_SD,
	output          ENET_CLK,
	input           PCI_CLK2,
	output          OPBCLKUB
	
);

    localparam Clk10HzDivDflt   = 16'd5000;
    localparam ClkResDivDflt    = 16'd2000;
    localparam ClkAcqDivDflt    = 16'd100;
    localparam ClkRs485DivDflt  = 16'd20;
    localparam ClkCanDivDflt    = 16'd0;
    localparam Clk16kHzDivDflt  = 16'd1000;
    localparam ClkSerDatDivDflt = 16'd1;
		
	reg [15:0] Clk10HzDiv;
	reg [15:0] SysToResClkdiv;
	reg [15:0] aq_div;	
	reg [15:0] rs485_div;  	
	reg [15:0] cclk_div; 		
	reg [15:0] OpbTo16KHzDiv;	
	reg [15:0] sd_div;	

	// 10Hz
    wire pre10hz;
	CLOCK_DIV stat_led2_clk_pre(
		.CLK_DIV(16'd200),
		.CLK_IN(SYSCLK),
		.CLK_OUT(pre10hz),
        .RST(OPB_RST)
	);		
    
	CLOCK_DIV stat_led2_clk(
		.CLK_DIV(Clk10HzDiv),
		.CLK_IN(pre10hz),
		.CLK_OUT(CLK_10Hz),
        .RST(OPB_RST)
	);		
	
    // Resolver clock: default = 10KHz
	CLOCK_DIV res_clk(
		.CLK_DIV(SysToResClkdiv),
		.CLK_IN(SYSCLK),
		.CLK_OUT(CLK_RESOLVER),
        .RST(OPB_RST)
	);
		
    // aquasition clock: defualt = 200KHz
	CLOCK_DIV aqclkdiv(
		.CLK_DIV(aq_div),
		.CLK_IN(SYSCLK),
		.CLK_OUT(CLK_AQ),
        .RST(OPB_RST)
	);
		
    // RS485 clock: default = 1MHz
	CLOCK_DIV pci_clk_div(
		.CLK_DIV(rs485_div),
		.CLK_IN(SYSCLK),
		.CLK_OUT(CLK_DATA),
        .RST(OPB_RST)
	);
	
    // CAN ref_clk: default = 20MHz
	CLOCK_DIV cclk_clk_div(
		.CLK_DIV(cclk_div),
		.CLK_IN(SYSCLK),
		.CLK_OUT(CLK_CAN),
        .RST(OPB_RST)
	);
	
    // PCI 16KHz clk
	CLOCK_DIV pci_16KHz_div(
		.CLK_DIV(OpbTo16KHzDiv),
		.CLK_IN(OPBCLKUB),
		.CLK_OUT(CLK_16KHz),
        .RST(OPB_RST)
	);
	
    // serial data clock: = 20MHz
	CLOCK_DIV sclk(
		.CLK_DIV(sd_div),
		.CLK_IN(SYSCLK),
		.CLK_OUT(CLK_SD),
        .RST(OPB_RST)
	);
	
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `CLK_10HZ_ADDR))   ?  {16'b0, Clk10HzDiv}     : 32'bz;
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `CLK_RES_ADDR))    ?  {16'b0, SysToResClkdiv} : 32'bz;
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `CLK_DIV_AQ_ADDR)) ?  {16'b0, aq_div}         : 32'bz;
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `CLK_RS485_ADDR))  ?  {16'b0, rs485_div}      : 32'bz;
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `CCLK_DIV_ADDR))   ?  {16'b0, cclk_div}       : 32'bz;
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `CLK_16KHZ_ADDR))  ?  {16'b0, OpbTo16KHzDiv}  : 32'bz;
	assign OPB_DO = (OPB_RE && (OPB_ADDR == `CLK_DIV_SD_ADDR)) ?  {16'b0, sd_div}         : 32'bz;
	
	always@(posedge OPB_CLK or posedge OPB_RST) begin	
		if(OPB_RST) begin
			Clk10HzDiv     <= Clk10HzDivDflt;
			SysToResClkdiv <= ClkResDivDflt;
			aq_div         <= ClkAcqDivDflt;
			rs485_div      <= ClkRs485DivDflt;
			cclk_div       <= ClkCanDivDflt;
			OpbTo16KHzDiv  <= Clk16kHzDivDflt;
			sd_div         <= ClkSerDatDivDflt;
		end	
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`CLK_10HZ_ADDR:   Clk10HzDiv     <= OPB_DI;
				`CLK_RES_ADDR:    SysToResClkdiv <= OPB_DI;
				`CLK_DIV_AQ_ADDR: aq_div         <= OPB_DI;
				`CLK_RS485_ADDR:  rs485_div      <= OPB_DI;
				`CCLK_DIV_ADDR:   cclk_div       <= OPB_DI;
				`CLK_16KHZ_ADDR:  OpbTo16KHzDiv  <= OPB_DI;
				`CLK_DIV_SD_ADDR: sd_div         <= OPB_DI;
			endcase
		end
	end

	CLOCK_DIV enet_clk(
		.CLK_DIV(16'd2),
		.CLK_IN(SYSCLK),
		.CLK_OUT(ENET_CLK),
        .RST(OPB_RST)
	);

    assign OPBCLKUB = PCI_CLK2;
    
endmodule