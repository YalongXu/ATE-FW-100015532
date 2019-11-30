`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:13:11 08/25/2008 
// Design Name: 
// Module Name:    CAN_SJA1000 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`define S0				2'b00
`define S1				2'b01
`define S2				2'b10
`define S3				2'b11
`define S00				4'b0000     /* Ready State */
`define S01				4'b0001     /* Address Latch State */
`define S11				4'b0101	    /* Wait State */
`define S12				4'b0110	    /* Write State */
`define S13				4'b0111	    /* Read State */

`define CAN1			16'h0100    /* Base Address of Controller 1 1000 */
`define CAN2			16'h0200	/* Base Address of Controller 2 2000 */
`define CAN3			16'h0400	/* Base Address of Controller 3 4000 */
`define CONTROLR		16'h0500	/* Control Register 5000 */

module CAN_SJA1000(
    output  [31:0]  OPB_DO,
    input   [7:0]   OPB_DI,
    input   [15:0]  OPB_ADDR,
    input           OPB_RE,
    input           OPB_WE,
    input           OPB_CLK,
    input           OPB_RST,

    output          CAN_RST,
    input           CAN_INT1,
    input           CAN_INT2,
    inout   [7:0]   CAN_AD,
    output          CAN_ALE,
    output          CAN_RD,
    output          CAN_WR,
    output          CAN_CS1,
    output          CAN_CS2,
    output          CAN_CS3,
    output          CAN_BUF_DIR1,
    output          INT1,
    output          INT2
);	
	
	 reg [1:0]  state0;
	 reg        state1;
	 reg [7:0]  can_di;
	 reg [2:0]  control;
	 
	 wire [3:0] state          = {state1 , state0};	 
	 wire       can_wr_en      = (OPB_WE && (state == `S12)) ? 1'b1 : 1'b0;
	 wire       can_rd_en      = (OPB_RE && (state == `S13)) ? 1'b1 : 1'b0;	 
	 
	 wire       control_access = (OPB_ADDR == `CONTROLR);
	 
	 assign INT1 = CAN_INT1;
	 assign INT2 = CAN_INT2;

	 assign CAN_RST = control[2];								//control set to 0x8
	 assign CAN_ALE = (state == `S01) ? 1'b1 : 1'b0;
	 assign CAN_RD  = ~(can_rd_en & OPB_RE);
	 assign CAN_WR  = ~(can_wr_en & OPB_WE);
	 assign CAN_CS1 = (CAN_RD & CAN_WR) | ~OPB_ADDR[8];
	 assign CAN_CS2 = (CAN_RD & CAN_WR) | ~OPB_ADDR[9];
	 assign CAN_CS3 = (CAN_RD & CAN_WR) | ~OPB_ADDR[10];
	 assign CAN_BUF_DIR1 = (state == `S13);
	 
	 ///// Write Access
	 assign CAN_AD = (state == `S01 || state == `S11) ? OPB_ADDR[7:0] : 8'bz;
	 assign CAN_AD = (can_wr_en)                      ? can_di        : 8'bz;
	 
	 ///// Read Access
	 assign OPB_DO = (can_rd_en)                         ? {24'b0 , CAN_AD}  : 32'bz;
	 assign OPB_DO = ((OPB_ADDR == `CONTROLR) && OPB_RE) ? {28'b0 , control} : 32'bz;


	 ///// State Machine
	 always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin				
			state0 <= `S0;
			can_di <= 8'b0;
		end		
		else if((OPB_RE | OPB_WE) & ~control_access) begin
			case(state0)
    			`S0: state0 <= `S1;
    			`S1: begin
    				state0 <= (OPB_WE) ? `S2 : `S3;
    				can_di <= OPB_DI[7:0];
    			 end
			endcase
		end
		else begin
			state0 <= `S0;
			can_di <= 8'b0;
		end
	 end

	 always@(posedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			state1 <= `S0;
		end
		else if((OPB_RE | OPB_WE) & ~control_access) begin
			case(state1)
				`S0: state1 <= `S1;
			endcase
		end
		else begin
			state1 <= `S0;
		end
	end

	//Control Register Write Access
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			control <= 3'b0;
		end	
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`CONTROLR: control[2] <= OPB_DI[3];
			endcase
		end
		else begin
			control[0] <= CAN_INT1;
			control[1] <= CAN_INT2;
		end
	end

endmodule