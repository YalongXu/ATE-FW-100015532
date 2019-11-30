`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:29:52 08/21/2008 
// Design Name: 
// Module Name:    Oscillator_Counter 
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

`define CNTRLR          2'h0    /* Control Register (R/W) 0 */
`define CLKDIVR         2'h1    /* Aquasition clock divier Register (R/W) 10 */
`define COUNTR          2'h2    /* Count Register (R/W) 20 */
`define SPR					2'h3    /* Scratch Pad Register 30 */

module OSCILLATOR_COUNTER(
    output  [31:0]  OPB_DO,
    input   [31:0]  OPB_DI,
    input   [1:0]   OPB_ADDR,
    input           OPB_RE,
    input           OPB_WE,
    input           OPB_CLK,				//32MHz
    input           OPB_RST,
    input           SYSCLK,					//80MHz
	input           REF_CLK
); 
	
	//Module						
	wire        cntr_en;							// counter_en
	wire [15:0] count_data;					// counter register
	wire [2:0]  control;					// control register (control[0] = Start Measurement, control[1] = Reset Counter, control[2] = Measurement Inprogress)
	reg         resetb;
	reg         startb;
	reg         cntr_trig;
	reg  [31:0] sp;
	
	assign control[0] = startb;
	assign control[1] = resetb;
	assign control[2] = (startb || cntr_trig);
	assign cntr_en    = (cntr_trig && REF_CLK && ~resetb);

	TIMER_COUNTER counter(
		.ENABLE(cntr_en),
		.CLOCK(SYSCLK),
		.RESET(resetb),
		.DATA(count_data)
    );
		
	always@(negedge REF_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
            cntr_trig <= 1'b0;
        end
        else if(control[0])begin
			cntr_trig <= 1'b1;
		end
		else begin
			cntr_trig <= 1'b0;
		end
	end

/* Read Access */
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `CNTRLR)) ? {29'b0 , control}    : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `COUNTR)) ? {16'b0 , count_data} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SPR))    ? sp                   : 32'bz;
			
/* Write Access */
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			startb  <= 1'b0;
			resetb  <= 1'b0;
            sp      <= 32'h1f2e3d4c;
		end		
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`CNTRLR: begin
					startb <= OPB_DI[0];
					resetb <= OPB_DI[1];
				end
				`SPR: sp <= OPB_DI;
			endcase
		end
		else if(startb && cntr_trig) begin
			startb <= 1'b0;
		end
		else if(resetb && ~cntr_trig) begin
			resetb <= 1'b0;
			startb <= 1'b0;
		end	
	end
				
endmodule