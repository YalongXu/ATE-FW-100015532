`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    11:43:08 07/17/2008 
// Design Name: 
// Module Name:    CLK_DIV 
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
module CLOCK_DIV(
	CLK_DIV,
	CLK_IN,
	CLK_OUT,
    RST
);

	input   [15:0]  CLK_DIV;		// # of clk_in cycles to half clk_out cycle
	input           CLK_IN;
    input           RST;
	output          CLK_OUT;


	reg             clkout;
	reg     [15:0]  cntr;
	
	always@(posedge CLK_IN or posedge RST) begin
        if(RST) begin
            cntr    <= 1;
            clkout  <= 1'b0;
        end          
		else if(CLK_DIV > 0) begin
            if(cntr >= CLK_DIV) begin
                cntr    <= 1;
                clkout  <= ~clkout;
            end
            else
                cntr    <= cntr + 1;
        end
        else begin
            cntr    <= 1;
            clkout  <= 1'b0;
        end
	end
	
	assign CLK_OUT = (CLK_DIV > 0) ? clkout : CLK_IN;


endmodule
