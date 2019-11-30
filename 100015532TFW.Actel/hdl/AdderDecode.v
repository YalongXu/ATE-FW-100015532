`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:41:07 06/23/2009 
// Design Name: 
// Module Name:    AdderDecode 
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

`define SP1_ADDR		24'h010000		/* Scratch Pad 1 address */
`define SP2_ADDR		24'h020000		/* Scratch Pad 2 address */
`define DO_ADDR			24'h060000		/* Digital Outputs Address */
`define LPMUX_ADDR		24'h060004		/* LP MUX ADDR */
`define ADSEL_ADDR		24'h060008		/* AD_SEL ADDR */
`define DI_ADDR			24'h050000		/* Digital Inputs Address */
`define P_SW_ADDR		24'h051000		/* Position Switch Inputs */
`define RES_OUT_ADDR	24'h052000		/* Resolver output enable */
`define CAN_ADDR		24'h400000		/* CAN Controller address */
`define CAN_SIZE		24'h006010		/* CAN Controller size */
`define COUNTER_ADDR	24'h000000		/* COUNTER address */
`define COUNTER_SIZE	24'h000040		/* COUNTER size */
`define AD1_ADDR		24'h070000		/* Analog to Digital converter 1 */
`define AD2_ADDR		24'h080000		/* Analog to Digital converter 2 */
`define AD_SIZE			24'h006000		/* Analog to Digital converter size */
`define RS485_ADDR		24'h090000		/* RS485 test module address */
`define RS485_SIZE		24'h000038		/* RS485 test module size */
`define BRG1_ADDR		24'h0a0000		/* BRG1 module address */
`define BRG2_ADDR		24'h0a1000		/* BRG2 module address */
`define BRG3_ADDR		24'h0a2000		/* BRG3 module address */
`define BRG4_ADDR		24'h0a3000		/* BRG4 module address */
`define BRG5_ADDR		24'h0a4000		/* BRG5 module address */
`define BRK1_ADDR		24'h0a5000		/* BRK1 module address */
`define BRK2_ADDR		24'h0a6000		/* BRK2 test module address */
`define BRG_SIZE		24'h000028		/* BRG module size */
`define ILIM_DAC_ADDR	24'h0b0000		/* ILIM_DAC module address */
`define ILIM_DAC_SIZE	24'h00000e		/* ILIM_DAC module size */
`define MEL_ADDR		24'h0c0000		/* MEL module address */
`define MEL_SIZE		24'h000048		/* MEL module size */
`define FOPT_ADDR		24'h0d0000		/* FIBER OPTIC module address */
`define FOPT_SIZE		24'h000c00		/* FIBER OPTIC module size */
`define CLOCK_ADDR		24'h011000		/* CLOCK module address */
`define CLOCK_SIZE		24'h000028		/* CLOCK module size */
`define ENET_ADDR       24'h0e0000      /* Ethernet Interface Module */
`define ENET_SIZE       24'h010000

module AdderDecode(
    input   [23:0]  OPB_ADDR,
	input           OPB_RE,
	input           OPB_WE,
	output          SP1_RE,
	output          SP1_WE,
	output          SP2_RE,
	output          SP2_WE,
	output          DO_RE,
	output          DO_WE,
	output          LPMUX_RE,
	output          LPMUX_WE,
	output          DI_RE,
	output          P_SW_RE,
	output          RES_OUT_WE,
	output          CAN_RE,
    output          CAN_WE,
	output          COUNTER_RE,
	output          COUNTER_WE,
	output          AD1_RE,
	output          AD1_WE,
	output          AD2_RE,
	output          AD2_WE,
	output          RS485_RE,
	output          RS485_WE,
	output          BRG1_RE,
	output          BRG1_WE,
	output          BRG2_RE,
	output          BRG2_WE,
	output          BRG3_RE,
	output          BRG3_WE,
	output          BRG4_RE,
	output          BRG4_WE,
	output          BRG5_RE,
	output          BRG5_WE,
	output          BRK1_RE,
	output          BRK1_WE,
	output          BRK2_RE,
	output          BRK2_WE,
	output          ILIM_DAC_RE,
	output          ILIM_DAC_WE,
	output          MEL_RE,
	output          MEL_WE,
	output          FOPT_RE,
	output          FOPT_WE,
	output          CLOCK_RE,
	output          CLOCK_WE,
	output          ADSEL_RE,
	output          ADSEL_WE,
	output          ENET_RE,
	output          ENET_WE
);
	 
	assign SP1_RE       = OPB_RE & (OPB_ADDR == `SP1_ADDR);
	assign SP1_WE       = OPB_WE & (OPB_ADDR == `SP1_ADDR);

	assign SP2_RE       = OPB_RE & (OPB_ADDR == `SP2_ADDR);
	assign SP2_WE       = OPB_WE & (OPB_ADDR == `SP2_ADDR);

	assign DO_RE        = OPB_RE & (OPB_ADDR == `DO_ADDR);
	assign DO_WE        = OPB_WE & (OPB_ADDR == `DO_ADDR);

	assign LPMUX_RE     = OPB_RE & (OPB_ADDR == `LPMUX_ADDR);
	assign LPMUX_WE     = OPB_WE & (OPB_ADDR == `LPMUX_ADDR);

	assign DI_RE        = OPB_RE & (OPB_ADDR == `DI_ADDR);

	assign P_SW_RE      = OPB_RE & (OPB_ADDR == `P_SW_ADDR);

	assign RES_OUT_WE   = OPB_WE & (OPB_ADDR == `RES_OUT_ADDR);
	
	assign ADSEL_RE     = OPB_RE & (OPB_ADDR == `ADSEL_ADDR);
	assign ADSEL_WE     = OPB_WE & (OPB_ADDR == `ADSEL_ADDR);

	assign CAN_WE       = OPB_WE & (OPB_ADDR >= `CAN_ADDR) & (OPB_ADDR < (`CAN_ADDR+`CAN_SIZE));
	assign CAN_RE       = OPB_RE & (OPB_ADDR >= `CAN_ADDR) & (OPB_ADDR < (`CAN_ADDR+`CAN_SIZE));

	assign COUNTER_RE   = OPB_RE & (OPB_ADDR >= `COUNTER_ADDR) & (OPB_ADDR < (`COUNTER_ADDR+`COUNTER_SIZE));	
	assign COUNTER_WE   = OPB_WE & (OPB_ADDR >= `COUNTER_ADDR) & (OPB_ADDR < (`COUNTER_ADDR+`COUNTER_SIZE));

	assign AD1_RE       = OPB_RE & (OPB_ADDR >= `AD1_ADDR) & (OPB_ADDR < (`AD1_ADDR + `AD_SIZE));
	assign AD1_WE       = OPB_WE & (OPB_ADDR >= `AD1_ADDR) & (OPB_ADDR < (`AD1_ADDR + `AD_SIZE));

	assign AD2_RE       = OPB_RE & (OPB_ADDR >= `AD2_ADDR) & (OPB_ADDR < (`AD2_ADDR + `AD_SIZE));
	assign AD2_WE       = OPB_WE & (OPB_ADDR >= `AD2_ADDR) & (OPB_ADDR < (`AD2_ADDR + `AD_SIZE));

	assign RS485_WE     = OPB_WE & (OPB_ADDR >= `RS485_ADDR) & (OPB_ADDR < (`RS485_ADDR + `RS485_SIZE));
	assign RS485_RE     = OPB_RE & (OPB_ADDR >= `RS485_ADDR) & (OPB_ADDR < (`RS485_ADDR + `RS485_SIZE));

	assign BRG1_WE      = OPB_WE & (OPB_ADDR >= `BRG1_ADDR) & (OPB_ADDR < (`BRG1_ADDR + `BRG_SIZE));
	assign BRG1_RE      = OPB_RE & (OPB_ADDR >= `BRG1_ADDR) & (OPB_ADDR < (`BRG1_ADDR + `BRG_SIZE));

	assign BRG2_WE      = OPB_WE & (OPB_ADDR >= `BRG2_ADDR) & (OPB_ADDR < (`BRG2_ADDR + `BRG_SIZE));
	assign BRG2_RE      = OPB_RE & (OPB_ADDR >= `BRG2_ADDR) & (OPB_ADDR < (`BRG2_ADDR + `BRG_SIZE));

	assign BRG3_WE      = OPB_WE & (OPB_ADDR >= `BRG3_ADDR) & (OPB_ADDR < (`BRG3_ADDR + `BRG_SIZE));
	assign BRG3_RE      = OPB_RE & (OPB_ADDR >= `BRG3_ADDR) & (OPB_ADDR < (`BRG3_ADDR + `BRG_SIZE));

	assign BRG4_WE      = OPB_WE & (OPB_ADDR >= `BRG4_ADDR) & (OPB_ADDR < (`BRG4_ADDR + `BRG_SIZE));
	assign BRG4_RE      = OPB_RE & (OPB_ADDR >= `BRG4_ADDR) & (OPB_ADDR < (`BRG4_ADDR + `BRG_SIZE));

	assign BRG5_WE      = OPB_WE & (OPB_ADDR >= `BRG5_ADDR) & (OPB_ADDR < (`BRG5_ADDR + `BRG_SIZE));
	assign BRG5_RE      = OPB_RE & (OPB_ADDR >= `BRG5_ADDR) & (OPB_ADDR < (`BRG5_ADDR + `BRG_SIZE));

	assign BRK1_WE      = OPB_WE & (OPB_ADDR >= `BRK1_ADDR) & (OPB_ADDR < (`BRK1_ADDR + `BRG_SIZE));
	assign BRK1_RE      = OPB_RE & (OPB_ADDR >= `BRK1_ADDR) & (OPB_ADDR < (`BRK1_ADDR + `BRG_SIZE));

	assign BRK2_WE      = OPB_WE & (OPB_ADDR >= `BRK2_ADDR) & (OPB_ADDR < (`BRK2_ADDR + `BRG_SIZE));
	assign BRK2_RE      = OPB_RE & (OPB_ADDR >= `BRK2_ADDR) & (OPB_ADDR < (`BRK2_ADDR + `BRG_SIZE));

	assign ILIM_DAC_WE  = OPB_WE  & (OPB_ADDR >= `ILIM_DAC_ADDR) & (OPB_ADDR < (`ILIM_DAC_ADDR + `ILIM_DAC_SIZE));
	assign ILIM_DAC_RE  = OPB_RE & (OPB_ADDR >= `ILIM_DAC_ADDR) & (OPB_ADDR < (`ILIM_DAC_ADDR + `ILIM_DAC_SIZE));

	assign MEL_WE       = OPB_WE & (OPB_ADDR >= `MEL_ADDR) & (OPB_ADDR < (`MEL_ADDR + `MEL_SIZE));
	assign MEL_RE       = OPB_RE & (OPB_ADDR >= `MEL_ADDR) & (OPB_ADDR < (`MEL_ADDR + `MEL_SIZE));

	assign FOPT_WE      = OPB_WE & (OPB_ADDR >= `FOPT_ADDR) & (OPB_ADDR < (`FOPT_ADDR + `FOPT_SIZE));
	assign FOPT_RE      = OPB_RE & (OPB_ADDR >= `FOPT_ADDR) & (OPB_ADDR < (`FOPT_ADDR + `FOPT_SIZE));

	assign CLOCK_WE     = OPB_WE  & (OPB_ADDR >= `CLOCK_ADDR) & (OPB_ADDR < (`CLOCK_ADDR + `CLOCK_SIZE));
	assign CLOCK_RE     = OPB_RE & (OPB_ADDR >= `CLOCK_ADDR) & (OPB_ADDR < (`CLOCK_ADDR + `CLOCK_SIZE));

	assign ENET_WE      = OPB_WE & (OPB_ADDR >= `ENET_ADDR) & (OPB_ADDR < (`ENET_ADDR + `ENET_SIZE));
	assign ENET_RE      = OPB_RE & (OPB_ADDR >= `ENET_ADDR) & (OPB_ADDR < (`ENET_ADDR + `ENET_SIZE));
							
							
endmodule
