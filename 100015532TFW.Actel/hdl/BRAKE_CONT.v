`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:13:40 09/15/2008 
// Design Name: 
// Module Name:    BRIDGE_CONT 
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

`define PWM_F_CLK_DIV_ADDR		4'h2 //8  clock divider is # of sysclk cycles in a 1/3 pulse cycle
`define SAMP_TIME_SET_ADDR		4'h3 //C
`define SAMP_TRIG_ADDR			4'h4 //10
`define FAULT_ADDR				4'h5 //14
`define ENABLE_ADDR				4'h6 //18

`define POL_ADDR				4'h7 //1c
`define CYCLE_ADDR				4'h8 //20
`define OVERI_ADDR				4'h9 //24


module BRAKE_CONT(
    output  [31:0]  OPB_DO,
    input   [31:0]  OPB_DI,
    input   [3:0]   OPB_ADDR,
    input           OPB_RE,
    input           OPB_WE,
    input           OPB_CLK,    //32MHz
    input           OPB_RST,
    input           SYSCLK,	    //80MHz
	 
	output          PD_PWM,
	output          PD_ENABLE,
	input           OVER_CURR,
    input           FAULTb  
);

	reg [15:0]  clk_divider;    // frequency control	# of sysclk cycles in a 1/3 pulse cycle
	reg [15:0]  cntr_freq;		// frequency counter
	reg         StrobeA;		// Leg A strobe

	reg [31:0]  CycleCount;     // duty cycle control
	reg [31:0]  cntr_dutyA;		// duty cycle counter A

	reg         polarity;		// polarity control
	reg         PulseA;							
	
	reg [31:0]  sample_time_set;
	reg         sample_trig;

	
	reg         over_curr_buf;
	reg         faultB_buf;
	reg         pdenable;
		
	wire [15:0] full_clk_div = (clk_divider+clk_divider+clk_divider);
	
	assign PD_ENABLE = pdenable;
	
	assign PD_PWM = (polarity) ? PulseA : !PulseA;

///////////////////////////////////////////////////////////////////////////
// Frequency Strobes 	
	always@(negedge SYSCLK or posedge OPB_RST) begin
        if(OPB_RST) 
			cntr_freq <= 0;
		else if(cntr_freq >= full_clk_div) 
			cntr_freq <= 0;
		else 
			cntr_freq <= cntr_freq + 1;			
	end
	
	always@(posedge SYSCLK or posedge OPB_RST) begin
        if(OPB_RST) begin
            StrobeA <= 1'b0;
        end
        else begin
    		case(cntr_freq)
    			full_clk_div: StrobeA <= 1'b1;
    			default:      StrobeA <= 1'b0;
    		endcase
        end
	end

/////////////////////////////////////////////////////////////////////////
// Pulse control
	always@(negedge SYSCLK or posedge OPB_RST) begin
        if(OPB_RST)
			cntr_dutyA <= 32'h0;
        else if(cntr_dutyA < CycleCount)
		    cntr_dutyA <= cntr_dutyA +1;
		else if(StrobeA)  
			cntr_dutyA <= 32'h0;
	end
	
	always@(posedge SYSCLK or posedge OPB_RST) begin
        if(OPB_RST)
			PulseA <= 1'b1;
		else if(cntr_dutyA >= CycleCount)
			PulseA <= 1'b0;
		else 
			PulseA <= 1'b1;
	end

//Fault Monitoring 32'h4b0;
	reg  [15:0] over_i_set;
	reg         over_i_rst;
	reg         fault_reset;
	wire [15:0] over_i_count;
	
	TIMER_COUNTER #(
		.DATA_WIDTH(16)
	) over_i_counter(
		.ENABLE(1'b1),				// always enabled
		.CLOCK(SYSCLK),
		.RESET(over_i_rst),
		.DATA(over_i_count)
    );
		
	always@(posedge SYSCLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			over_curr_buf <= 1'b0;
			over_i_rst    <= 1'b1;
		end
		else if(fault_reset) begin
			over_curr_buf <= 1'b0;
			over_i_rst    <= 1'b1;
		end
		else if(OVER_CURR) begin
			over_i_rst <= 1'b0;
			if(over_i_count > over_i_set)
				over_curr_buf <= 1'b1;
		end 
		else begin
			over_i_rst <= 1'b1;
		end
	end

	always@(posedge SYSCLK or posedge OPB_RST) begin
		if(OPB_RST) 
			faultB_buf <= 1'b1;
		else if(!FAULTb) 
			faultB_buf <= 1'b0;
	end

//////////////////////////////////////////////////////////////////////////////	
/* Read Access */
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `PWM_F_CLK_DIV_ADDR)) ? {16'b0 , clk_divider}               : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SAMP_TIME_SET_ADDR)) ? {16'b0 , sample_time_set}           : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SAMP_TRIG_ADDR))     ? {31'b0 , sample_trig}               : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `FAULT_ADDR))         ? {30'b0 , over_curr_buf, faultB_buf} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `ENABLE_ADDR))        ? {31'b0 , pdenable}                  : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `POL_ADDR))           ? {31'b0 , polarity}                  : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `CYCLE_ADDR))         ? CycleCount                          : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `OVERI_ADDR))         ? {16'b0 , over_i_set}                : 32'bz;

/* Write Access */
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
//			clk_divider     <= 16'ha6a;	
			clk_divider     <= 16'h535;	
			sample_trig     <= 1'b0;
//			sample_time_set <= 32'h4b0;
			sample_time_set <= 32'h258;
			pdenable        <= 1'b0;
			polarity        <= 1'b1;
//			CycleCount      <= 32'h4b0;
			CycleCount      <= 32'h258;
//			over_i_set      <= 16'h258;
			over_i_set      <= 16'h12c;
			fault_reset     <= 1'b0;
		end		
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`PWM_F_CLK_DIV_ADDR: clk_divider     <= OPB_DI[15:0];
				`SAMP_TIME_SET_ADDR: sample_time_set <= OPB_DI;
				`SAMP_TRIG_ADDR:     sample_trig     <= OPB_DI[0];
				`ENABLE_ADDR:        pdenable        <= OPB_DI[0];
				`POL_ADDR:           polarity        <= OPB_DI[0];
				`CYCLE_ADDR:         CycleCount      <= OPB_DI;
				`OVERI_ADDR:         over_i_set      <= OPB_DI[15:0];
				`FAULT_ADDR:         fault_reset     <= OPB_DI[0];
			endcase
		end
	end
endmodule