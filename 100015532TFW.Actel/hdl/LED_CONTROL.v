`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:07:34 06/24/2009 
// Design Name: 
// Module Name:    LED_CONTROL 
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
module LED_CONTROL(
	input           CLK_10HZ,
	output          STAT_LED1,
	output          STAT_LED2,
	output          STAT_LED3,
	output          HEARTBEAT,
	output          TX_LED1,
	output          TX_LED2,
	output          RX_LED1,
	output          RX_LED2,
	output  [7:0]   SP_LED,
    input           RST
);

	reg     [3:0]   counter;
	reg     [7:0]   spLedReg;
	reg     [2:0]   statLedReg;
	
    assign TX_LED1      = CLK_10HZ;
	assign RX_LED1      = ~CLK_10HZ;
	assign TX_LED2      = CLK_10HZ;
	assign RX_LED2      = ~CLK_10HZ;
	assign HEARTBEAT    = CLK_10HZ;
	assign SP_LED       = spLedReg;
	assign STAT_LED1    = statLedReg[0];
	assign STAT_LED2    = statLedReg[1];
	assign STAT_LED3    = statLedReg[2];
	
	always@(posedge CLK_10HZ or posedge RST) begin
        if(RST) begin
            spLedReg    <= 8'hfe;
            statLedReg  <= 3'h6;
        end
        else begin
    		statLedReg <= {statLedReg[1:0], statLedReg[2]};

    		if(counter[3])
    			spLedReg <= {spLedReg[0], spLedReg[7:1]};
    		else
    			spLedReg <= {spLedReg[6:0], spLedReg[7]};
        end
	end
	
	always@(negedge CLK_10HZ or posedge RST) begin
        if(RST)
            counter <= 1'b1;
		else if(spLedReg[0] == 1'b0)
			counter <= 4'h1;
		else
			counter <= counter + 1;
	end

endmodule