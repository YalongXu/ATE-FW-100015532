
/******************************************************************************
*
*    File Name:  md.v
*      Version:  1.1
*         Date:  January 22, 2000
*        Model:  Manchester Decoder 
*
*      Company:  Xilinx
*
*
*   Disclaimer:  THESE DESIGNS ARE PROVIDED "AS IS" WITH NO WARRANTY 
*                WHATSOEVER AND XILINX SPECIFICALLY DISCLAIMS ANY 
*                IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
*                A PARTICULAR PURPOSE, OR AGAINST INFRINGEMENT.
*
*                Copyright (c) 2000 Xilinx, Inc.
*                All rights reserved
*
******************************************************************************/
`timescale 1ps / 1ps

(* opt_mode="speed", optimize_primitives="yes" *)
module md(
    rst,
    clk16x,
    mdi,
    rdn,
    dout,
    data_ready
);

    input           rst ;
    input           clk16x ;
    input           mdi ;
    input           rdn ;
    output  [7:0]   dout ;
    output          data_ready ;
    reg             clk1x_enable ;
    reg             mdi1 ;
    reg             mdi2 ;
    reg     [7:0]   dout ;
    reg     [3:0]   no_bits_rcvd ;
    reg     [3:0]   clkdiv ;
    reg             data_ready ;
    wire            clk1x ;
    reg             nrz ;
    wire            sample ;
    reg     [7:0]   rsr ;

    // Generate 2 FF register to accept serial Manchester data in

    always @(posedge clk16x or posedge rst) 
    begin
        if (rst)
        begin
            mdi1 <= 1'b0 ;
            mdi2 <= 1'b0 ;
        end
        else
        begin
            mdi2 <= mdi1 ;
            mdi1 <= mdi ;
        end
    end

    // Enable the 1x clock when there is an edge on mdi 

    always @(posedge clk16x or posedge rst)
    begin
        if (rst)
            clk1x_enable <= 1'b0 ;
        else if (mdi1 ^ mdi2)
            clk1x_enable <= 1'b1 ;
        else if (!mdi1 && !mdi2 && no_bits_rcvd == 4'b1010)
            clk1x_enable <= 1'b0 ;
    end

    // Generate center sample at points 1/4 and 3/4 through the data cell

    assign sample = (!clkdiv[3] && !clkdiv[2] && clkdiv[1] && clkdiv[0]) || (clkdiv[3] && clkdiv[2] && !clkdiv[1] && !clkdiv[0]) ;

    // Decode Manchester into NRZ code

    always @(posedge clk16x or posedge rst)
        if (rst)
            nrz <= 1'b0 ;
        else if (no_bits_rcvd > 0 && sample == 1'b1)
            nrz <= mdi2 ^ clk1x ;

    // Generate 1x clock

    always @(posedge clk16x or posedge rst)
    begin
        if (rst)
            clkdiv <= 4'b0 ;
        else if (clk1x_enable)
            clkdiv <= clkdiv + 1 ;
        else 
            clkdiv <= 4'b0000 ;
    end

    assign clk1x = clkdiv[3] ;

    // Serial to parallel conversion

    always @(posedge clk1x or posedge rst)
        if (rst)
        begin
            rsr <= 8'h0 ;
        end
        else begin
            rsr <= {rsr[6:0], ~nrz} ;
//            rsr[7:1] <= rsr[6:0] ;
//            rsr[0] <= ~nrz ;
        end

    // Transfer data from receiver shift register to data register

    always @(posedge clk1x or posedge rst)
        if (rst)
        begin
            dout <= 8'h0 ;
        end
        else begin
            dout <= rsr ;
        end

    // Determine word size

    always @(posedge clk1x or posedge rst or negedge clk1x_enable)
    begin
        if (rst)
            no_bits_rcvd <= 4'b0000 ;
        else if (!clk1x_enable)
        begin
            no_bits_rcvd <= 4'b0000 ;
        end
        else
            no_bits_rcvd <= no_bits_rcvd + 1 ;
    end

    // Generate data_ready status signal

    always @(negedge clk1x_enable or posedge rst)
    begin
        if (rst)
            data_ready <= 1'b0 ;
        else  if (!rdn)
            data_ready <= 1'b0 ;
        else
            data_ready <= 1'b1 ;
    end

endmodule







