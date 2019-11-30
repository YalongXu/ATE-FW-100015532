/******************************************************************************
*
*    File Name:  me.v
*      Version:  1.0
*         Date:  January 22, 2000
*        Model:  Manchester Encoder Chip
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
module me(
    rst,
    clk16x,
    wrn,
    din,
    tbre,
    mdo
);

    input           rst ;
    input           clk16x ;
    input           wrn ;
    input   [7:0]   din ;
    output          tbre ;
    output          mdo ;

    wire            clk1x ;
    reg             clk1x_enable ;
    wire            clk1x_disable ;
    reg     [3:0]   clkdiv ;
    reg     [3:0]   no_bits_sent ;
    wire            mdo ;
    reg             tbre ;
    reg     [7:0]   tsr ;
    reg     [7:0]   tbr ;
    reg             wrn1 ;
    reg             wrn2 ;

    // Form 2 FF register for write pulse detection

    always @(posedge rst or posedge clk16x)
        if (rst)
        begin
            wrn2 <= 1'b1 ;
            wrn1 <= 1'b1 ;
        end
        else
        begin
            wrn2 <= wrn1 ;
            wrn1 <= wrn ;
        end

    // Enable clock when detect edge on write pulse

    always @(posedge rst or posedge clk16x)
    begin
        if (rst)
            clk1x_enable <= 1'b0 ;
        else if (wrn1 == 1'b1 && wrn2 == 1'b0)
            clk1x_enable <= 1'b1 ;
        else if (no_bits_sent == 4'b1010)
            clk1x_enable <= 1'b0 ;
    end

    // Generate Transmit Buffer Register Empty signal

    always @(posedge rst or posedge clk16x)
    begin
        if (rst)
            tbre <= 1'b1 ;
        else if (wrn1 == 1'b1 && wrn2 == 1'b0)
            tbre <= 1'b0 ;
        else if (no_bits_sent == 4'b1010) 
            tbre <= 1'b1 ;
        else
            tbre <= 1'b0 ;
    end

    // Detect edge on write pulse to load transmit buffer

    always @(posedge rst or posedge clk16x)
    begin
        if (rst)
            tbr <= 8'h0 ;
        else if (wrn1 == 1'b1 && wrn2 == 1'b0)
            tbr <= din ;
    end

    // Increment clock

    always @(posedge rst or posedge clk16x)
    begin
        if (rst)
            clkdiv <= 4'b0000 ;
        else if (clk1x_enable == 1'b1)
            clkdiv <= clkdiv + 1 ;
        else
            clkdiv <= 4'b0000 ;
    end

    assign clk1x = clkdiv[3] ;

    // Load TSR from TBR, shift TSR

    always @(posedge rst or posedge clk1x)
    begin
        if (rst)
            tsr <= 8'h0 ;
        else if (no_bits_sent == 4'b0001)
            tsr <= tbr ;
        else if (no_bits_sent >= 4'b0010 && no_bits_sent < 4'b1010)
        begin
            tsr[7:1] <= tsr[6:0] ;
            tsr[0] <= 1'b0 ;
        end
    end

    // Generate Manchester data from NRZ

    assign mdo = tsr[7] ^ clk1x ;

    // Calculate number of bits sent

    always @(posedge rst or posedge clk1x)
    begin
        if (rst)
            no_bits_sent <= 4'b0000 ;
        else if (clk1x_enable)
            no_bits_sent <= no_bits_sent + 1 ;
        else if (clk1x_disable)
            no_bits_sent <= 4'b0000 ;
    end

    assign clk1x_disable = !clk1x_enable ;

endmodule

