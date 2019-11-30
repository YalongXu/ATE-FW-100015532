/******************************************************************************
*
*    File Name:  med.v
*      Version:  1.1
*         Date:  January 22, 2000
*        Model:  Manchester Encoder Decoder Chip
* Dependencies:  me.v, md.v
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
module MED(
    rst,
    clk16x,
    mdi,
    rdn,
    dout,
    data_ready,
    wrn,
    din,
    tbre,
    mdo
);

    output tbre ;
    output mdo ;
    output [7:0] dout ;
    output data_ready ;
    input [7:0] din ;
    input rst ;
    input clk16x ;
    input wrn ;
    input rdn ;
    input mdi ;

    md u1 (rst,clk16x,mdi,rdn,dout,data_ready) ;

    me u2 (rst,clk16x,wrn,din,tbre,mdo) ;

endmodule
