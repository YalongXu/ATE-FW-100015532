`timescale 1 ns/100 ps
// Version: 9.1 SPB 9.1.0.22


module Ram256x9_DPort(DINA,DOUTA,DINB,DOUTB,ADDRA,ADDRB,RWA,RWB,
        BLKA,BLKB,CLKA,CLKB);
input [8:0] DINA;
output [8:0] DOUTA;
input [8:0] DINB;
output [8:0] DOUTB;
input [7:0] ADDRA, ADDRB;
input RWA, RWB, BLKA, BLKB, CLKA, CLKB;

    wire WEAP, WEBP, VCC, GND;
    
    VCC VCC_1_net(.Y(VCC));
    GND GND_1_net(.Y(GND));
    INV WEBUBBLEB(.A(BLKB), .Y(WEBP));
    INV WEBUBBLEA(.A(BLKA), .Y(WEAP));
    RAM4K9 Ram256x9_DPort_R0C0(.ADDRA11(GND), .ADDRA10(GND), 
        .ADDRA9(GND), .ADDRA8(GND), .ADDRA7(ADDRA[7]), .ADDRA6(
        ADDRA[6]), .ADDRA5(ADDRA[5]), .ADDRA4(ADDRA[4]), .ADDRA3(
        ADDRA[3]), .ADDRA2(ADDRA[2]), .ADDRA1(ADDRA[1]), .ADDRA0(
        ADDRA[0]), .ADDRB11(GND), .ADDRB10(GND), .ADDRB9(GND), 
        .ADDRB8(GND), .ADDRB7(ADDRB[7]), .ADDRB6(ADDRB[6]), 
        .ADDRB5(ADDRB[5]), .ADDRB4(ADDRB[4]), .ADDRB3(ADDRB[3]), 
        .ADDRB2(ADDRB[2]), .ADDRB1(ADDRB[1]), .ADDRB0(ADDRB[0]), 
        .DINA8(DINA[8]), .DINA7(DINA[7]), .DINA6(DINA[6]), .DINA5(
        DINA[5]), .DINA4(DINA[4]), .DINA3(DINA[3]), .DINA2(
        DINA[2]), .DINA1(DINA[1]), .DINA0(DINA[0]), .DINB8(
        DINB[8]), .DINB7(DINB[7]), .DINB6(DINB[6]), .DINB5(
        DINB[5]), .DINB4(DINB[4]), .DINB3(DINB[3]), .DINB2(
        DINB[2]), .DINB1(DINB[1]), .DINB0(DINB[0]), .WIDTHA0(VCC), 
        .WIDTHA1(VCC), .WIDTHB0(VCC), .WIDTHB1(VCC), .PIPEA(GND), 
        .PIPEB(GND), .WMODEA(GND), .WMODEB(GND), .BLKA(WEAP), 
        .BLKB(WEBP), .WENA(RWA), .WENB(RWB), .CLKA(CLKA), .CLKB(
        CLKB), .RESET(VCC), .DOUTA8(DOUTA[8]), .DOUTA7(DOUTA[7]), 
        .DOUTA6(DOUTA[6]), .DOUTA5(DOUTA[5]), .DOUTA4(DOUTA[4]), 
        .DOUTA3(DOUTA[3]), .DOUTA2(DOUTA[2]), .DOUTA1(DOUTA[1]), 
        .DOUTA0(DOUTA[0]), .DOUTB8(DOUTB[8]), .DOUTB7(DOUTB[7]), 
        .DOUTB6(DOUTB[6]), .DOUTB5(DOUTB[5]), .DOUTB4(DOUTB[4]), 
        .DOUTB3(DOUTB[3]), .DOUTB2(DOUTB[2]), .DOUTB1(DOUTB[1]), 
        .DOUTB0(DOUTB[0]));
    
endmodule

// _Disclaimer: Please leave the following comments in the file, they are for internal purposes only._


// _GEN_File_Contents_

// Version:9.1.0.22
// ACTGENU_CALL:1
// BATCH:T
// FAM:ProASIC3E
// OUTFORMAT:Verilog
// LPMTYPE:LPM_RAM
// LPM_HINT:DUAL
// INSERT_PAD:NO
// INSERT_IOREG:NO
// GEN_BHV_VHDL_VAL:F
// GEN_BHV_VERILOG_VAL:F
// MGNTIMER:F
// MGNCMPL:T
// DESDIR:T:/Test Working/KIK2-MSVisualStudio/VMS/100015532/Firmware/100015532TFW.Actel/smartgen\Ram256x9_DPort
// GEN_BEHV_MODULE:T
// SMARTGEN_DIE:
// SMARTGEN_PACKAGE:
// AGENIII_IS_SUBPROJECT_LIBERO:T
// WWIDTH:9
// WDEPTH:256
// RWIDTH:9
// RDEPTH:256
// CLKS:2
// RESET_POLARITY:2
// INIT_RAM:F
// DEFAULT_WORD:0x000
// CASCADE:0
// WCLK_EDGE:RISE
// RCLK_EDGE:RISE
// CLKA_PN:CLKA
// CLKB_PN:CLKB
// WMODE1:0
// WMODE2:0
// PMODE1:0
// PMODE2:0
// DATAA_IN_PN:DINA
// DATAA_OUT_PN:DOUTA
// ADDRESSA_PN:ADDRA
// RWA_PN:RWA
// BLKA_PN:BLKA
// DATAB_IN_PN:DINB
// DATAB_OUT_PN:DOUTB
// ADDRESSB_PN:ADDRB
// RWB_PN:RWB
// BLKB_PN:BLKB
// WE_POLARITY:1
// RE_POLARITY:1
// PTYPE:2

// _End_Comments_

