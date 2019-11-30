 `timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:41:49 08/28/2008 
// Design Name: 
// Module Name:    top_100015532-TFW 
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
module top_100015532TFW(
    /* PCI Bus Communications Signals */
    inout   [31:0]  PCI_AD,
    (*PERIOD="32 MHz"*)
    input           PCI_CLK2,
    input   [3:0]   PCI_CBE,
    input           PCI_FRAME,
    input           PCI_DEVSEL,
    input           PCI_RST,

    /* PCI GPIO Signals */
    inout           PCI_TRDY,
    inout           PCI_IRDY,
    inout           PCI_PAR,
    inout           PCI_STOP,
    inout           PCI_GNT0,
    inout           PCI_REQ0,
    inout           PCI_GPERR,
    inout           PCI_SERR,
    inout           PCI_INTB,
     
    /* SYSCLK */
    (*PERIOD="80 MHz"*)
    input           SYSCLK_IN,

    output          FPGA_DONE,
    input           RESET_N,
     
    /* CAN Signals */
    output          CAN_CCLK,       
    output          CAN_ALE,
    output          CAN_RD,
    output          CAN_WR, 
    output          CAN_RST,
    output          CAN_CS1,
    output          CAN_CS2,
    input           CAN_INT1,
    input           CAN_INT2,
    output          CAN_BUF_DIR1,
    inout   [7:0]   CAN_AD,
    input           CAN_TX,
    input           CAN_RX,
    input           PCAN_TX,
    input           PCAN_RX,
     
    /* STAT LEDS */
    output          STAT_LED1,
    output          STAT_LED2,
    output          STAT_LED3,
    output          HEARTBEAT,
    output          TX_LED1,
    output          TX_LED2,
    output          RX_LED1,
    output          RX_LED2,
    output  [7:0]   SP_LED,
     
    /* Digital Inputs */
    input   [11:0]  P_SWb,
    input   [2:0]   HW_REV,
     
    input           ACC_TG_COLL,
    input           ACC_HW_RESETb,
    input           ACC_HW_STATUSb,
    input           P_OVLb,
    input           PD_UVFLT,
    input           PEND_MOT_EN_CPLDb,
    input           PEND_ACTIVE,
    input           PEND_PWR_OKb,
    input           COLL_DETECTb,
    input           COLL_DETECT,
    input           SPENLP_STATE,
    input           KVBENLP_STATE,
    input           CNTL_FPGA5,
    input           CNTL_FPGA4,
    input           CNTL_FPGA3,
    input           P28V_PGOOD_JAWb,
    input           P28V_PGOOD_ROTb,
    input           P24V_PGOODb,
    input           PD_SHUNT_SENSE,
//    input           IDSELb,
        
    /* Digital Outputs */
    output          PD_EN_PWR_ROT,
    output          PD_EN_PWR_JAW,
    output  [6:0]   AD_SEL,
    output  [2:0]   LP_MUX_S,
    output          LP_MUX_EN,
    output          CNTL_FPGA0,
    output          CNTL_FPGA1,
    output          CNTL_FPGA2,
    output          KVBENLP_CNTL,
    output          SPENLP_CNTL,
    output          SP485_1_DIR,
    output          SP485_2_DIR,
    output          COLLISION_PWR_EN,
    output          PEND_PWR_EN,
    output          ACC_LEDGRN,
    output          ACC_LEDRED,
    output          ACC_LED_PWR_EN,
    output          ACC_RST_LMP_CNTL,
    output          PD_DRV_EN_FPGA_R,
    output          PD_DRV_EN_JAW,
    output          EN_12Vb,
     
    /* ADC 1 */
    output          AD1_CNVSTb,
    output          AD1_SCLK,
    input           AD_SDOUT1,
    input           AD_BUSY1,

    /* ADC 1 */
    output          AD2_CNVSTb,
    output          AD2_SCLK,
    input           AD_SDOUT2,
    input           AD_BUSY2,
     
    /* RS485 Inputs */
    input           SYNC,
    input           BMPLS,
    input           COLLISION_IN,
    input           COLL_SP1_IN,
    input           COLL_SP2_IN,
    input           IMTX_IN,
    input           TST_SPI_MISO,
    input           EATX_IN,
    input           AMTX_IN,
    input           TCTX_IN,
    input           Sp485_1_R,
    input           Sp485_2_R,

    /* RS485 Outputs */
    output          COLL_CS_OUTb,
    output          COLL_CLK_OUT,
    output          COLLISION_OUT,
    output          TST_SPI_CLK,
    output          TST_SPI_MOSI,
    output          TST_SPI_CS,
    output          Sp485_1_D,
    output          Sp485_2_D,
     
    /* Bridge Drivers */
    output  [3:1]   PD_PWM1,
    output  [3:1]   PD_PWM2,
    output  [3:1]   PD_PWM3,
    output  [3:1]   PD_PWM4,
    output  [3:1]   PD_PWM5,
    output  [2:1]   PD_PWM6,
    input   [7:1]   PD_OVER_CURR,
    output  [7:1]   PD_ENABLE,
    output  [5:1]   PD_CUR_SL,
    input   [7:1]   PD_FAULTb,
     
    /* current limit DAC */
    output          ILIM_DAC_CS,
    output          ILIM_DAC_SDI,
    output          ILIM_DAC_CLK,

    output  [20:1]  RES_PWM,
     
    /* MEL */ 
    input           MEL_INT,
    input           MEL_ENABLE,
    input           MEL_ERROR,
    output          MEL_ACK,
    output          MEL_ERROR_RESET,
    input   [3:1]   MEL_XTRA,
    input           FPGA2CPLD_CLK,
    input   [7:0]   CPLD_VERSION,
    input   [4:1]   MEL_BUS,
     
    /* HotLink */ 
    output          REFCLK,
    output          RTCLK,
    input           VLTN,
     
    output  [7:0]   TXDATA,
    output  [3:0]   TXCMD,
    output          TXSC,
    output          TXENb,
    output          TXRSTb,
    input           TXEMPTYb,
     
    input   [7:0]   RXDATA,
    input   [3:0]   RXCMD,
    input           RXSC,
    output          RXENb,
    output          RXRSTb,
    input           RXEMPTYb,
     
    input           LFI1b,
    input           LFI2b,
    output          CE1b,
    output          CE2b,
        
    /* Test Points */
    output          TP133,
    output          TP134,
    output          TP135,
    output          TP143,  
    output          TP145,  
    output          TP146,  
    output          TP147,  
    output          TP148,  
    output          TP149,  
    output          TP150,  
    output          TP151,  
    output          TP152,  
    output          TP156
);

    reg SYSCLK;
    always@(posedge SYSCLK_IN) begin
        SYSCLK <= ~SYSCLK;
    end

    assign FPGA_DONE = RESET_N;

    /* OPB Interface */
    wire [31:0] opb_di;
    wire [31:0] opb_do;
    wire [31:0] opb_addr;
    wire        opb_re;
    wire        opb_we;
    wire        opb_clk;
    wire        opb_rst;
    /*  END OPB Interface */

    /*  Scratch Pads */
    wire        sp1_re;
    wire        sp1_we;
    wire        sp2_re;
    wire        sp2_we;
    reg  [31:0] dev_sp1;    //scratch pad register1
    reg  [31:0] dev_sp2;    //scratch pad register2
    
    assign opb_do = sp1_re ? dev_sp1 : 32'bz;
    assign opb_do = sp2_re ? dev_sp2 : 32'bz;
    
    always@(negedge opb_clk or posedge opb_rst) begin
        if(opb_rst) begin   
            dev_sp1 <= 32'hf5f5f5f5;
            dev_sp2 <= 32'hacacacac;
        end
        else if(sp1_we) begin
            dev_sp1 <= opb_di;
        end
        else if(sp2_we) begin
            dev_sp2 <= opb_di;           
        end
    end
    /*  END Scratch Pads */
    
    /* Digital Outputs */
    reg [27:0]  digital_out;
    reg [3:0]   lpMuxCont;
    reg [6:0]   adselCont;
    
    assign LP_MUX_EN        = lpMuxCont[3];
    assign LP_MUX_S         = lpMuxCont[2:0];
    assign AD_SEL           = adselCont;
    assign PD_EN_PWR_ROT    = digital_out[27];
    assign PD_EN_PWR_JAW    = digital_out[26];
    assign CNTL_FPGA0       = digital_out[15];  //PWRENLP_CNTL
    assign CNTL_FPGA1       = digital_out[14];  //BMENLP_CNTL
    assign CNTL_FPGA2       = digital_out[13];  //MTNENLP_CNTL
    assign KVBENLP_CNTL     = digital_out[12];
    assign SPENLP_CNTL      = digital_out[11];
    assign SP485_1_DIR      = digital_out[10];
    assign SP485_2_DIR      = digital_out[9];
    assign COLLISION_PWR_EN = digital_out[8];
    assign PEND_PWR_EN      = digital_out[7];
    assign ACC_LEDGRN       = digital_out[6];
    assign ACC_LEDRED       = digital_out[5];
    assign ACC_LED_PWR_EN   = digital_out[4];
    assign ACC_RST_LMP_CNTL = digital_out[3];
    assign PD_DRV_EN_FPGA_R = digital_out[2];
    assign PD_DRV_EN_JAW    = digital_out[1];
    assign EN_12Vb          = digital_out[0];
    
    wire do_re;
    wire do_we;
    wire lpMux_re;
    wire lpMux_we;
    wire adSel_we;
    wire adSel_re;
    
    assign opb_do = do_re    ? {4'b0,  digital_out} : 32'bz;
    assign opb_do = lpMux_re ? {28'b0, lpMuxCont}   : 32'bz;
    assign opb_do = adSel_re ? {25'h0, adselCont}   : 32'bz;

    always@(negedge opb_clk or posedge opb_rst) begin
        if(opb_rst) begin
            digital_out <= 28'h30001;
            lpMuxCont   <= 4'h0;            
            adselCont   <= 7'h0;
        end 
        else if(do_we) begin
            digital_out <= opb_di[27:0];
        end
        else if(lpMux_we) begin
            lpMuxCont <= opb_di[3:0];
        end
        else if(adSel_we) begin
            adselCont <= opb_di[6:0];
        end
    end
    /* END Digital Outputs */

    /* Digital Inputs */
    reg [31:0] digital_in;
    reg [11:0] p_switch;
    
    wire di_re;
    wire p_sw_re;

    assign opb_do = di_re   ? digital_in        : 32'bz;
    assign opb_do = p_sw_re ? {20'b0, p_switch} : 32'bz;
        
    always@(posedge opb_clk or posedge opb_rst) begin   
        if(opb_rst) begin
            digital_in <= 32'h0;
            p_switch   <= 12'h0;
        end
        else begin
            p_switch         <= P_SWb;
            digital_in[0]    <= ACC_TG_COLL;
            digital_in[1]    <= ACC_HW_RESETb;
            digital_in[2]    <= ACC_HW_STATUSb;
            digital_in[3]    <= PEND_PWR_OKb;
            digital_in[4]    <= PD_SHUNT_SENSE;
            digital_in[5]    <= P_OVLb;
            digital_in[6]    <= PD_UVFLT;
            digital_in[7]    <= 1'b0;
//          digital_in[7]    <= IDSELb;
            digital_in[15:8] <= CPLD_VERSION;
            digital_in[16]   <= PEND_MOT_EN_CPLDb;
            digital_in[17]   <= PEND_ACTIVE;
            digital_in[18]   <= COLL_DETECTb;
            digital_in[19]   <= COLL_DETECT;
            digital_in[20]   <= SPENLP_STATE;
            digital_in[21]   <= KVBENLP_STATE;
            digital_in[22]   <= CNTL_FPGA5;
            digital_in[23]   <= CNTL_FPGA4;
            digital_in[24]   <= CNTL_FPGA3;
            digital_in[25]   <= HW_REV[0];
            digital_in[26]   <= HW_REV[1];
            digital_in[27]   <= HW_REV[2];
            digital_in[28]   <= 1'b0;
            digital_in[29]   <= P28V_PGOOD_JAWb;
            digital_in[30]   <= P28V_PGOOD_ROTb;
            digital_in[31]   <= P24V_PGOODb;
        end
    end
    /* END Digital Inputs */
     
    /* Resolver Output */
    reg [19:0] resolver_out;
    
    wire resolver_clk;
    wire res_out_we;
    
    assign RES_PWM[1]  = resolver_out[0] & resolver_clk;
    assign RES_PWM[2]  = ~(resolver_out[1] & resolver_clk);
    assign RES_PWM[3]  = resolver_out[2] & resolver_clk;
    assign RES_PWM[4]  = ~(resolver_out[3] & resolver_clk);
    assign RES_PWM[5]  = resolver_out[4] & resolver_clk;
    assign RES_PWM[6]  = ~(resolver_out[5] & resolver_clk);
    assign RES_PWM[7]  = resolver_out[6] & resolver_clk;
    assign RES_PWM[8]  = ~(resolver_out[7] & resolver_clk);
    assign RES_PWM[9]  = resolver_out[8] & resolver_clk;
    assign RES_PWM[10] = ~(resolver_out[9] & resolver_clk);
    assign RES_PWM[11] = resolver_out[10] & resolver_clk;
    assign RES_PWM[12] = ~(resolver_out[11] & resolver_clk);
    assign RES_PWM[13] = resolver_out[12] & resolver_clk;
    assign RES_PWM[14] = ~(resolver_out[13] & resolver_clk);
    assign RES_PWM[15] = resolver_out[14] & resolver_clk;
    assign RES_PWM[16] = ~(resolver_out[15] & resolver_clk);
    assign RES_PWM[17] = resolver_out[16] & resolver_clk;
    assign RES_PWM[18] = ~(resolver_out[17] & resolver_clk);
    assign RES_PWM[19] = resolver_out[18] & resolver_clk;
    assign RES_PWM[20] = ~(resolver_out[19] & resolver_clk);
    
    /* Top Level Write Access */
    always@(negedge opb_clk or posedge opb_rst) begin
        if(opb_rst) begin
            resolver_out <= 20'h0;      
        end 
        else if(res_out_we)begin
            resolver_out <= opb_di[19:0];
        end
    end
    /* END Resolver Output */
        
    wire opbclkub;
    wire pci_opb_rst;
    assign opb_rst = pci_opb_rst || ~RESET_N;
    PCI_EMU_TARGET pci_target(
         .PCI_AD(PCI_AD),
         .PCI_CLK2(PCI_CLK2),
         .PCI_CBE(PCI_CBE),
         .PCI_FRAME(PCI_FRAME),
         .PCI_DEVSEL(PCI_DEVSEL),
         .PCI_RST(PCI_RST),
         .PCI_CLK3(1'b0),
         .PCI_CLK4(1'b0),
         .PCI_TRDY(PCI_TRDY),
         .PCI_IRDY(PCI_IRDY),
         .PCI_PAR(PCI_PAR),
         .PCI_STOP(PCI_STOP),
         .PCI_GNT0(PCI_GNT0),
         .PCI_REQ0(PCI_REQ0),
         .PCI_GPERR(PCI_GPERR),
         .PCI_SERR(PCI_SERR),
         .PCI_INTB(PCI_INTB),
         .OPB_DI(opb_di),
         .OPB_DO(opb_do),
         .OPB_ADDR(opb_addr),
         .OPB_RE(opb_re),
         .OPB_WE(opb_we),
         .OPB_CLK(opb_clk),
         .OPB_RST(pci_opb_rst)
    );

    wire can_control_re;
    wire can_control_we;
    CAN_SJA1000 can_control(
         .OPB_DO(opb_do),
         .OPB_DI(opb_di[7:0]),
         .OPB_ADDR(opb_addr[19:4]),
         .OPB_RE(can_control_re),
         .OPB_WE(can_control_we),
         .OPB_CLK(opb_clk),
         .OPB_RST(opb_rst),
         .CAN_RST(CAN_RST),
         .CAN_INT1(CAN_INT1),
         .CAN_INT2(CAN_INT2),
         .CAN_AD(CAN_AD),
         .CAN_ALE(CAN_ALE),
         .CAN_RD(CAN_RD),
         .CAN_WR(CAN_WR),
         .CAN_CS1(CAN_CS1),
         .CAN_CS2(CAN_CS2),
         .CAN_BUF_DIR1(CAN_BUF_DIR1)
    );
     
    wire ref_clk;
    wire osc_count_re;
    wire osc_count_we;
    OSCILLATOR_COUNTER osc_count(
         .OPB_DO(opb_do),
         .OPB_DI(opb_di),
         .OPB_ADDR(opb_addr[3:2]),
         .OPB_RE(osc_count_re),
         .OPB_WE(osc_count_we),
         .OPB_CLK(opb_clk),
         .OPB_RST(opb_rst),
         .SYSCLK(SYSCLK),
         .REF_CLK(ref_clk)
    );

    wire clk_aq;
    wire clk_sd;
    wire ad1_re;
    wire ad1_we;
    ADC_AD7663AS ad1_mod(
        .OPB_DO(opb_do),
        .OPB_DI(opb_di[11:0]),
        .OPB_ADDR(opb_addr[13:2]),
        .OPB_RE(ad1_re),
        .OPB_WE(ad1_we),
        .OPB_CLK(opb_clk),
        .OPB_RST(opb_rst),
        .clk_aq(clk_aq),
        .clk_sd(clk_sd),
        .AD_CNVST(AD1_CNVSTb),
        .AD_SCLK(AD1_SCLK),
        .AD_SDOUT(AD_SDOUT1),
        .AD_BUSY(AD_BUSY1)
    );
    
    wire ad2_re;
    wire ad2_we;
    ADC_AD7663AS ad2_mod(
        .OPB_DO(opb_do),
        .OPB_DI(opb_di[11:0]),
        .OPB_ADDR(opb_addr[13:2]),
        .OPB_RE(ad2_re),
        .OPB_WE(ad2_we),
        .OPB_CLK(opb_clk),
        .OPB_RST(opb_rst),
        .clk_aq(clk_aq),
        .clk_sd(clk_sd),
        .AD_CNVST(AD2_CNVSTb),
        .AD_SCLK(AD2_SCLK),
        .AD_SDOUT(AD_SDOUT2),
        .AD_BUSY(AD_BUSY2)
    );
    
    wire clk_data;
    wire rs485_re;
    wire rs485_we;
    RS485 rs485_mod(
        .OPB_DO(opb_do),
        .OPB_DI(opb_di),
        .OPB_ADDR(opb_addr[6:2]),
        .OPB_RE(rs485_re),
        .OPB_WE(rs485_we),
        .OPB_CLK(opb_clk),
        .OPB_RST(opb_rst),
        .DATACLK(clk_data),
        .SYNC(SYNC),
        .BMPLS(BMPLS),
        .COLL_SP1_IN(COLL_SP1_IN),
        .COLL_SP2_IN(COLL_SP2_IN),
        .IMTX_IN(IMTX_IN),
        .TST_SPI_MISO(TST_SPI_MISO),
        .EATX_IN(EATX_IN),
        .AMTX_IN(AMTX_IN),
        .TCTX_IN(TCTX_IN),
        .SP485_1_R(Sp485_1_R),
        .SP485_2_R(Sp485_2_R),
        .COLL_CS_OUTb(COLL_CS_OUTb),
        .COLL_CLK_OUT(COLL_CLK_OUT),
        .TST_SPI_CLK(TST_SPI_CLK),
        .TST_SPI_MOSI(TST_SPI_MOSI),
        .TST_SPI_CS(TST_SPI_CS),
        .Sp485_1_D(Sp485_1_D),
        .Sp485_2_D(Sp485_2_D)
    );
    
    wire brg1_re;
    wire brg1_we;
    BRIDGE_CONT brg1(
        .OPB_DO(opb_do),
        .OPB_DI(opb_di),
        .OPB_ADDR(opb_addr[5:2]),
        .OPB_RE(brg1_re),
        .OPB_WE(brg1_we),
        .OPB_CLK(opb_clk),              
        .OPB_RST(opb_rst),
        .SYSCLK(SYSCLK),                    
        .PD_PWM(PD_PWM1),
        .CUR_SAMP(PD_CUR_SL[1]),
        .PD_ENABLE(PD_ENABLE[1]),
        .OVER_CURR(PD_OVER_CURR[1]),
        .FAULTb(PD_FAULTb[1])  
);
    wire brg2_re;
    wire brg2_we;
    BRIDGE_CONT brg2(
        .OPB_DO(opb_do),
        .OPB_DI(opb_di),
        .OPB_ADDR(opb_addr[5:2]),
        .OPB_RE(brg2_re),
        .OPB_WE(brg2_we),
        .OPB_CLK(opb_clk),              
        .OPB_RST(opb_rst),
        .SYSCLK(SYSCLK),                    
        .PD_PWM(PD_PWM2),
        .CUR_SAMP(PD_CUR_SL[2]),
        .PD_ENABLE(PD_ENABLE[2]),
        .OVER_CURR(PD_OVER_CURR[2]),
        .FAULTb(PD_FAULTb[2])  
    );
    wire brg3_re;
    wire brg3_we;
    BRIDGE_CONT brg3(
        .OPB_DO(opb_do),
        .OPB_DI(opb_di),
        .OPB_ADDR(opb_addr[5:2]),
        .OPB_RE(brg3_re),
        .OPB_WE(brg3_we),
        .OPB_CLK(opb_clk),              
        .OPB_RST(opb_rst),
        .SYSCLK(SYSCLK),                    
        .PD_PWM(PD_PWM3),
        .CUR_SAMP(PD_CUR_SL[3]),
        .PD_ENABLE(PD_ENABLE[3]),
        .OVER_CURR(PD_OVER_CURR[3]),
        .FAULTb(PD_FAULTb[3])  
    );
    wire brg4_re;
    wire brg4_we;   
    BRIDGE_CONT brg4(
        .OPB_DO(opb_do),
        .OPB_DI(opb_di),
        .OPB_ADDR(opb_addr[5:2]),
        .OPB_RE(brg4_re),
        .OPB_WE(brg4_we),
        .OPB_CLK(opb_clk),              
        .OPB_RST(opb_rst),
        .SYSCLK(SYSCLK),                    
        .PD_PWM(PD_PWM4),
        .CUR_SAMP(PD_CUR_SL[4]),
        .PD_ENABLE(PD_ENABLE[4]),
        .OVER_CURR(PD_OVER_CURR[4]),
        .FAULTb(PD_FAULTb[4])  
    );
    wire brg5_re;
    wire brg5_we;
    BRIDGE_CONT brg5(
        .OPB_DO(opb_do),
        .OPB_DI(opb_di),
        .OPB_ADDR(opb_addr[5:2]),
        .OPB_RE(brg5_re),
        .OPB_WE(brg5_we),
        .OPB_CLK(opb_clk),              
        .OPB_RST(opb_rst),
        .SYSCLK(SYSCLK),                
        .PD_PWM(PD_PWM5),
        .CUR_SAMP(PD_CUR_SL[5]),
        .PD_ENABLE(PD_ENABLE[5]),
        .OVER_CURR(PD_OVER_CURR[5]),
        .FAULTb(PD_FAULTb[5])  
    );
    wire brk1_re;
    wire brk1_we;
    BRAKE_CONT brk1(
        .OPB_DO(opb_do),
        .OPB_DI(opb_di),
        .OPB_ADDR(opb_addr[5:2]),
        .OPB_RE(brk1_re),
        .OPB_WE(brk1_we),
        .OPB_CLK(opb_clk),              
        .OPB_RST(opb_rst),
        .SYSCLK(SYSCLK),                    
        .PD_PWM(PD_PWM6[1]),
        .PD_ENABLE(PD_ENABLE[6]),
        .OVER_CURR(PD_OVER_CURR[6]),
        .FAULTb(PD_FAULTb[6])  
    );
    wire brk2_re;
    wire brk2_we;   
    BRAKE_CONT brk2(
        .OPB_DO(opb_do),
        .OPB_DI(opb_di),
        .OPB_ADDR(opb_addr[5:2]),
        .OPB_RE(brk2_re),
        .OPB_WE(brk2_we),
        .OPB_CLK(opb_clk),              
        .OPB_RST(opb_rst),
        .SYSCLK(SYSCLK),                    
        .PD_PWM(PD_PWM6[2]),
        .PD_ENABLE(PD_ENABLE[7]),
        .OVER_CURR(PD_OVER_CURR[7]),
        .FAULTb(PD_FAULTb[7])  
    );
    
    wire ilim_dac_re;
    wire ilim_dac_we;
    DAC_AD8803AR ilim_dac_mod(
        .ILIM_DAC_CLK(ILIM_DAC_CLK),
        .ILIM_DAC_SDI(ILIM_DAC_SDI),
        .ILIM_DAC_CS(ILIM_DAC_CS),   
        .OPB_DO(opb_do),
        .OPB_DI(opb_di[15:0]),
        .OPB_ADDR(opb_addr[4:2]),
        .OPB_RE(ilim_dac_re),
        .OPB_WE(ilim_dac_we),
        .OPB_CLK(opb_clk),  
        .OPB_RST(opb_rst),
        .SYSCLK(SYSCLK)
    );
     
     wire mel_re;
     wire mel_we;
     MEL mel_mod(
        .MEL_INT(MEL_INT),
        .MEL_ENABLE(MEL_ENABLE),
        .MEL_ERROR(MEL_ERROR),
        .MEL_ACK(MEL_ACK),
        .MEL_ERROR_RESET(MEL_ERROR_RESET),
        .MEL_XTRA(MEL_XTRA),
        .FPGA2CPLD_CLK(FPGA2CPLD_CLK),
        .CPLD_VERSION(CPLD_VERSION),
        .MEL_BUS(MEL_BUS),
        .OPB_DO(opb_do),
        .OPB_DI(opb_di[15:0]),
        .OPB_ADDR(opb_addr[6:2]),
        .OPB_RE(mel_re),
        .OPB_WE(mel_we),
        .OPB_CLK(opb_clk),  
        .OPB_RST(opb_rst),
        .SYSCLK(SYSCLK)

    );
     
    wire fopt_re;
    wire fopt_we;
    HOTLink_CY7C9689A hotlink (
        .OPB_DO(opb_do),
        .OPB_DI(opb_di[15:0]),
        .OPB_ADDR(opb_addr[13:2]),
        .OPB_RE(fopt_re),
        .OPB_WE(fopt_we),
        .OPB_CLK(opb_clk),
        .OPB_RST(opb_rst), 
        .SYSCLK(SYSCLK),
        .REFCLK(REFCLK), 
        .RTCLK(RTCLK),
        .VLTN(VLTN), 
        .TXDATA(TXDATA), 
        .TXCMD(TXCMD), 
        .TXSC(TXSC), 
        .TXENb(TXENb), 
        .TXRSTb(TXRSTb), 
        .TXEMPTYb(TXEMPTYb), 
        .RXDATA(RXDATA), 
        .RXCMD(RXCMD), 
        .RXSC(RXSC), 
        .RXENb(RXENb), 
        .RXRSTb(RXRSTb), 
        .RXEMPTYb(RXEMPTYb), 
        .LFI1b(LFI1b), 
        .LFI2b(LFI2b), 
        .CE1b(CE1b), 
        .CE2b(CE2b)
    );
    
    wire clk_10Hz;
    LED_CONTROL ledcontrol(
        .CLK_10HZ(clk_10Hz),
        .STAT_LED1(STAT_LED1),
        .STAT_LED2(STAT_LED2),
        .STAT_LED3(STAT_LED3),
        .HEARTBEAT(HEARTBEAT),
        .TX_LED1(TX_LED1),
        .TX_LED2(TX_LED2),
        .RX_LED1(RX_LED1),
        .RX_LED2(RX_LED2),
        .SP_LED(SP_LED),
        .RST(opb_rst)
    );

    wire clock_re;
    wire clock_we;
    wire enet_clk;
    ClkGen Clocks(
        .OPB_DO(opb_do),
        .OPB_DI(opb_di[15:0]),
        .OPB_ADDR(opb_addr[5:2]),
        .OPB_RE(clock_re),
        .OPB_WE(clock_we),
        .OPB_CLK(opb_clk),              
        .OPB_RST(opb_rst),
        .SYSCLK(SYSCLK),                    
        .CLK_10Hz(clk_10Hz),
        .CLK_RESOLVER(resolver_clk),
        .CLK_AQ(clk_aq),
        .CLK_DATA(clk_data),
        .CLK_CAN(CAN_CCLK), 
        .CLK_16KHz(ref_clk),
        .CLK_SD(clk_sd),
        .ENET_CLK(enet_clk),
        .PCI_CLK2(PCI_CLK2),
        .OPBCLKUB(opbclkub)
    );

    wire enet_re;
    wire enet_we;
    ENETIF coll_mod(
        .OPB_DO(opb_do),
        .OPB_DI(opb_di[18:0]),
        .OPB_ADDR(opb_addr[15:2]),
        .OPB_RE(enet_re),
        .OPB_WE(enet_we),
        .OPB_CLK(opb_clk),
        .OPB_RST(opb_rst),
        .SDCLK(enet_clk),
        .LED_CLK(clk_10Hz),
        .ENET_TXD(COLLISION_OUT),
        .ENET_RXD(COLLISION_IN)
    );
     
    AdderDecode add_dec(
        .OPB_ADDR(opb_addr[23:0]),
        .OPB_RE(opb_re),
        .OPB_WE(opb_we),
        .SP1_RE(sp1_re),
        .SP1_WE(sp1_we),
        .SP2_RE(sp2_re),
        .SP2_WE(sp2_we),
        .DO_RE(do_re),
        .DO_WE(do_we),
        .LPMUX_RE(lpMux_re),
        .LPMUX_WE(lpMux_we),
        .DI_RE(di_re),
        .P_SW_RE(p_sw_re),
        .RES_OUT_WE(res_out_we),
        .CAN_RE(can_control_re),
        .CAN_WE(can_control_we),
        .COUNTER_RE(osc_count_re),
        .COUNTER_WE(osc_count_we),
        .AD1_RE(ad1_re),
        .AD1_WE(ad1_we),
        .AD2_RE(ad2_re),
        .AD2_WE(ad2_we),
        .RS485_RE(rs485_re),
        .RS485_WE(rs485_we),
        .BRG1_RE(brg1_re),
        .BRG1_WE(brg1_we),
        .BRG2_RE(brg2_re),
        .BRG2_WE(brg2_we),
        .BRG3_RE(brg3_re),
        .BRG3_WE(brg3_we),
        .BRG4_RE(brg4_re),
        .BRG4_WE(brg4_we),
        .BRG5_RE(brg5_re),
        .BRG5_WE(brg5_we),
        .BRK1_RE(brk1_re),
        .BRK1_WE(brk1_we),
        .BRK2_RE(brk2_re),
        .BRK2_WE(brk2_we),
        .ILIM_DAC_RE(ilim_dac_re),
        .ILIM_DAC_WE(ilim_dac_we),
        .MEL_RE(mel_re),
        .MEL_WE(mel_we),
        .FOPT_RE(fopt_re),
        .FOPT_WE(fopt_we),
        .CLOCK_RE(clock_re),
        .CLOCK_WE(clock_we),
        .ADSEL_RE(adSel_re),
        .ADSEL_WE(adSel_we),
        .ENET_RE(enet_re),
        .ENET_WE(enet_we)
    );

    assign TP133 = MEL_ENABLE;
    assign TP134 = MEL_ERROR;
    assign TP135 = MEL_INT;
    assign TP143 = MEL_ACK;


//    assign TP145 = PCAN_RX;
//  assign TP146 = PCAN_TX;

    assign TP145 = REFCLK;
    assign TP146 = RTCLK;

    assign TP147 = CAN_RX;
    assign TP148 = CAN_TX;
    assign TP149 = CAN_CCLK;
//  assign TP150 = ref_clk;
    assign TP150 = opb_clk;
    assign TP151 = clk_aq;
    assign TP152 = clk_sd;
    assign TP156 = SYSCLK;

endmodule
