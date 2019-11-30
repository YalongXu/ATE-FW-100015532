//////////////////////////////////////////////////////////////////////////////////
// Company: 		Jabil
// Engineer: 		Wade Jameson
// 
// Create Date:    	10:07:12 08/18/2008 
// Module Name:    	ENETIF 
// Target Devices: 	XC3S700AN-4FGG484
// Tool versions: 	Xilinx ISE Webpack 10.1
// Description: 
//
// Dependencies:
//      (1) DCM_SP
//      (2) BUFG
//
//      DP_RAM_2R_1W    (dual_port_ram.v)
//      DP_RAM_1R_1W    (dual_port_ram.v)
//      MED             (manchester_ed.v)
//          - md        (manchester_decoder.v)
//          - me        (manchester_encoder.v)
//
// Revision History: 
// V1.20	02/05/2009	W. Jameson		- Module Update
//      * Added 1 more clock cycle for the rxbuf_we - now at 3 clock cycles.
//
// V1.10	02/04/2009	W. Jameson		- Module Update
//      * Added 1 more clock cycle for the rxbuf_we.
//
// V1.00	10/27/2008	W. Jameson		- Initial Release
//      * Verified in HW.
//      * Updated the state machine and the med code downloaded from Xilinx.
//          - Encoder Changes:
//              - Changed the number of bits transmitted from 16 to 10.
//              - Added a clear of clkdiv when clk1x_enable == 1'b0.
//              - Removed parity generator - not used.
//          - Decoder Changes:
//              - Set clk1x_enable == 1'b1 when first edge received.
//              - Added a clear of clkdiv when clk1x_enable == 1'b0.
//              - Changed nrz input data to the 8-bit receive register to it's 
//                  complement. 
//      * Added ability to reset state machine from software (in case data is 
//          never received).
//      * Added a clear of the Rx buffer for the number of bytes in txr_bytes when 
//          in buffered Tx/Rx mode.
//
// V0.20	10/01/2008	W. Jameson		- Module Update
//      * Updated the shift TX and RX register scheme with the Manchester Encoder/
//          Decoder module to work with the hardware.
//
// V0.10	09/22/2008	W. Jameson		- Initial Design
//
// V0.01	08/18/2008	W. Jameson		- File Created
//
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ps / 1ps

/* Memory Map */
`define TXDHR       14'h0000    /* Transmit Data Holding Register - Single byte transfer */
`define RXDHR       14'h0000    /* Receive Data Holding Register - Single byte transfer */
`define TXRCSR      14'h0001    /* Transfer Control & Status Register */
`define CKCR        14'h0002    /* DCM Clock Control Register */
`define CKSR        14'h0003    /* DCM Clock Status Register */
`define LEDCR       14'h0004    /* LED Control Register */

`define BUF_SIZE    14'h0800    /* Buffer size - 2kB */
`define TXBUF       14'h1000    /* Transmit buffer base address */
`define RXBUF       14'h2000    /* Receive buffer base address */

(* opt_mode="speed", optimize_primitives="yes" *)
module ENETIF(
    /* OPB Interface */
    output  [31:0]  OPB_DO,
    input   [18:0]  OPB_DI,
    input   [13:0]  OPB_ADDR,
    input           OPB_RE,
    input           OPB_WE,
    input           OPB_CLK,
    input           OPB_RST,
    
    /* Clock Inputs */
    input           SDCLK,
    input           LED_CLK,
    
    /* ENET Interface */
    output          ENET_TXD,
    input           ENET_RXD,
    output          ENET_LILED,
    output          ENET_ACTLED
);

/* Transfer Control & Status Register */
    wire        tx_empty;
    wire        rx_drdy;
    wire [7:0]  med_dout;
    reg  [7:0]  med_din;
    reg         med_we;
    reg         med_en;

    reg  [3:0]  txr_fsm;

    reg         sw_rst;
    
    reg  [7:0]  tx_dhr;
    reg  [7:0]  rx_dhr;
    
    reg  [10:0] txr_bytes;
    reg  [10:0] txr_cnt;
    
    reg         txr_enable;
    reg         txr_busy;

    reg         txr_single;
    reg         txr_single_dly;
    
    reg         rxbuf_rst;
    reg  [10:0] rxbuf_rst_cnt;

    wire        mod_rst     = sw_rst | OPB_RST;
    wire [31:0] txr_csr_rb  = {
        1'b0,
        txr_cnt,
        1'b0,
        txr_bytes,
        txr_fsm,
        sw_rst,
        1'b0,
        txr_busy,
        1'b0
    };

/* Clock Control & Status Register */
    reg         dcm_rst;
    wire        dcm_locked;
    wire [31:0] clk_cr_rb = {
        31'b0,
        dcm_rst
    };
    wire [31:0] clk_sr_rb = {
        31'b0,
        dcm_locked
    };

/* LED Control Register */
    reg         liled_clk_en;
    reg         liled_do;
    reg         actled_clk_en;
    reg         actled_do;
    wire [31:0] led_cr_rb = {
        28'b0,
        actled_do,
        liled_do,
        actled_clk_en,
        liled_clk_en
    };

/* 16x Clock Generator - Expected input: 10.667MHz (93.75ns) */
/*                     - Output: 170.667MHz (5.86ns) */
    wire sd_clk16x;
    Clock16x sd_clk16x_inst(
        .POWERDOWN(~dcm_rst),
        .CLKA(SDCLK),
        .LOCK(dcm_locked),
        .GLA(sd_clk16x)
    );
        
    
/* TX Buffer - 2kB */
    wire [7:0]  txbuf_opb_do;
    wire        txbuf_opb_we = OPB_WE & (OPB_ADDR >= `TXBUF) & (OPB_ADDR < `TXBUF+`BUF_SIZE);
    wire [7:0]  txbuf_out;
    
    assign OPB_DO = (OPB_RE & (OPB_ADDR >= `TXBUF) & (OPB_ADDR < `TXBUF+`BUF_SIZE)) ?
                    {24'b0, txbuf_opb_do} : 32'bz;
    
//    DP_RAM_2R_1W #(
//        .DATA_WIDTH(8),
//        .ADDR_WIDTH(11)
//    ) tx_buf_ram(
//        .PA_DI(OPB_DI[7:0]),
//        .PA_DO(txbuf_opb_do),
//        .PA_ADDR(OPB_ADDR[10:0]),
//        .PA_WE(txbuf_opb_we),
//        .PA_CLK(OPB_CLK),        
//
//        .PB_DO(txbuf_out),
//        .PB_ADDR(txr_cnt),
//		.PB_CLK(sd_clk16x)
//    );

    Ram2048x8_DPort tx_buf_ram(
        .DINA(OPB_DI[7:0]),
        .DOUTA(txbuf_opb_do),
        .ADDRA(OPB_ADDR[10:0]),
        .RWA(~txbuf_opb_we),
        .BLKA(txbuf_opb_we),
        .CLKA(OPB_CLK),

        .DINB(8'h00),
        .DOUTB(txbuf_out),
        .ADDRB(txr_cnt),
        .RWB(1'b1),
        .BLKB(1'b1),
        .CLKB(sd_clk16x)
    );

/* RX Buffer - 2kB */
    wire [10:0] rxbuf_addr_in = (rxbuf_rst) ? rxbuf_rst_cnt : txr_cnt;
    reg  [7:0]  rxbuf_in;
    reg         rxbuf_we;
    wire [7:0]  rxbuf_opb_do;

    wire rxbuf_re = OPB_RE & (OPB_ADDR >= `RXBUF) & (OPB_ADDR < (`RXBUF+`BUF_SIZE));
    assign OPB_DO = (rxbuf_re) ? {24'b0, rxbuf_opb_do} : 32'bz;
    
//    DP_RAM_2R_1W #(
//        .DATA_WIDTH(8),
//        .ADDR_WIDTH(11)
//    ) rx_buf_ram(
//        .PA_DI(rxbuf_in),
//        .PA_ADDR(rxbuf_addr_in),
//        .PA_WE(rxbuf_we),
//        .PA_CLK(sd_clk16x),
//        
//        .PB_DO(rxbuf_opb_do),
//        .PB_ADDR(OPB_ADDR[10:0]),
//        //.PB_RE(rxbuf_re),
//        .PB_CLK(OPB_CLK)
//    );

    Ram2048x8_TPort rx_buf_ram(
        .WD(rxbuf_in),
        .WADDR(rxbuf_addr_in),
        .WEN(rxbuf_we),
        .WCLK(sd_clk16x),

        .RD(rxbuf_opb_do),
        .RADDR(OPB_ADDR[10:0]),
        .REN(1'b1),
        .RCLK(OPB_CLK)
    );
    
/* Manchester Encoder/Decoder */
    wire med_rst = ~med_en | mod_rst;
    
    MED med_mod(
        .rst(med_rst),          // Module reset input - needs to be asserted between T/R cycles
        .clk16x(sd_clk16x),     // 16x clock reference input for clock recovery & center sampling
        .mdi(ENET_RXD),         // Serial manchester data input
        .rdn(med_en),           // Control input to enable the read operation
        .dout(med_dout),        // Parallel data bus output
        .data_ready(rx_drdy),   // Status output indicating data is present on dout
        .wrn(med_we),           // Control input to strobe the din data into the buffer register
        .din(med_din),          // Parallel data bus input
        .tbre(tx_empty),        // Status output indicating the encoder can except data
        .mdo(ENET_TXD)          // Serial manchester data output
    );

/* TX/RX + RX Buffer Reset State Machine */
    always@(negedge sd_clk16x or posedge mod_rst) begin
        if(mod_rst) begin
            txr_fsm   		<= 4'b0000;
            txr_busy  		<= 1'b0;
            rx_dhr    		<= 8'h00;
            rxbuf_rst 		<= 1'b0;
            rxbuf_in  		<= 8'h00;
            rxbuf_we  		<= 1'b0;
            med_we    		<= 1'b0;
            med_din   		<= 8'h00;
            med_en    		<= 1'b0;
            txr_cnt			<= 11'h000;
            rxbuf_rst_cnt 	<= 11'h000;
        end
        else if(txr_enable) begin
            (* full_case, parallel_case *)
            case(txr_fsm)
                /* S0 - Set busy status true */
                /*    - Clear buffer counter */
                /*    - Clear Rx holding reg */
                /*    - If buffered, set to clear RX buffer */
                4'b0000: begin
                    txr_busy <= 1'b1;
                    txr_cnt  <= 11'h000;
                    rx_dhr   <= 8'h00;
                    
                    if(!txr_single) begin
                        rxbuf_rst     <= 1'b1;
                        rxbuf_in      <= 8'h00;
                        rxbuf_we      <= 1'b1;
                        rxbuf_rst_cnt <= 11'h000;
                        txr_fsm       <= txr_fsm + 1;
                    end
                    else
                        txr_fsm <= txr_fsm + 3;
                end
                /* S1 - Allow Rx buffer cell to be cleared  */
                4'b0001: begin
                    txr_fsm <= txr_fsm + 1;
                end
                /* S2 - If Rx buffer clear counter is not equal to byte Tx counter, */
                /*        increment counter and go back 1 */
                4'b0010: begin
                    if(rxbuf_rst_cnt < txr_bytes) begin
                        rxbuf_rst_cnt <= rxbuf_rst_cnt + 1;
                        txr_fsm       <= txr_fsm - 1;
                    end
                    else begin
                        rxbuf_rst <= 1'b0;
                        rxbuf_we  <= 1'b0;
                        txr_fsm   <= txr_fsm + 1;
                    end
                end
                /* S3 - Enable encoder/decoder */
                /*    - If buffered, transfer buffer byte to encoder input, */
                /*        otherwise, transfer Tx holding reg byte to encoder input */
                4'b0011: begin
                    med_en <= 1'b1;
                    
                    if(!txr_single) begin
                        med_din <= txbuf_out;
                    end
                    else begin
                        med_din <= tx_dhr;
                    end
                    
                    txr_fsm <= txr_fsm + 1;
                end
                /* S4 - Set encoder write enable */ 
                4'b0100: begin
                    med_we  <= 1'b1;
                    txr_fsm <= txr_fsm + 1;
                end
                /* S5 - Delay 1 clock cycle */
                4'b0101: begin
                    txr_fsm <= txr_fsm + 1;
                end
                /* S6 - Clear encoder write enable */
                4'b0110: begin
                    med_we  <= 1'b0;
                    txr_fsm <= txr_fsm + 1;
                end
                /* S7 - Wait for Tx empty & Rx drdy status to be set */
                /*    - If status is set transfer decoder byte to Rx holding reg */
                4'b0111: begin
                    if(tx_empty && rx_drdy) begin
                        rx_dhr  <= med_dout;
                        txr_fsm <= txr_fsm + 1;
                    end
                end
                /* S8 - Delay 1 clock cycle */
                4'b1000: begin
                    txr_fsm <= txr_fsm + 1;
                end
                /* S9 - Disable encoder/decoder */
                /*    - Set Rx buffer write enable */
                4'b1001: begin
                    med_en <= 1'b0;
                    
                    if(!txr_single) begin
                        rxbuf_in <= rx_dhr;
                        rxbuf_we <= 1'b1;
                        txr_fsm  <= txr_fsm + 1;
                    end
                    else begin
                        txr_fsm <= txr_fsm + 3;
                    end 
                end
                /* S10 - Delay 1 clock cycle */
                4'b1010: begin
                    txr_fsm <= txr_fsm + 1;
                end
                /* S11 - Delay 1 clock cycle */
                4'b1011: begin
                    txr_fsm <= txr_fsm + 1;
                end
                /* S12 - Delay 1 clock cycle */
                4'b1100: begin
                    txr_fsm <= txr_fsm + 1;
                end
                /* S13 - Clear Rx buffer write enable */
                4'b1101: begin
                    rxbuf_we <= 1'b0;
                    txr_fsm  <= txr_fsm + 1;
                end
                /* S14 - If more bytes to transfer, increment byte counter and go to S3, */
                /*         otherwise, clear busy and go to end */
                4'b1110: begin
                    if(!txr_single && (txr_cnt < txr_bytes)) begin
                        txr_cnt <= txr_cnt + 1;
                        txr_fsm <= 4'b0011;
                    end
                    else begin
                        txr_busy <= 1'b0;
                        txr_fsm  <= txr_fsm + 1;
                    end
                end
                /* S15 -  */
                4'b1111: begin
                    txr_fsm <= txr_fsm;
                end
            endcase
        end
        else begin
            txr_fsm     <= 4'b0000;
            txr_busy    <= 1'b0;
            rxbuf_rst   <= 1'b0;
            rxbuf_in    <= 8'h00;
            rxbuf_we    <= 1'b0;
            med_we      <= 1'b0;
            med_din     <= 8'h00;
            med_en      <= 1'b0;
        end
    end

/* LED Control */
    assign ENET_LILED  = (liled_clk_en)  ? LED_CLK : liled_do;
    assign ENET_ACTLED = (actled_clk_en) ? LED_CLK : actled_do;

/* Register Read/Write Access */
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `RXDHR))  ? {24'b0, rx_dhr} : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TXRCSR)) ? txr_csr_rb      : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `CKCR))   ? clk_cr_rb       : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `CKSR))   ? clk_sr_rb       : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `LEDCR))  ? led_cr_rb       : 32'bz;

    always@(negedge OPB_CLK or posedge mod_rst) begin
        if(mod_rst) begin
            sw_rst          <= 1'b0;
            
            txr_bytes       <= 11'h000;
            txr_enable      <= 1'b0;
            txr_single      <= 1'b0;
            txr_single_dly  <= 1'b1;
            tx_dhr			<= 8'h00;

            dcm_rst         <= 1'b1;

            actled_do       <= 1'b1;
            liled_do        <= 1'b1;
            actled_clk_en   <= 1'b0;
            liled_clk_en    <= 1'b0;
        end
        else if(OPB_WE) begin
            (* full_case, parallel_case *)
            case(OPB_ADDR)
                `TXDHR: begin
                    tx_dhr          <= OPB_DI[7:0];
                    txr_single_dly  <= 1'b0;
                    txr_single      <= 1'b1;
                end
                `TXRCSR: begin
                    txr_bytes   <= OPB_DI[18:8];
                    sw_rst      <= OPB_DI[7];
                    txr_enable  <= OPB_DI[0];
                end
                `CKCR: begin
                    dcm_rst <= OPB_DI[0];
               end
                `LEDCR: begin
                    actled_do       <= OPB_DI[3];
                    liled_do        <= OPB_DI[2];
                    actled_clk_en   <= OPB_DI[1];
                    liled_clk_en    <= OPB_DI[0];
                end
            endcase
        end
        else if(!txr_busy && !txr_single_dly) begin
            txr_enable      <= 1'b1;
            txr_single_dly  <= 1'b1;
        end
        else if(!txr_busy && txr_single_dly) begin
            txr_enable <= 1'b0;
            txr_single <= 1'b0;
        end
    end

endmodule