`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:34:42 09/17/2008 
// Design Name: 
// Module Name:    HOTLink_CY7C9689A 
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
`define FO_WAIT                 8'h0    /* wait */
`define FO_TX                   8'h1    /* Transmit */
`define FO_RX                   8'h3    /* Recieve */
`define FO_RST_TX               8'h5    /* Reset TX FIFO  */
`define FO_RST_RX               8'h6    /* Reset RX FIFO  */
`define FO_RD_EMPTY_TX          8'h9
`define FO_RD_EMPTY_RX          8'ha

`define TX_S0                   2'h0
`define TX_S1                   2'h1
`define TX_S2                   2'h2
`define TX_S3                   2'h3

`define RX_S0                   3'h0
`define RX_S1                   3'h1
`define RX_S2                   3'h2
`define RX_S3                   3'h3
`define RX_S4                   3'h4

`define DEV_SELECT_ADDR         12'h0  // 0
`define CNTRL_TX_ADDR           12'h1    // 4
`define CNTRL_RX_ADDR           12'h2  // 8
`define TX_SIZE_ADDR            12'h3    // c   
`define REF_DIV_ADDR            12'h4  // 10
`define RT_DIV_ADDR             12'h5  // 14
`define TX_EMPTY_ADDR           12'h6  // 18
`define RX_EMPTY_ADDR           12'h7  // 1c
`define TX_STATE_ADDR           12'h8  // 20
`define RX_STATE_ADDR           12'h9  // 24
`define TX_RST_CNT_ADDR         12'ha  // 28
`define RX_RST_CNT_ADDR         12'hb  // 2c
`define GLITCH_CNT_ADDR         12'hc    // 30
`define DCM_STAT_ADDR           12'hd    // 34
`define FO_OUT_RAM_ADDR         12'h100
`define FO_IN_RAM_ADDR          12'h200
`define FO_RAM_SIZE             12'h100

module HOTLink_CY7C9689A(
    output  [31:0]  OPB_DO,
    input   [15:0]  OPB_DI,
    input   [11:0]  OPB_ADDR,
    input           OPB_RE,
    input           OPB_WE,
    input           OPB_CLK,    //32MHz
    input           OPB_RST,
    input           SYSCLK,     //80MHz

    output          REFCLK,     // 16MHz divided the PCI clk by 2 //
    output          RTCLK,      // 20MHz divided the sysclk by  4   //
    input           VLTN,
     
    output  [7:0]   TXDATA,     // tx data //    sampled on rising edge of RTCLK
    output  [3:0]   TXCMD,      // tx command //
    output          TXSC,       //COMMAND /data //
    output          TXENb,      // TX enable
    output          TXRSTb,     // TX FIFO reset
    input           TXEMPTYb,   // TX FIFO empty
     
    input   [7:0]   RXDATA,     // rx data //   change on rising edge of RTCLK
    input   [3:0]   RXCMD,      // rx command //
    input           RXSC,       //COMMAND /data //
    output          RXENb,      // RX enable
    output          RXRSTb,     // Reset RX FIFO
    input           RXEMPTYb,   // RX FIFO empty
     
    input           LFI1b,      // Link Fault indicator
    input           LFI2b,      // Link Fault Indicator
    output          CE1b,       // Chip Enable 1
    output          CE2b        // Chip Enable 2
);
     
    reg  [7:0]     tx_size;             // number of charater to transmit
    reg  [7:0]     fo_control_tx;
    reg  [7:0]     fo_control_rx;
    reg            chipEn_tx;
    reg            chipEn_rx;
    reg  [15:0]    refclk_divider;
    reg  [15:0]    rtclk_divider;
    reg            txen;
    reg            txreset;
    reg            rxen;
    reg            rxreset;
    reg  [7:0]     in_ram_wr_pt;
    reg  [7:0]     in_ram_wr_pt_n;
    reg  [7:0]     out_ram_rd_pt;
    reg            in_ram_wr_en;
    reg  [3:0]     reset_cntr;          // reset counter for tx fifo
    reg  [3:0]     reset_cntr_rx;       // reset counter for rx fifo
    reg            tx_empty_buf;
    reg            rx_empty_buf;
    reg            device_select;           // 0 selects IF 1, 1 selects IF 2
    reg  [1:0]     tx_state;
    reg  [2:0]     rx_state;
    reg  [31:0]    glitch_count;
     
    wire           lfi         = (CE2b) ? LFI1b : LFI2b;
    wire [10:0]    in_ram_buf  = (RXSC) ? {VLTN, lfi, RXSC, 4'h0, RXCMD} : {VLTN, lfi, RXSC, RXDATA};
    wire [8:0]     out_ram_buf;
    wire [10:0]    in_ram_do;
    wire [8:0]     out_ram_do;
    wire           chip_enB    = !(chipEn_tx | chipEn_rx);
    
    wire out_ram_re    = OPB_RE & (OPB_ADDR >= `FO_OUT_RAM_ADDR) & (OPB_ADDR < (`FO_OUT_RAM_ADDR + `FO_RAM_SIZE));
    wire out_ram_we    = OPB_WE & (OPB_ADDR >= `FO_OUT_RAM_ADDR) & (OPB_ADDR < (`FO_OUT_RAM_ADDR + `FO_RAM_SIZE));
    wire in_ram_re     = OPB_RE & (OPB_ADDR >= `FO_IN_RAM_ADDR) & (OPB_ADDR < (`FO_IN_RAM_ADDR + `FO_RAM_SIZE));

    assign TXDATA  = out_ram_buf[7:0];
    assign TXCMD   = out_ram_buf[3:0];
    assign TXSC    = out_ram_buf[8];
    assign TXENb   = !txen;
    assign RXENb   = !rxen;
    assign TXRSTb  = !txreset;
    assign RXRSTb  = !rxreset;
    assign CE1b    = (device_select) ? 1'b1     : chip_enB;
    assign CE2b    = (device_select) ? chip_enB : 1'b1;
     
    reg [2:0] pcnt = 0;
    reg [2:0] lcnt = 0;
    
    always@(posedge SYSCLK or posedge OPB_RST) begin
        if(OPB_RST)
            pcnt <= 0;
        else if(lcnt == 4)
            pcnt <= 0;
        else
            pcnt <= pcnt+1;
    end
    
    always@(negedge SYSCLK or posedge OPB_RST) begin
        if(OPB_RST) 
            lcnt <= 0;
        else if(pcnt == 0)
            lcnt <= 0;
        else
            lcnt <= lcnt+1;
    end
    
    assign REFCLK = (lcnt > 1) && (pcnt > 1);

    CLOCK_DIV rtclkdiv(
        .CLK_DIV(rtclk_divider),
        .CLK_IN(SYSCLK),
        .CLK_OUT(RTCLK),
        .RST(OPB_RST)
    );
    
//  DP_RAM_2R_1W #(
//      .ADDR_WIDTH(8),             // 256 samples of data
//      .DATA_WIDTH(11)             // 10 bit samples
//  ) data_in_ram(                      // data read from Transeiver
//      .PA_DI(in_ram_buf),         // data in buffer
//      .PA_DO(),
//      .PA_ADDR(in_ram_wr_pt),     // pointer to next memory location
//      .PA_WE(in_ram_wr_en),       // ram is controled by state machine
//      .PA_CLK(RTCLK),             // updated on positive edge of RTCLK
//      .PB_DO(in_ram_do),
//      .PB_ADDR(OPB_ADDR[7:0]),
//      .PB_CLK(OPB_CLK)
//  );

    Ram256x11_TPort data_in_ram(
        .WD(in_ram_buf),
        .WADDR(in_ram_wr_pt),
        .WEN(in_ram_wr_en),
        .WCLK(RTCLK),
        .RD(in_ram_do),
        .RADDR(OPB_ADDR[7:0]),
        .REN(in_ram_re),
        .RCLK(OPB_CLK)
    );  

//  DP_RAM_2R_1W #(
//      .ADDR_WIDTH(8),             // 256 samples of data
//      .DATA_WIDTH(9)                  // 9 bit samples
//  ) data_out_ram(                 // data write Transeiver
//      .PA_DI(OPB_DI[8:0]),            
//      .PA_DO(out_ram_do),
//      .PA_ADDR(OPB_ADDR[7:0]),            
//      .PA_WE(out_ram_we),                 
//      .PA_CLK(OPB_CLK),       
//      .PB_DO(out_ram_buf),            
//      .PB_ADDR(out_ram_rd_pt),
//      .PB_CLK(!RTCLK)                 // updated on positive edge (negative edge of RTCLK)
//  );

    wire data_out_ram_we = ~out_ram_we;
    wire data_out_ram_blk = out_ram_we | out_ram_re;
    Ram256x9_DPort data_out_ram(
        .DINA(OPB_DI[8:0]),
        .DOUTA(out_ram_do),
        .ADDRA(OPB_ADDR[7:0]),
        .RWA(data_out_ram_we),
        .BLKA(data_out_ram_blk),
        .CLKA(OPB_CLK),
        
        .DINB(9'h00),
        .DOUTB(out_ram_buf),
        .ADDRB(out_ram_rd_pt),
        .RWB(1'b1),
        .BLKB(txen),
        .CLKB(~RTCLK)
    );


////////////////////////////////////////////////////
// Transmit Path

    always@(posedge RTCLK or posedge OPB_RST) begin
        if(OPB_RST) begin
            out_ram_rd_pt <= 8'h0;
        end
        else if(fo_control_tx == `FO_TX) begin
            if(tx_state == `TX_S0) begin
                out_ram_rd_pt <= 8'h0;
            end
            else if(tx_state == `TX_S2) begin
                out_ram_rd_pt <= out_ram_rd_pt + 1;
            end
        end
    end
    
    always@(negedge RTCLK or posedge OPB_RST) begin
        if(OPB_RST) begin
            tx_state        <= `TX_S0;
            chipEn_tx       <= 1'b0;
            txen            <= 1'b0;
            txreset         <= 1'b0;
            reset_cntr      <= 4'h0;
            tx_empty_buf    <= 1'b0;
        end
        else begin
            case(fo_control_tx)
                `FO_WAIT: begin
                    tx_state <= `TX_S0;
                end
                `FO_TX: begin
                    case(tx_state)
                        `TX_S0: begin
                            chipEn_tx   <= 1'b1;
                            tx_state    <= `TX_S1;
                        end
                        `TX_S1: begin
                            txen        <= 1'b1;
                            chipEn_tx   <= 1'b0;
                            if(tx_size == 1)
                                tx_state <= `TX_S3;
                            else
                                tx_state <= `TX_S2;
                        end
                        `TX_S2: begin
                            if((out_ram_rd_pt+1) >= tx_size)                            
                                tx_state <= `TX_S3;                         
                        end
                        `TX_S3: begin
                            txen <= 1'b0;
                        end
                    endcase
                end
                `FO_RST_TX: begin
                    case(tx_state)
                        `TX_S0: begin
                            chipEn_tx   <= 1'b1;
                            txreset     <= 1'b1;
                            reset_cntr  <= 4'h0;
                            tx_state    <= `TX_S1;
                        end
                        `TX_S1: begin
                            if(reset_cntr > 4'h8)
                                tx_state <= `TX_S2;
                            else
                                reset_cntr <= reset_cntr + 1;
                        end
                        `TX_S2: begin
                            chipEn_tx   <= 1'b0;
                            txreset     <= 1'b0;
                        end
                    endcase
                end
                `FO_RD_EMPTY_TX: begin
                    case(tx_state)
                        `TX_S0: begin
                            chipEn_tx   <= 1'b1;
                            tx_state    <= `TX_S1;
                        end
                        `TX_S1: begin
                            chipEn_tx    <= 1'b0;
                            tx_empty_buf <= TXEMPTYb;
                            tx_state     <= `TX_S2;
                        end
                    endcase
                end
            endcase
        end
    end
////////////////////////////////////////////////////
// Recieve Path
    always@(negedge RTCLK or posedge OPB_RST) begin
        if(OPB_RST) begin
            in_ram_wr_pt_n  <= 8'h0;
            chipEn_rx       <= 1'b0;
            rxen            <= 1'b0;
            rx_state        <= `RX_S0;
            glitch_count    <= 32'h0;
            rxreset         <= 1'b0;
            rx_empty_buf    <= 1'b0;
            reset_cntr_rx   <= 4'h0;
            in_ram_wr_en    <= 1'b0;
        end
        else begin
            case(fo_control_rx)
                `FO_WAIT: begin
                    rx_state <= `RX_S0;             
                end
                `FO_RX: begin
                    case(rx_state)
                        `RX_S0: begin
                            in_ram_wr_pt_n  <= 8'h0;
                            chipEn_rx       <= 1'b1;
                            rxen            <= 1'b0;
                            rx_state        <= `RX_S1;
                            glitch_count    <= 32'h0;
                        end
                        `RX_S1: begin
                            if(RXEMPTYb) begin
                                rxen     <= 1'b1;
                                rx_state <= `RX_S2;
                            end
                            else begin
                                chipEn_rx <= 1'b0;
                                rx_state  <= `RX_S4;
                            end
                        end
                        `RX_S2: begin
                            if(RXEMPTYb) begin
                                in_ram_wr_en <= 1'b1;
                                rx_state     <= `RX_S3;
                            end
                            else begin
                                chipEn_rx <= 1'b0;
                                rxen     <= 1'b0;
                                rx_state <= `RX_S4;
                            end
                        end
                        `RX_S3: begin
                            if(RXEMPTYb) begin
                                if((in_ram_buf == 11'h700) || (in_ram_buf == 11'h300))
                                    glitch_count <= glitch_count + 1;
                                else
                                    in_ram_wr_pt_n <= in_ram_wr_pt + 1;
                            end
                            else begin
                                in_ram_wr_pt_n  <= in_ram_wr_pt + 1;
                                chipEn_rx       <= 1'b0;
                                rxen            <= 1'b0;
                                rx_state        <= `RX_S4;
                            end
                        end
                        `RX_S4: begin
                            in_ram_wr_en <= 1'b0;
                        end
                    endcase                     
                end
                `FO_RST_RX: begin
                    case(rx_state)
                        `RX_S0: begin
                            chipEn_rx     <= 1'b1;
                            rxreset       <= 1'b1;
                            reset_cntr_rx <= 4'h0;
                            rx_state      <= `RX_S1;
                        end
                        `RX_S1: begin
                            if(reset_cntr_rx > 4'h8)
                                rx_state <= `RX_S2;
                            else
                                reset_cntr_rx <= reset_cntr_rx + 1;
                        end
                        `RX_S2: begin
                            chipEn_rx   <= 1'b0;
                            rxreset     <= 1'b0;
                        end
                    endcase
                end
                `FO_RD_EMPTY_RX: begin
                    case(rx_state)
                        `RX_S0: begin
                            chipEn_rx   <= 1'b1;
                            rx_state    <= `RX_S1;
                        end
                        `RX_S1: begin
                            chipEn_rx    <= 1'b0;
                            rx_empty_buf <= RXEMPTYb;
                            rx_state     <= `RX_S2;
                        end
                    endcase
                end
            endcase
        end
    end
    
    always@(posedge RTCLK) begin
        in_ram_wr_pt <= in_ram_wr_pt_n;
    end
        
////////////////////////////////////////////////////
/* Read Access */
    assign OPB_DO = (in_ram_re)                                 ? {21'b0 , in_ram_do}       : 32'bz;
    assign OPB_DO = (out_ram_re)                                ? {23'b0 , out_ram_do}      : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `DEV_SELECT_ADDR))  ? {31'b0 , device_select}   : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `CNTRL_TX_ADDR))    ? {24'b0 , fo_control_tx}   : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `CNTRL_RX_ADDR))    ? {24'b0 , fo_control_rx}   : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TX_SIZE_ADDR))     ? {24'b0 , tx_size}         : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `REF_DIV_ADDR))     ? {16'b0 , refclk_divider}  : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `RT_DIV_ADDR))      ? {16'b0 , rtclk_divider}   : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TX_EMPTY_ADDR))    ? {31'b0 , tx_empty_buf}    : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `RX_EMPTY_ADDR))    ? {31'b0 , rx_empty_buf}    : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TX_STATE_ADDR))    ? {24'b0 , tx_state}        : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `RX_STATE_ADDR))    ? {24'b0 , rx_state}        : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TX_RST_CNT_ADDR))  ? {28'b0 , reset_cntr}      : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `RX_RST_CNT_ADDR))  ? {28'b0 , reset_cntr_rx}   : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `GLITCH_CNT_ADDR))  ? glitch_count              : 32'bz;
/* Write Access */
    always@(negedge OPB_CLK or posedge OPB_RST) begin
        if(OPB_RST) begin
             device_select  <= 1'b0;            // 0 selects IF 1, 1 selects IF 2   
             fo_control_tx  <= 8'h0;
             fo_control_rx  <= 8'h0;    
             tx_size        <= 8'h1;    
             refclk_divider <= 16'h1;
             rtclk_divider  <= 16'h2;
        end     
        else if(OPB_WE) begin
            case(OPB_ADDR)
                `DEV_SELECT_ADDR:   device_select   <= OPB_DI[0];
                `CNTRL_TX_ADDR:     fo_control_tx   <= OPB_DI[7:0];
                `CNTRL_RX_ADDR:     fo_control_rx   <= OPB_DI[7:0];
                `TX_SIZE_ADDR:      tx_size         <= OPB_DI[7:0];
                `REF_DIV_ADDR:      refclk_divider  <= OPB_DI[15:0];
                `RT_DIV_ADDR:       rtclk_divider   <= OPB_DI[15:0];                
            endcase
        end
    end
    
endmodule
