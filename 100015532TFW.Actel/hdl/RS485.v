`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:35:07 09/11/2008 
// Design Name: 
// Module Name:    RS485 
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
`define RS_CNTRL_ADDR       5'h0    // control register address
`define TST_PAT				5'h1	// test pattern address
`define SYNC				5'h2
`define BMPLS				5'h3
`define COLLISION_IN		5'h4
`define COLLSP1_IN			5'h5
`define COLLSP2_IN			5'h6
`define IMTX_IN				5'h7
`define TST_SPI_MISO		5'h8	
`define EATX_IN				5'h9
`define AMTX_IN				5'ha
`define TCTX_IN				5'hb
`define SP4851_IN			5'hc
`define SP4852_IN			5'hd


module RS485(
    output  [31:0]  OPB_DO,
    input   [31:0]  OPB_DI,
    input   [4:0]   OPB_ADDR,
    input           OPB_RE,
    input           OPB_WE,
    input           OPB_CLK,		//32MHz
    input           OPB_RST,
    input           DATACLK,		//80MHz

    input           SYNC,
    input           BMPLS,
    input           COLL_SP1_IN,
    input           COLL_SP2_IN,
    input           IMTX_IN,
    input           TST_SPI_MISO,
    input           EATX_IN,
    input           AMTX_IN,
    input           TCTX_IN,
    input           SP485_1_R,
    input           SP485_2_R,

    output          COLL_CS_OUTb,
    output          COLL_CLK_OUT,
    output          TST_SPI_CLK,
    output          TST_SPI_MOSI,
    output          TST_SPI_CS,
    output          Sp485_1_D,
    output          Sp485_2_D
);

	reg         control;				// set high to trigger transfer
	reg         done;
	reg [7:0]   bit_count;							
	reg [31:0]  test_pattern;		// pattern to transfer
	reg [31:0]  test_pattern_buf;	// output buffer

	reg [31:0]  sync_d;
	reg [31:0]  bmpls_d;
	reg [31:0]  coll_sp1_in_d;
	reg [31:0]  coll_sp2_in_d;	
	reg [31:0]  imtx_in_d;
	reg [31:0]  tst_spi_miso_d;
	reg [31:0]  eatx_in_d;
	reg [31:0]  amtx_in_d;
	reg [31:0]  tctx_in_d;
	reg [31:0]  sp485_1_data;
	reg [31:0]  sp485_2_data;		 

	
	
	assign COLL_CS_OUTb = test_pattern_buf[31];
	assign COLL_CLK_OUT = test_pattern_buf[27];
	assign TST_SPI_CLK  = test_pattern_buf[19];
	assign TST_SPI_MOSI = test_pattern_buf[15];
	assign TST_SPI_CS   = test_pattern_buf[11];
	assign Sp485_1_D    = test_pattern_buf[7];
	assign Sp485_2_D    = test_pattern_buf[3];
		
	always@(posedge DATACLK or posedge OPB_RST) begin
        if(OPB_RST) begin
			sync_d          <= 0;
			bmpls_d         <= 0;
			coll_sp1_in_d   <= 0;
			coll_sp2_in_d   <= 0;
			imtx_in_d       <= 0;
			tst_spi_miso_d  <= 0;
			eatx_in_d       <= 0;
			amtx_in_d       <= 0;
			tctx_in_d       <= 0;
			sp485_1_data    <= 0;
			sp485_2_data    <= 0;
			bit_count       <= 0;
        end
		else if(control && !done) begin
			bit_count       <= bit_count+1;

			sync_d          <= {sync_d[30:0], SYNC};
			bmpls_d         <= {bmpls_d[30:0], BMPLS};
			coll_sp1_in_d   <= {coll_sp1_in_d[30:0], COLL_SP1_IN};
			coll_sp2_in_d   <= {coll_sp2_in_d[30:0], COLL_SP2_IN};
			imtx_in_d       <= {imtx_in_d[30:0], IMTX_IN};
			tst_spi_miso_d  <= {tst_spi_miso_d[30:0], TST_SPI_MISO};
			eatx_in_d       <= {eatx_in_d[30:0], EATX_IN};
			amtx_in_d       <= {amtx_in_d[30:0], AMTX_IN};
			tctx_in_d       <= {tctx_in_d[30:0], TCTX_IN};
			sp485_1_data    <= {sp485_1_data[30:0], SP485_1_R};
			sp485_2_data    <= {sp485_2_data[30:0], SP485_2_R};
		end
		else begin
			bit_count <= 8'h0;
		end
	end
	
	always@(negedge DATACLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			done             <= 1'b0;
			test_pattern_buf <= 0;
		end
		else if(!control) begin
			done             <= 1'b0;
			test_pattern_buf <= test_pattern;
		end
		else if(bit_count > 0) begin
			test_pattern_buf <= {test_pattern_buf[30:0], test_pattern_buf[31]};

			if(bit_count > 31) begin
				done <= 1'b1;				
			end
		end
	end
	
/* Read Access */
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TST_PAT)) 		 ? test_pattern      : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SYNC)) 		 ? sync_d            : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `BMPLS)) 		 ? bmpls_d           : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `COLLSP1_IN)) 	 ? coll_sp1_in_d     : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `COLLSP2_IN)) 	 ? coll_sp2_in_d     : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `IMTX_IN)) 		 ? imtx_in_d         : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TST_SPI_MISO))  ? tst_spi_miso_d    : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `EATX_IN)) 		 ? eatx_in_d         : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `AMTX_IN))	 	 ? amtx_in_d         : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `TCTX_IN)) 		 ? tctx_in_d         : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SP4851_IN)) 	 ? sp485_1_data      : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `SP4852_IN)) 	 ? sp485_2_data      : 32'bz;
    assign OPB_DO = (OPB_RE && (OPB_ADDR == `RS_CNTRL_ADDR)) ? {31'b0 , control} : 32'bz;
			
/* Write Access */
	always@(negedge OPB_CLK or posedge OPB_RST) begin
		if(OPB_RST) begin
			control      <= 1'b0;
			test_pattern <= 32'haf654321;
		end		
		else if(OPB_WE) begin
			case(OPB_ADDR)
				`RS_CNTRL_ADDR: control      <= OPB_DI[0];
				`TST_PAT:       test_pattern <= OPB_DI;
			endcase
		end	
	end

endmodule