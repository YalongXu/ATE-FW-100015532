--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   10:18:35 03/23/2019
-- Design Name:   
-- Module Name:   D:/FPGA/Rodger_BGM/100015403/10001543_ise/hdl/AD7663AS_tb.vhd
-- Project Name:  top
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: AD7663AS
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
  
ENTITY AD7663AS_tb IS
END AD7663AS_tb;
 
ARCHITECTURE behavior OF AD7663AS_tb IS 
 
    -- Component Declaration for the Unit Under Test (UUT)

    COMPONENT ADC_AD7663AS
    PORT(
         OPB_DO : OUT  std_logic_vector(15 downto 0);
         OPB_DI : IN  std_logic_vector(15 downto 0);
         OPB_ADDR : IN  std_logic_vector(11 downto 0);
         OPB_RE : IN  std_logic;
         OPB_WE : IN  std_logic;
         OPB_CLK : IN  std_logic;
         OPB_RST : IN  std_logic;
         SYSCLK : IN  std_logic;
         AD_CNVST : OUT  std_logic;
         AD_SCLK : OUT  std_logic;
         AD_SDOUT : IN  std_logic;
         AD_BUSY : IN  std_logic;
			debug_status: OUT std_logic_vector(3 downto 0)
        );
    END COMPONENT;
    
	COMPONENT AD7663_DUT
	port (
		 VIN           : in std_logic_vector(15 downto 0);
		 CNVST_N       : in std_logic;
		 SCLK          : in std_logic;
		 ENABLE_CHECK  : in std_logic;

		 SDOUT         : out std_logic;
		 BUSY          : out std_logic
		 );

	end COMPONENT;	

	constant 	CNTRL_ADDR		:	std_logic_vector(11 downto 0)	:=	x"800";
	constant	CLK_DIV_AQ_ADDR	:	std_logic_vector(11 downto 0)	:=	x"802";
	constant	CLK_DIV_SD_ADDR	:	std_logic_vector(11 downto 0)	:=	x"804";
	constant	D_LENGTH_ADDR	:	std_logic_vector(11 downto 0)	:=	x"806";
	constant	STATUS_ADDR		:	std_logic_vector(11 downto 0)	:=	x"808";
	constant	STATE_ADDR		:	std_logic_vector(11 downto 0)	:=	x"80a";
	constant	D_RAM_ADDR		:	std_logic_vector(11 downto 0)	:=	x"000";
	constant	D_RAM_SIZE		:	std_logic_vector(11 downto 0)	:=	x"800";				
	
   --Inputs
   signal OPB_DI : std_logic_vector(15 downto 0) := (others => '0');
   signal OPB_ADDR : std_logic_vector(11 downto 0) := (others => '0');
   signal OPB_RE : std_logic := '0';
   signal OPB_WE : std_logic := '0';
   signal OPB_CLK : std_logic := '0';
   signal OPB_RST : std_logic := '0';
   signal SYSCLK : std_logic := '0';
   signal AD_SDOUT : std_logic := '0';
   signal AD_BUSY : std_logic := '0';
	signal VIN		: std_logic_vector(15 downto 0) := (others => '0');

 	--Outputs
   signal OPB_DO : std_logic_vector(15 downto 0);
   signal AD_CNVST : std_logic := '1';
   signal AD_SCLK : std_logic := '0';
	signal debug_status: std_logic_vector(3 downto 0);

   -- Clock period definitions
   constant OPB_CLK_period : time := 20 ns; --30MHz
   constant SYSCLK_period : time := 10 ns;--100MHz
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: ADC_AD7663AS PORT MAP (
          OPB_DO => OPB_DO,
          OPB_DI => OPB_DI,
          OPB_ADDR => OPB_ADDR,
          OPB_RE => OPB_RE,
          OPB_WE => OPB_WE,
          OPB_CLK => OPB_CLK,
          OPB_RST => OPB_RST,
          SYSCLK => SYSCLK,
          AD_CNVST => AD_CNVST,
          AD_SCLK => AD_SCLK,
          AD_SDOUT => AD_SDOUT,
          AD_BUSY => AD_BUSY,
			 debug_status => debug_status
        );

	uut1: AD7663_DUT PORT MAP(
		VIN			=> 	VIN,
		CNVST_N		=>	 AD_CNVST,	
		SCLK		=>	AD_SCLK,
		ENABLE_CHECK=> '1',
		
		SDOUT		=>	AD_SDOUT,
		BUSY       	=>	AD_BUSY
	);	
		
   -- Clock process definitions
   OPB_CLK_process :process
   begin
		OPB_CLK <= '0';
		wait for OPB_CLK_period/2;
		OPB_CLK <= '1';
		wait for OPB_CLK_period/2;
   end process;
 
   SYSCLK_process :process
   begin
		SYSCLK <= '0';
		wait for SYSCLK_period/2;
		SYSCLK <= '1';
		wait for SYSCLK_period/2;
   end process;


   -- Stimulus process
	stim_proc: process
	begin		
		-- hold reset state for 100 ns.
		wait for 100 ns;	
		OPB_RST	<=	'1';
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';
		OPB_DI	<=	X"0000";
		OPB_ADDR<=	X"000";
		wait for OPB_CLK_period*10;
		OPB_RST	<=	'0';
		wait for OPB_CLK_period*100;

		-- insert stimulus here 
		-- 1. read status regigster
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	STATUS_ADDR;
		wait for OPB_CLK_period*1;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*3;
		-- 2.1 write data length register
		OPB_RE	<=	'0';
		OPB_WE	<=	'1';
		OPB_DI	<=	x"0002";
		OPB_ADDR<=	D_LENGTH_ADDR;
		wait for OPB_CLK_period*1;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*3;
		VIN		<=	x"1111";
		
		-- 2.2 write control register
		OPB_RE	<=	'0';
		OPB_WE	<=	'1';
		OPB_DI	<=	x"0001";
		OPB_ADDR<=	CNTRL_ADDR;
		wait for OPB_CLK_period*1;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';	
		wait for OPB_CLK_period*1000;		
		--wait for 70 us;
		
		-- 3. read status register
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	STATUS_ADDR;
		wait for OPB_CLK_period*3;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;

		-- 4. read adc data register
		-- 4.1. D_RAM_ADDR
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	D_RAM_ADDR;
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;
		
		-- 4.2. D_RAM_ADDR+1
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	x"001";
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;		

--------------------------------------
		-- 5.1 write data length register
		OPB_RE	<=	'0';
		OPB_WE	<=	'1';
		OPB_DI	<=	x"000A";
		OPB_ADDR<=	D_LENGTH_ADDR;
		wait for OPB_CLK_period*1;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*3;
		VIN		<=	x"2222";
		
		-- 5.2 write control register
		OPB_RE	<=	'0';
		OPB_WE	<=	'1';
		OPB_DI	<=	x"0001";
		OPB_ADDR<=	CNTRL_ADDR;
		wait for OPB_CLK_period*1;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*1000;		
--		wait for 70 us;
		
		-- 6. read status register
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	STATUS_ADDR;
		wait for OPB_CLK_period*3;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*1000;

		-- 7. read adc data register
		-- 7.1. D_RAM_ADDR
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	D_RAM_ADDR;
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;
		-- 7.2. D_RAM_ADDR+1
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	x"002";
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;		
		-- 7.2. D_RAM_ADDR+2
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	x"004";
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;	
		-- 7.2. D_RAM_ADDR+3
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	x"006";
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;
		-- 7.2. D_RAM_ADDR+4
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	x"008";
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;	
		-- 7.2. D_RAM_ADDR+5
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	x"00a";
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;	
		-- 7.2. D_RAM_ADDR+6
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	x"00c";
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;	
		-- 7.2. D_RAM_ADDR+7
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	x"00e";
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;	
		-- 7.2. D_RAM_ADDR+8
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	x"010";
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;			
		-- 7.2. D_RAM_ADDR+9
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	x"012";
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;
		
		-- 7.2. D_RAM_ADDR+9
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	x"014";
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;

		-- 7.2. D_RAM_ADDR+9
		OPB_RE	<=	'1';
		OPB_WE	<=	'0';
		OPB_DI	<=	x"0000";
		OPB_ADDR<=	x"016";
		wait for OPB_CLK_period*10;		
		OPB_RE	<=	'0';
		OPB_WE	<=	'0';		
		wait for OPB_CLK_period*10;
		
		wait;
	end process;

END;
