----------------------------------------------------------------------------------
-- Project:    	CureRay530
-- Company: 	CureRay Medical
-- Engineer:	DZ
-- 
-- Module Name:	ad7663.vhd - Simulation 
--
-- Description: 
--
--   AD7663AST.
--
-- Functional Assumptions:
--
--   SDOUT is clocked out by SCLK on the rising edge. (INVSCLK = 0)
--
-- Fault Detection:
--
--   Conversion initiated while Acquiring (t8 = 2.75 ns) or Converting (t7 = 1.25 ns).
--
-- Dependencies: 
-- None
-- Revision: 
-- Revision 1.0 Initial Release
-- Additional Comments: 
--
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_ARITH.all;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity AD7663_DUT is
port (
     VIN           : in std_logic_vector(15 downto 0);
     CNVST_N       : in std_logic;
     SCLK          : in std_logic;
     ENABLE_CHECK  : in std_logic;

     SDOUT         : out std_logic;
     BUSY          : out std_logic
     );

end AD7663_DUT;

architecture BEHAVIORAL of AD7663_DUT is

     signal  ACQUIRING       : std_logic;
     signal  SHIFTER         : std_logic_vector(15 downto 0);
     signal  SAMPLED_SIGNAL  : std_logic_vector(15 downto 0);
     signal  CONVERTING      : std_logic := '0';
     signal  CONVERSION_DONE : std_logic;

begin
     pADC : process (CNVST_N, SCLK, CONVERSION_DONE)
          
     begin
          if falling_edge(CNVST_N) then
               CONVERTING <= '1';
--               if ACQUIRING'stable(2750 ns) and VIN'stable(2750 ns) then
               if ACQUIRING'stable(10 ns) and VIN'stable(10 ns) then					
                    SAMPLED_SIGNAL <= VIN;
               else
                    SAMPLED_SIGNAL <= X"BAD1";
                    if ENABLE_CHECK = '1' then
                         report("Bad sample taken by ADC.") severity warning;
                    end if;
               end if;
          end if;

          if rising_edge(CONVERSION_DONE) then
               CONVERTING <= '0';
               SHIFTER    <= SAMPLED_SIGNAL;
          end if;

          if rising_edge(SCLK) then
               SHIFTER <= SHIFTER(14 downto 0) & '0';
               SDOUT   <= SHIFTER(15);
          end if;
          
     end process;

     ACQUIRING       <= not(CONVERTING);
     CONVERSION_DONE <= transport CONVERTING after 1250 ns;

		BUSY  <= CONVERTING after 30 ns;
	  
end BEHAVIORAL;
