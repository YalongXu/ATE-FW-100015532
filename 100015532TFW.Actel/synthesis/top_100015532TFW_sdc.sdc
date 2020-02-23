# Top Level Design Parameters

# Clocks

create_clock -name {SYSCLK_IN} -period 12.500000 -waveform {0.000000 6.250000} SYSCLK_IN
create_clock -name {PCI_CLK2} -period 31.250000 -waveform {0.000000 15.625000} PCI_CLK2
create_clock -name {SD_CLK16x} -period 6.250000 -waveform {0.000000 3.125000} coll_mod/sd_clk16x_inst/Core:GLA
create_clock -name {pci_16kHz} -period 1000.000000 -waveform {0.000000 500.000000} Clocks/pci_16KHz_div/clkout:Q
create_clock -name {aqclk} -period 1000.000000 -waveform {0.000000 500.000000} Clocks/aqclkdiv/clkout:Q
create_clock -name {sclk} -period 50.000000 -waveform {0.000000 25.000000} Clocks/sclk/clkout:Q
create_clock -name {pci_clk_div} -period 1000.000000 -waveform {0.000000 500.000000} Clocks/pci_clk_div/clkout:Q
create_clock -name {stat_led2_clk} -period 1000.000000 -waveform {0.000000 500.000000} Clocks/stat_led2_clk/clkout:Q
create_clock -name {stat_led2_clk_pre} -period 1000.000000 -waveform {0.000000 500.000000} Clocks/stat_led2_clk_pre/clkout:Q

# False Paths Between Clocks


# False Path Constraints


# Maximum Delay Constraints


# Multicycle Constraints


# Virtual Clocks
# Output Load Constraints
# Driving Cell Constraints
# Wire Loads
# set_wire_load_mode top

# Other Constraints
