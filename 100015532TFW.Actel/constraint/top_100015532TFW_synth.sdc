# Synopsys, Inc. constraint file
# T:/Test Working/KIK2-MSVisualStudio/VMS/100015532/Firmware/100015532TFW.Actel/constraint/top_100015532TFW_synth.sdc
# Written on Fri Apr 29 10:54:13 2011
# by Synplify Pro, E-2010.09A-1 Scope Editor

#
# Collections
#

#
# Clocks
#
define_clock   {p:SYSCLK_IN} -name {SYSCLK_IN}  -freq 80 -clockgroup default_clkgroup_0
define_clock   {n:SYSCLK} -name {SYSCLK}  -freq 40 -clockgroup default_clkgroup_1
define_clock   {p:PCI_CLK2} -name {PCI_CLK2}  -freq 32 -clockgroup default_clkgroup_2
define_clock   {n:coll_mod.sd_clk16x_inst.GLA} -name {SD_CLK16x}  -freq 160 -clockgroup default_clkgroup_3
define_clock   {n:Clocks.pci_16KHz_div.clkout} -name {pci_16kHz}  -freq 1 -clockgroup default_clkgroup_4
define_clock   {n:Clocks.aqclkdiv.clkout} -name {aqclk}  -freq 1 -clockgroup default_clkgroup_5
define_clock   {n:Clocks.sclk.clkout} -name {sclk}  -freq 20 -clockgroup default_clkgroup_6
define_clock   {n:Clocks.pci_clk_div.clkout} -name {pci_clk_div}  -freq 1 -clockgroup default_clkgroup_7
define_clock   {n:Clocks.stat_led2_clk.clkout} -name {stat_led2_clk}  -freq 1 -clockgroup default_clkgroup_8
define_clock   {n:Clocks.stat_led2_clk_pre.clkout} -name {stat_led2_clk_pre}  -freq 1 -clockgroup default_clkgroup_9

#
# Clock to Clock
#

#
# Inputs/Outputs
#

#
# Registers
#

#
# Delay Paths
#

#
# Attributes
#
define_global_attribute  {syn_global_buffers} {012}
define_attribute {n:SYSCLK_IN} {syn_insert_buffer} {CLKINT}
define_attribute {n:PCI_CLK2} {syn_insert_buffer} {CLKINT}
define_attribute {n:PCI_RST} {syn_insert_buffer} {CLKINT}

#
# I/O Standards
#

#
# Compile Points
#

#
# Other
#
