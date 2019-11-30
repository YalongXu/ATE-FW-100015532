
#Begin clock constraint
define_clock -name {p:top_100015532TFW|PCI_CLK2} -period 10.992 -clockgroup Autoconstr_clkgroup_0 -rise 0.000 -fall 5.496 -route 0.000 
#End clock constraint

#Begin clock constraint
define_clock -name {n:CLOCK_DIV_pci_16KHz_div|clkout_inferred_clock} -period 10000000.000 -clockgroup Autoconstr_clkgroup_1 -rise 0.000 -fall 5000000.000 -route 0.000 
#End clock constraint

#Begin clock constraint
define_clock -name {p:top_100015532TFW|SYSCLK_IN} -period 1.581 -clockgroup Autoconstr_clkgroup_2 -rise 0.000 -fall 0.790 -route 0.000 
#End clock constraint

#Begin clock constraint
define_clock -name {n:me|clkdiv_inferred_clock[3]} -period 5.105 -clockgroup Autoconstr_clkgroup_3 -rise 0.000 -fall 2.552 -route 0.000 
#End clock constraint

#Begin clock constraint
define_clock -name {n:md|clkdiv_inferred_clock[3]} -period 2.726 -clockgroup Autoconstr_clkgroup_4 -rise 0.000 -fall 1.363 -route 0.000 
#End clock constraint

#Begin clock constraint
define_clock -name {n:md|clk1x_enable_inferred_clock} -period 10000000.000 -clockgroup Autoconstr_clkgroup_5 -rise 0.000 -fall 5000000.000 -route 0.000 
#End clock constraint

#Begin clock constraint
define_clock -name {i:top_100015532TFW|SYSCLK_inferred_clock} -period 433.109 -clockgroup Autoconstr_clkgroup_6 -rise 0.000 -fall 216.554 -route 0.000 
#End clock constraint

#Begin clock constraint
define_clock -name {n:CLOCK_DIV_stat_led2_clk_pre_stat_led2_clk_pre|clkout_inferred_clock} -period 7.671 -clockgroup Autoconstr_clkgroup_7 -rise 0.000 -fall 3.835 -route 0.000 
#End clock constraint

#Begin clock constraint
define_clock -name {n:CLOCK_DIV_stat_led2_clk|clkout_inferred_clock} -period 427.545 -clockgroup Autoconstr_clkgroup_8 -rise 0.000 -fall 213.773 -route 0.000 
#End clock constraint

#Begin clock constraint
define_clock -name {n:ADC_AD7663AS_ad2_mod|ram_wr_clk_inferred_clock} -period 10000000.000 -clockgroup Autoconstr_clkgroup_12 -rise 0.000 -fall 5000000.000 -route 0.000 
#End clock constraint

#Begin clock constraint
define_clock -name {n:CLOCK_DIV_aqclkdiv|clkout_inferred_clock} -period 432.156 -clockgroup Autoconstr_clkgroup_13 -rise 0.000 -fall 216.078 -route 0.000 
#End clock constraint

#Begin clock constraint
define_clock -name {n:ADC_AD7663AS_ad1_mod|ram_wr_clk_inferred_clock} -period 10000000.000 -clockgroup Autoconstr_clkgroup_15 -rise 0.000 -fall 5000000.000 -route 0.000 
#End clock constraint

#Begin clock constraint
define_clock -name {n:Clock16x|sd_clk16x_inferred_clock} -period 428.702 -clockgroup Autoconstr_clkgroup_16 -rise 0.000 -fall 214.351 -route 0.000 
#End clock constraint
