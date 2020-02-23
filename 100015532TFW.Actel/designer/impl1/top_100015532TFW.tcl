# Created by Microsemi Libero Software 11.9.0.4
# Sun Feb 23 11:31:48 2020

# (OPEN DESIGN)

open_design "top_100015532TFW.adb"

# set default back-annotation base-name
set_defvar "BA_NAME" "top_100015532TFW_ba"
set_defvar "IDE_DESIGNERVIEW_NAME" {Impl1}
set_defvar "IDE_DESIGNERVIEW_COUNT" "1"
set_defvar "IDE_DESIGNERVIEW_REV0" {Impl1}
set_defvar "IDE_DESIGNERVIEW_REVNUM0" "1"
set_defvar "IDE_DESIGNERVIEW_ROOTDIR" {D:\git\ATE-FW-100015532\100015532TFW.Actel\designer}
set_defvar "IDE_DESIGNERVIEW_LASTREV" "1"


layout -timing_driven
report -type "status" {top_100015532TFW_place_and_route_report.txt}
report -type "globalnet" {top_100015532TFW_globalnet_report.txt}
report -type "globalusage" {top_100015532TFW_globalusage_report.txt}
report -type "iobank" {top_100015532TFW_iobank_report.txt}
report -type "pin" -listby "name" {top_100015532TFW_report_pin_byname.txt}
report -type "pin" -listby "number" {top_100015532TFW_report_pin_bynumber.txt}

save_design
