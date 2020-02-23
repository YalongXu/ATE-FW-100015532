# ATE-FW-100015532
1. To fix one potential hardware bug: BUSY probably would NOT go from 0 to 1 after AD_CNVST is set as '0' from '1', then shall try it several times (max repeat_times = 6) before exit.
