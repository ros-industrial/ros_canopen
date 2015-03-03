#!/usr/bin/env python3

PDOs={
"RPDO1": [1, ('6040',     "control_word",                 2), ('6042',     "target_velocity",              2), ('60C1sub1', "target_interpolated_velocity", 4)],
"RPDO2": [1, ('607A',     "target_position",              4), ('6081',     "profile_velocity",             4)],

"TPDO1": [1, ('6041',     "status_word",                  2), ('6061',     "op_mode_display",              1)],
"TPDO3": [1, ('6064',     "actual_position",              4), ('606C',     "actual_velocity",              4)],
}

if __name__ == "__main__":
    import pdo, sys

    fname = sys.argv[1]
    pdo.patch_all(fname, fname, PDOs)
