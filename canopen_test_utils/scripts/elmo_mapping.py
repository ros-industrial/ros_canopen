#!/usr/bin/env python3
import pdo
import sys

fname = sys.argv[1]
pdo.patch(fname, fname, True,  0, ['6040','60FF'], 1) # control_word #target_profiled_velocity
pdo.patch(fname, fname, True,  2, ['607A','6081'], 1) # target_position profile_velocity
pdo.patch(fname, fname, True,  3, ['60C1sub1','60C1sub2'], 1) # target_interpolated_position  target_interpolated_velocity

pdo.patch(fname, fname, False, 0, ['6041','6061'], 1) # status_word op_mode_display
pdo.patch(fname, fname, False, 2, ['6064','606C'], 1)  # actual_position actual_velocity
