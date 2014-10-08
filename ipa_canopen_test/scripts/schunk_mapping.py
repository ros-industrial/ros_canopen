#!/usr/bin/env python3
import pdo
import sys

fname = sys.argv[1]

pdo.patch(fname, fname, True,  0, ['6040','6042','60C1sub1'], 1) # control_word target_velocity target_interpolated_position
pdo.patch(fname, fname, True,  1, ['607A','6081'], 1) # target_position profile_velocity

pdo.patch(fname, fname, False, 0, ['6041','6061'], 1) # status_word op_mode_display
pdo.patch(fname, fname, False, 2, ['6064','606C'], 1)  # actual_position actual_velocity