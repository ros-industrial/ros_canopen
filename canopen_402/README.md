## Overview

This package contains the implementation of the CiA 402 DSP protocol. It
just communicates via objects
([canopen_master](https://index.ros.org/p/canopen_master), no direct CAN
communication is needed.

It runs the 402 state machine and per-mode state machines, if needed. It
does not handle the read of actual positions, velocities, etc..

**It recommended to map the control word (object 6040, 0x60400010), the
status word (object 6041, 0x60410010) and the operation mode display
(object 6061, 0x60610008) to PDOs.**

## Drive operation modes

All standardized drive modes are implemented, these modes can be
substituted in sub classes and others can be added.

The following table lists all supported drive modes and their data
objects. The objects used should be mapped to PDOs.

  -- --------------------------------- -- -------- -- ------------------- -- ---------------------------- -- ---------------- --
     <tablewidth="100%">**Name**      **ID**      **target object**      **RPDO mapping parameter**      comments         
     Profiled Position                    1           607A                   0x607a0020                                       
     Velocity                             2           6042                   ﻿0x60420010                                       
     Profiled Velocity                    3           60FF                   0x60ff0020                                       
     ﻿Profiled Torque                      4           6071                   0x60710010                      not yet tested   
     Interpolated Position                7           60C1sub1               0x60c10120                                       
     Cyclic Synchronous Position          8           607A                   0x607a0020                      ﻿not yet tested   
     ﻿Cyclic Synchronous Velocity          9           60FF                   0x60ff0020                      ﻿not yet tested   
     ﻿Cyclic Synchronous Torque            10          7071                   0x60710010                      not yet tested   
  -- --------------------------------- -- -------- -- ------------------- -- ---------------------------- -- ---------------- --

The modes can be switched at run-time, the 402 state might be switched
in order to switch the mode, if necessary.

Not all devices support all operation modes. The driver limits the set
of available modes based on object 6502.

## Homing

Homing is supported, but is not a available as drive mode. Instead it is
performed at initialization if homing is supported and the homing method
is not zero.

The implemented homing mode just triggers the execution of the selected
homing method. Please refer to your device manual to get a list of
supported methods. Currently the driver aborts the procedure if the
homing was not performed successfully within 60 seconds.

## Tested Devices

The driver was tested for the below listed devices. For each of them,
the supported modes are checked.

*PP: Profiled Position, VL: Velocity, PV: Profiled Velocity, PT:
Profiled Torque, HM: Homing, IP: Interpolated Position, CSP: Cyclic
Synchronous Position, CSV: Cyclic Synchronous Velocity, CST: Cyclic
Synchronous Torque*

*(./) : works, (X) : does not work, - : unsupported, ? : unknown/not
tested*

### SCHUNK

  -- --------------------------------------------------------------------------------------------------------------- -- ------ -- ------ -- ---- -- ---- -- ------ -- ------ -- ----- -- ----- -- ----- --
     <tablewidth="100%">Device Name                                                                                 PP        VL        PV      PT      HM        IP        CSP      CSV      CST   
     [SCHUNK LWA4D](http://mobile.schunk-microsite.com/en/produkte/produkte/dextrous-lightweight-arm-lwa-4d.html)       (./)      (./)      \-      \-      ?         (./)      \-       \-       \-    
     [SCHUNK LWA4P](http://mobile.schunk-microsite.com/en/produkte/produkte/powerball-lightweight-arm-lwa-4p.html)      (./)      (./)      \-      \-      ?         (./)      \-       \-       \-    
     [PG+ 70](http://mobile.schunk-microsite.com/en/produkte/produkte/pg-plus-70-2-finger-parallel-gripper.html)        (./)      (./)      \-      \-      (./)      (./)      \-       \-       \-    
     [PRL](http://www.schunk.com/schunk_files/attachments/OM_AU_PRL__EN.pdf)                                            (./)      (./)      \-      \-      ?         (./)      \-       \-       \-    
  -- --------------------------------------------------------------------------------------------------------------- -- ------ -- ------ -- ---- -- ---- -- ------ -- ------ -- ----- -- ----- -- ----- --

### ELMO

  -- ---------------------------------------------------------------------------------- -- ------ -- ---- -- ------ -- ---- -- ------ -- ------ -- ----- -- ----- -- ----- --
     <tablewidth="100%">Device Name                                                    PP        VL      PV        PT      HM        IP        CSP      CSV      CST   
     [Elmo Whistle Gold](http://www.elmomc.com/products/whistle-servo-drive-gold.htm)      (./)      \-      (./)      ?       (./)      (./)      ?        ?        ?     
  -- ---------------------------------------------------------------------------------- -- ------ -- ---- -- ------ -- ---- -- ------ -- ------ -- ----- -- ----- -- ----- --

### maxon

  -- ------------------------------------------- -- ------ -- ---- -- ------ -- ---- -- ------ -- ---- -- ------ -- ----- -- ----- --
     <tablewidth="100%">Device Name             PP        VL      PV        PT      HM        IP      CSP       CSV      CST   
     [maxon EPOS4](http://epos.maxongroup.com)      (./)      \-      (./)      \-      (./)      \-      (./)      ?        ?     
  -- ------------------------------------------- -- ------ -- ---- -- ------ -- ---- -- ------ -- ---- -- ------ -- ----- -- ----- --
