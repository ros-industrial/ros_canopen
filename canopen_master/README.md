## Overview

This packages contains a master implementation of the CANopen DS 301
protocol. It provides libraries to connect to CANopen devices and access
their data objects via high-level interfaces.

## Features

This implementation provides support for many CANopen interfaces and
services:

- High-level object dictionary for all simple types, except booleans
  (1bit)
- EDS/DCF parser
- Automatic configuration based on communication objects
- Full SDO client implementation
- PDO subscriber/publisher for different transmission types
- EMCY handling with diagnostics interfaces
- NMT with heartbeat (client), without node-guarding
- Plug-in system for bus synchronization and the SYNC objects

## CANopen basics

### Node ID

Each device is assigned a node ID in range between 1 and 127. It
determines the CAN frame the device and listens to, so it must be unique
(per bus).

### Object dictionary

All data is stored as objects, which are adressed with 4-digit
hexadecimal numbers and an optional hexadecimal 8-bit sub index. The
object types range from simple integer values to complex structs and
arrays.

The object dictionary contains all objects (data + types) and is
structured based on the index, notable ranges are:

- 1000-1FFF: Communication objects
- 2000-5FFF: Manufacturer-specific objects
- 6000-6FFF: Profile-specific objects (for first logical device)

### Network management (NMT)

The network management includes a state machine for the overall
status(pre-operational, operation, stopped) of the devices and service
to control the states. Further it can be used to monitor the devices.

### Service data objects (SDOs)

Client/server interface for reading and writing objects. SDO cannot be
used with stopped devices.

### Process data objects (PDOs)

Publisher/subscriber interface for reading and writing objects. Limited
to 8 bytes per PDO, can contain multiple (simple) objects. Not all
objects can be mapped to PDOS, this depends on the object and the
device. PDOs are available in the operational state only.

### Synchronization object (SYNC)

Periodically object sent by the master to synchronize the devices of one
bus. PDOs might be set-up to be synchronous (recommended for
ros_canopen).

### Emergency objects (EMCY)

The nodes send these objects on critical errors. The standard lists a
wide range of errors code, profile can have additional codes.

## EDS/DCF files

The electronic data sheet and the device configuration files are
standardized in CiA 306 DS. These are INI files that contain all objects
descriptions. The new XML based XDD is currently not supported.

The EDS just contains the defaults, the DCF can include specific
parameter values, however canopen_master treats both alike. All entries
that have parameter values different from the defaults are written to
the device at initialization. PDO objects have additional restrictions
and are handled separately.

All manufactures should provide EDS or DCF files. The editor Vector
CANeds 3.6 is available freely and plays well with wine. The 3.7 version
is free as well, but bundled to the demos version of the big CAN
software suites. However, it can extract the object dictionary from any
CANopen device.

## Master plug-ins

Each CANopen bus shall have up to one master that is responsible for the
synchronization. The driver has preliminary support for inter-process
communitcation which should allow to run two driver instances on one
bus. This implementation is currently not ready for production use
however.

For now 2 different implementations exist with allocator plug-ins:

- `canopen::SimpleMaster::Allocator`: wait-based implementation, only
  for one process, uses half the time for reading form the bus and half
  the time for updating an writing, **this is the default**
- `canopen::ExternalMaster::Allocator`: Synchronize to external SYNC
  message, uses half the time for reading form the bus and half the time
  for updating an writing, might be used in conjunction with
  canopen_sync_node

3 additional IPC-based implementations exist, but got deprecated:

- `canopen::SharedMaster::Allocator`: IPC-based master, uses shared
  memory
- `canopen::UnrestrictedMaster::Allocator`: like the shared master,
  but the shared memory is not restricted to a certain user
- `canopen::LocalMaster::Allocator`: IPC-based implementation, only
  for one process, without shared memory
