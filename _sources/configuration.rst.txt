Bus Configuration
============================

The ros2_canopen stack relies on a YAML configuration file that is used 
for configuring the bus topology and specifying configurations for
each device. From this configuration file, we generate the device configuration
file (DCF) for the CANopen master as well as concise DCF files for master and
each slave. The ros2_canopen stack uses the YAML configuration file to choose
the correct drivers for each device on the bus and the generated DCF for configuring
the CANopen Master as well as the devices on the bus.

Structure
---------

The YAML configuration file contains a section for each device on the bus. The section
for the master has different configuration options than the section for the slave devices.
The file has the following structure. The master section has to be named master. The
device sections need be named uniquely.
.. code-block:: 

  master:
    [configuration item]: [value]
    [...]
  
  [device_name]:
    [configuration item]: [value]
    [...]


Master Section
--------------
The master section has a number of configuration options. These are not unique to ros2_canopen
but come from the lely core library. Below you find a list of possible configuration items.

.. csv-table:: Master Configuration
  :header-rows: 1
  :class: longtable
  :delim: ;
  :widths: 1 1

  configuration item; description
  node_id; The node-ID (default: 255)
  driver; The fully qualified class name of the master to use.
  package; The ros2 package name in which the master class can be found.
  baudrate; The baudrate in kbit/s (default: 1000)
  vendor_id;The vendor-ID (default: 0x00000000)
  product_code;The product code (default: 0x00000000)
  revision_number;	 The revision number (default: 0x00000000).
  serial_number; 	The serial number (default: 0x00000000).
  heartbeat_multiplier;	The multiplication factor used to obtain the slave heartbeat consumer time from the master heartbeat producer time (default: see options section).
  heartbeat_consumer;	Specifies whether the master should monitor the heartbeats of the slaves (default: true).
  heartbeat_producer;	The heartbeat producer time in ms (default: 0).
  emcy_inhibit_time;	The EMCY inhibit time in multiples of 100 μs (default: 0, see object 1015).
  sync_period;	The SYNC interval in μs (default: 0).
  sync_window;	The SYNC window length in μs (default: 0, see object 1007).
  sync_overflow;	The SYNC counter overflow value (default: 0, see object 1019).
  error_behavior;	A dictionary of error behaviors for different classes or errors (default: {1: 0x00}, see object 1029).
  nmt_inhibit_time;	The NMT inhibit time in multiples of 100 μs (default: 0, see object 102A).
  start;	Specifies whether the master shall switch into the NMT operational state by itself (default: true, see bit 2 in object 1F80).
  start_nodes;	Specifies whether the master shall start the slaves (default: true, see bit 3 in object 1F80).
  start_all_nodes;	Specifies whether the master shall start all nodes simultaneously (default: false, see bit 1 in object 1F80).
  reset_all_nodes;	Specifies whether all slaves shall be reset in case of an error event on a mandatory slave (default: false, see bit 4 in object 1F80).
  stop_all_nodes;	Specifies whether all slaves shall be stopped in case of an error event on a mandatory slave (default: false, see bit 6 in object 1F80).
  boot_time;	The timeout for booting mandatory slaves in ms (default: 0, see object 1F89).

Device Section
--------------
The device configuration enables configuring the characteristics of the connected CANopen
device.

.. csv-table:: Device Configuration
  :header-rows: 1
  :class: longtable
  :delim: ;
  :widths: 1 1

  configuration item; description
  driver; The fully qualified class name of the driver to use.
  package; The ros2 package name in which the driver class can be found.
  enable_lazy_load; A flag that states whether the driver is loaded on start-up.
  dcf;	The filename of the EDS/DCF describing the slave (mandatory).
  dcf_path;	The directory in which the generated .bin file will be available at runtime (default: see options section).
  node_id;	The node-ID (default: 255, can be omitted if specified in the DCF).
  revision_number;	The revision number (default: 0x00000000, can be omitted if specified in the DCF).
  serial_number;	The serial number (default: 0x00000000, can be omitted if specified in the DCF).
  heartbeat_multiplier;	The multiplication factor used to obtain master heartbeat consumer time from the slave heartbeat producer time (default: see options section).
  heartbeat_consumer;	Specifies whether the slave should monitor the heartbeat of the master (default: false).
  heartbeat_producer;	The heartbeat producer time in ms (default: 0).
  error_behavior;	A dictionary of error behaviors for different classes or errors (default: {}, see object 1029).
  rpdo;	The Receive-PDO configuration (see below).
  tpdo;	The Transmit-PDO configuration (see below).
  boot;	Specifies whether the slave will be configured and booted by the master (default: true, see bit 2 in object 1F81).
  mandatory;	Specifies whether the slave is mandatory (default: false, see bit 3 in object 1F81).
  reset_communication;	Specifies whether the NMT reset communication command may be sent to the slave (default: true, see bit 4 in object 1F81).
  software_file;	The name of the file containing the firmware (default: "", see object 1F58).
  software_version;	The expected software version (default: 0x00000000, see object 1F55).
  configuration_file;	The name of the file containing the configuration (default: "<dcf_path>/<name>.bin" (where <name> is the section name), see object 1F22).
  restore_configuration;	The sub-index of object 1011 to be used when restoring the configuration (default: 0x00).
  sdo;	Additional SDO requests to be sent during configuration (see below).


Further references
------------------
The dcfgen documentation gives more details on the usage of the dcfgen tool for generating DCF: https://opensource.lely.com/canopen/docs/dcf-tools/