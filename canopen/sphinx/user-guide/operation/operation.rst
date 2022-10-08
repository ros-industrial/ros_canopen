Operation
=========

The ros2_canopen stack has three different operation modes that should fit for any given operating requirements.
The following operation modes:

* service interface
* managed service interface
* ros2_control system interface


Service interface
""""""""""""""""""
The service interface has a very simple service interface that enables
using the ros2_canopen stack. Drivers used in this mode cannot be managed
drivers (have Lifecycle in their name). The service interface was developed
for low control frequency and debug purposes. It is not thought for high 
freuqency usage.

Managed service interface
""""""""""""""""""""""""""
The managed service interface has the same properties as the service interface
with the exception, that it can only use lifecycle drivers (drivers that have
lifecycle in their name). The managed service interface provides more control
over runtime behaviour and recovery options than the service interface.


ros2_control system interface
""""""""""""""""""""""""""""""
The ros2_control system interface provides access to the ros2_canopen stack
via the ros2_control infrastructure. This interface is thought for high frequency
applications. 
