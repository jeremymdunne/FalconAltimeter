��#   F a l c o n   A l t i m e t e r  
 

TODO List:
Flight Routine
User Interrupt
User Resume
GPS
Flight Estimation


To Fix List:
Erase Confirmation
Fix FLASH_FAT
File System Improvements


Modular system for a simple rocket O.S.
The idea is to have main modules which control individual systems
are routed through the main program. Information is conveyed through request
flags, typically conveyed in the module's update() function. This allows the
main controller to decide the severity of the request and act appropriately
in any and all circumstances, as to not overload the system. Typically, high
priority will be given to certain modules, such as the sensor package,
stabilization routines, etc., which must have fast update speeds in all
circumstances.

Also, with a modular system, it should be relatively easy to add new modules,
like active altitude tracking, communications, etc. without needing to remake
the entire system. 
