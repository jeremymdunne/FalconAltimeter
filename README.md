Falcon Altimeter V.0.2


Main objective: Record flight information for data recall, control up to two pyros and
all associated safety requirements  

Secondary Objective: Easily modifiable to add 'modules' and extra data recall for other
systems that may be included. First and foremost, an airbrake controlling module

Tertiary Objective: Real time telemetry, overrides, and all associated controls


Hardware: stm32 f1 processor for speed and program sizing requirements. IMU, barometer,
and gps for data collection. Using the mpu9250, bmp280, and ublox gps respectively for
these components. Data recall through the use of a spi-flash chip for non-volatile data

Adding methods of controlling pyros through external battery sources


Software: Tight loop controlled program is necessary for this. The main idea is to
build the state of the rocket through sensors and algorithms, give this information
to any modules, and then lastly to record the information at pre-determined rates.
Data is also collected while waiting for launch detection, at which only recent data
will be stored. Data recall is handled through serial, with the necessary control
transferred to this once detected (no computer will be hooked up to the controller in a launch...).  
