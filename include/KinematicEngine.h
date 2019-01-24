#ifndef _KINEMATIC_ENGINE_H_
#define _KINEMATIC_ENGINE_H_
/*
This class contains the algorithms used to determine the rockets:
  Velocity
  Acceleration
  Altitude
Through sensor fusion and other algorithms
*/

#include <header.h>

class KinematicEngine{
public:
  int init();
  int update(RawRocketSensorData *newData); 
private:

};

#endif
