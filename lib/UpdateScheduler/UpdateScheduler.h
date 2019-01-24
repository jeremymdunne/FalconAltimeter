#ifndef _UPDATE_SCHEDULER_H_
#define _UPDATE_SCHEDULER_H_

#include <Arduino.h>

#define TIME_UPDATER_UPDATE_REQUIRED 1
#define TIME_UPDATER_UPDATE_NOT_REQUIRED 0
#define TIME_UPDATER_INACTIVE -11

class UpdateScheduler{
public:
  UpdateScheduler(ulong updateMillis);
  UpdateScheduler(){};
  void setUpdateMillis(ulong updateMillis);
  void setUpdateFrequency(float frequency);
  int update();
  void pause();
  void resume();
private:
  ulong lastUpdate;
  ulong updateMillis;
  bool isActive = false;
};

#endif
