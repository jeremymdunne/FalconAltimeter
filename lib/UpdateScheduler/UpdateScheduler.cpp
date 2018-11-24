#include <UpdateScheduler.h>

UpdateScheduler::UpdateScheduler(ulong updateMillis){
  lastUpdate = millis();
  updateMillis = updateMillis;
  isActive = true;
}

void UpdateScheduler::setUpdateFrequency(float frequency){
  updateMillis = (ulong)(1000/frequency + .5);
  lastUpdate = millis();
  isActive = true;
}

void UpdateScheduler::setUpdateMillis(ulong updateMillis){
  updateMillis = updateMillis;
  lastUpdate = millis();
  isActive = true;
}

int UpdateScheduler::update(){
  if(!isActive) return TIME_UPDATER_INACTIVE;
  if(millis() - lastUpdate >= updateMillis){
    lastUpdate = millis();
    return TIME_UPDATER_UPDATE_REQUIRED;
  }
  return TIME_UPDATER_UPDATE_NOT_REQUIRED;
}

void UpdateScheduler::pause(){
  isActive = false;
}

void UpdateScheduler::resume(){
  isActive = true;
}
