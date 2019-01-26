#ifndef SERIAL_LINE_BUFFER_H_
#define SERIAL_LINE_BUFFER_H_

#include <Arduino.h>
#include <HardwareSerial.h>

class SerialLineBuffer{
public:
  int init(HardwareSerial *target);
  int init(USBSerial *usbTarget);
  int update();
  int getLine(char *buff, uint max);
  int available();

private:
  int checkAvailibility();
  char readChar();
  char buffer[512]; //maximum buffer size
  uint writeIndex = 0;
  bool lineAvailable = false;
  HardwareSerial *target;
  bool usingHardware = true;
  USBSerial *usbTarget;
};


#endif
