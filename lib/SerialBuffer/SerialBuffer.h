#ifndef _SERIAL_BUFFER_H_
#define _SERIAL_BUFFER_H_

#include <Arduino.h>
#define SERIAL_BUFFER_MAX_LENGTH 64

class SerialBuffer{
public:
  int available();
  int readLine(String *target);

private:

  int charBufferToString(char *buf, int n, String *message);
  char serialBuffer[SERIAL_BUFFER_MAX_LENGTH];
  int serialBufferIndex = 0;
  String newestMessage;
  char tempChar;
  bool lineAvailable = false;
};

#endif
