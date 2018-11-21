#ifndef _HOST_COMMUNICATOR_H_
#define _HOST_COMMUNICATOR_H_

#define NEW_HOST_COMMAND_AVAILABLE 5


#include <Arduino.h>
#include <SerialBuffer.h>


class HostCommunicator{
public:
  enum Communication_Type{
    VERBOSE, STATEMENT, ERROR
  };
  int init();
  int available();
  int readLine(String *msg);
  //this will encode the string with checksums as appropriate
  int sendData(String *data, Communication_Type type = STATEMENT);
private:
  int parseCheckSumFromMessage(String *msg, bool modifyString = false);
  int handleSend(String *msg, Communication_Type type = STATEMENT);
  int computeChecksum(String message);
  int requestResend();
  SerialBuffer computerBuffer;
  String temp;
  String recievedMessages[8];
  int recievedMessagesIndex = 0;
};

#endif
