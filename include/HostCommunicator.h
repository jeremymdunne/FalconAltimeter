#ifndef _HOST_COMMUNICATOR_H_
#define _HOST_COMMUNICATOR_H_

#define NEW_HOST_COMMAND_AVAILABLE 5
#define ACKNOWLEDGE "ACK"
#define FAILURE "FAIL"

#define START_STATEMENT_CHAR "$"
#define START_VERBOSE_CHAR "#"
#define HOST_START_CHAR "@"
#define NUMBER_OF_ATTEMPTS 5
#define COMPUTER_COMMUNICATION_TIMEOUT 5000
#define COMMS_FAILURE -57
#define SEND_FILE "SF"
#define SEND_FAT_TABLE "ST"
#define WRITE_CONFIGURATION "WC"
#define ERASE_FILE "FE"
#define SEND_CONFIGURATION "SC"
#define SEND_ERROR "ERR:"


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
