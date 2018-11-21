#include <HostCommunicator.h>


int ComputerCommunication::requestResend(){
  temp = FAILURE;
  handleSend(&temp, ERROR);
  return 0;
}

int ComputerCommunication::readLine(String *msg){
  if(recievedMessagesIndex == 0) return -1;
  *msg = recievedMessages[recievedMessagesIndex-1];
  recievedMessagesIndex --;
  return 0;
}

int ComputerCommunication::available(){
  //go and see if any are available
  if(computerBuffer.available() > 0){
    //go and grab it
    computerBuffer.readLine(&temp);
    //first make sure it has the appropriate symbol
    if(temp.indexOf(HOST_START_CHAR) < 0){
      String error = "NO START CHAR";
      sendData(&error, VERBOSE);
      requestResend();
      return recievedMessagesIndex;
    }
    temp = temp.substring(temp.indexOf(HOST_START_CHAR) + 1);
    Serial.println("Recieved message ; " + temp);
    //if it has a checksum, check it
    int checkSum = -1;
    if(temp.indexOf("*") >= 0){
      //compute the checksum and ask for a resend if necessary
      checkSum = parseCheckSumFromMessage(&temp, true);
      //compare to the computed checksum
      if(computeChecksum(temp) != checkSum){
        //request resed
        String error = "CHECK SUM MISMATCH. Expected: " + String(computeChecksum(temp)) + " from message: " + temp;
        sendData(&error, VERBOSE);
        requestResend();

        return recievedMessagesIndex;
      }
      else{
        recievedMessages[recievedMessagesIndex] = temp;
        recievedMessagesIndex ++;
        return recievedMessagesIndex;
      }
    }
    else{
      recievedMessages[recievedMessagesIndex] = temp;
      recievedMessagesIndex ++;
      return recievedMessagesIndex;
    }
  }
  return recievedMessagesIndex;
}

int ComputerCommunication::parseCheckSumFromMessage(String *msg, bool modifyString){
  int sum = -1;
  if(msg->indexOf("*") >= 0){
    int start = msg->indexOf("*") + 1;
    int end = msg->length();
    sum = atoi(msg->substring(start, end).c_str());
    if(modifyString) *msg = msg->substring(0, start-1);
  }
  return sum;
}


int ComputerCommunication::computeChecksum(String message){
  int check = 0;
  for(uint i = 0; i < message.length(); i ++){
    check = check ^ message.charAt(i);
  }
  return check;
}


int ComputerCommunication::sendData(String *msg, Communication_Type type){
  //encode in a standard start char, checkSum, endChar
  switch(type){
    case(VERBOSE):
      //no checkSome,#initiator
      temp = START_VERBOSE_CHAR + *msg + "*";
      break;
    case(STATEMENT):
      temp = START_STATEMENT_CHAR + *msg + "*" + String(computeChecksum(*msg));
      break;
    case(ERROR):
      temp = SEND_ERROR + *msg + "*" + String(computeChecksum(*msg));
      break;
  }
  return handleSend(&temp, type);
}
int ComputerCommunication::handleSend(String *msg, Communication_Type type){
  //make sure there's a newline char
  if(msg->charAt(msg->length()-1) != '\n') *msg += '\n';
  //clear the buffer
  //while(computerBuffer.available()) computerBuffer.readLine(&temp);
  //String newMessage = START_CHAR + *message + "*" + String(computeChecksum(*message)) + "\n";
  //go and send the message and wait for an ack

  //check if we need to await an ack
  if((type == STATEMENT) || (type == ERROR)){
    int attempt = 0;
    long timeStart = millis();
    while((attempt < NUMBER_OF_ATTEMPTS) & (millis()-timeStart < COMPUTER_COMMUNICATION_TIMEOUT)){
      Serial.print(*msg);
      while(computerBuffer.available() <= 0 && (millis() - timeStart < COMPUTER_COMMUNICATION_TIMEOUT));
      if(computerBuffer.available() <= 0) break;
      computerBuffer.readLine(&temp);
      if(temp.indexOf(ACKNOWLEDGE) >= 0) return 0;
    }
    return COMMS_FAILURE;
  }
  else{
    Serial.print(*msg);
    return 0;
  }
  //IDK how you got here....
  return -1;
}

int ComputerCommunication::init(){
  return 0;
}
