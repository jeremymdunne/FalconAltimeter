#include <SerialBuffer.h>


int SerialBuffer::readLine(char *target, uint length){
  if((available() == 1) | lineAvailable){
    target = newestMessage;
    length = strlen(newestMessage); 
    newestMessage = "";
    lineAvailable = false;
    serialBufferIndex = 0;
  }
  return -1;
}

int SerialBuffer::charBufferToString(char *buf, int n, String *message){
  *message = "";
  for(int i = 0; i < n; i ++){
    *message += buf[i];
  }
  return 0;
}

int SerialBuffer::available(){
  //go and read the entire buffer, look for newline
  if(lineAvailable) return 1;
  while(Serial.available()){
    if(Serial.available()){
      tempChar=Serial.read();
      if(tempChar == '\n' || tempChar == '\r'){
        //end of line
        if(serialBufferIndex > 0){
          lineAvailable = true;
          charBufferToString(&serialBuffer[0], serialBufferIndex, &newestMessage);
          return 1;
        }
      }
      else{
        serialBuffer[serialBufferIndex] = tempChar;
        serialBufferIndex ++;
      }
    }
  }
  return 0;
}
