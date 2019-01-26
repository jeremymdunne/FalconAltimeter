#include <SerialLineBuffer.h>


int SerialLineBuffer::checkAvailibility(){
  if(!usingHardware){
    //Serial.println("Checking" + String(usbTarget->available()));
    //Serial.println("Done");
    return usbTarget->available();
  }
  return target->available();
}

char SerialLineBuffer::readChar(){
  if(!usingHardware){
    return usbTarget->read();
  }
  return target->read();
}

int SerialLineBuffer::init(HardwareSerial *target){
  this->target = target;
  return 0;
}

int SerialLineBuffer::init(USBSerial *usbTarget){
  usingHardware = false;
  this->usbTarget = usbTarget;
  return 0;
}

int SerialLineBuffer::available(){
  return update();
}

int SerialLineBuffer::getLine(char *buff, uint max){
  if(max < writeIndex) return -1;
  for(uint i = 0; i < writeIndex; i ++){
    buff[i] = buffer[i];
  }
  uint temp = writeIndex;
  writeIndex = 0;
  lineAvailable = false;
  return temp;
}

int SerialLineBuffer::update(){
  //read the serial
  if(lineAvailable == true) return writeIndex;
  //Serial.println("Checking!");
  while(checkAvailibility() > 0){
    //Serial.print("New:");
    char c = readChar();
    Serial.println(c);
    if(c == '\n' || c == '\r'){
      if(writeIndex > 0){
        lineAvailable = true;
        //add a null-termination char
        buffer[writeIndex] = '\0';
        writeIndex ++;
        return writeIndex;
      }
    }
    else{
      buffer[writeIndex] = c;
      writeIndex ++;
    }
  }
  return 0;
}
