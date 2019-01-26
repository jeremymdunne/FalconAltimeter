#ifndef COMMUNICATION_TAGS_H_
#define COMMUNICATION_TAGS_H_

#define SERIAL_COMMUNICATION_START_CHAR '{'
#define SERIAL_COMMUNICATION_END_CHAR '}'

#define SERIAL_COMMUNICATION_ERROR_MISSING_DELIMINATOR_CHARS          -2
#define SERIAL_COMMUNICATION_ERROR_MESSAGE_TOO_SMALL                  -3
#define SERIAL_COMMUNICATION_ERROR_CHECKSUM_FAILURE                   -4
#define SERIAL_COMMUNICATION_VALIDATION_NO_CHECKSUM                   1
#define SERIAL_COMMUNICATION_VALIDATION_WITH_CHECKSUM                 2


static int computeChecksum(String *message, uint start = 0, uint stop = 0){
  int check = 0;
  if(stop == 0) stop = message->length();
  for(uint i = start; i < stop; i ++){
    check = check ^ message->charAt(i);
  }
  return check;
}

/*
Helper function to handle string to hex on checksums
*/
static int stringToHex(String *message, uint start = 0, uint stop = 0){
  int value = 0;
  int tempValue = 0;
  if(stop == 0) stop = message->length();
  for(uint i = start; i < stop; i ++){
    int asciiVal = message->charAt(i);
    //Serial.println("Ascii: " + String(asciiVal));
    if(asciiVal >= (int)'0' && asciiVal <= (int)'9'){
      //scale it by its value
      tempValue = asciiVal - (int)('0');
    }
    else if(asciiVal >= (int)'A' && asciiVal <= (int)'Z'){
      tempValue = asciiVal - (int)'A' + 10;
    }
    else if(asciiVal >= (int)'a' && asciiVal <= (int)'z'){
      tempValue = asciiVal - (int)'a' + 10;
    }
    else{
      return -1; //error
    }
    //Serial.println("Temp Value: " + String(tempValue));
    //now raise to it's expected value
    //Serial.println((stop-i));
    if(i == stop-1) value += tempValue;
    else {
      value += tempValue * pow(16, stop - i - 1);
    }
  }
  return value;
}

#endif
