#include <FlashFAT.h>

int FlashFAT::eraseAllFiles(){
  //just remove their FAT entries then write the table again
  table.numFiles = 0;
  status = waitUntilFree();
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("erase all timout error: " + String(status));
    #endif
    return status;
  }
  return writeNewFileAllocationTable();
}

int FlashFAT::write(byte *buf, uint length){
  if(currentMode != WRITE){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Attempting write when in mode: " + String(currentMode));
    #endif
    return WRONG_MODE;
  }
  status = waitUntilFree();
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("write timout error: " + String(status));
    #endif
    return status;
  }
  //check my highest erase
  if(currentAddress + length >= highestErasedAddress){
    eraseNextPartOfWriteSector();
  }
  status = waitUntilFree();
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("write sector erase timout error: " + String(status));
    #endif
    return status;
  }
  //the next problem to solve: if we exceed the 256 B page, then we'll need to move to the next page
  uint spaceUntilEndOfPage = (((currentAddress >> 8) + 1) << 8) - currentAddress;
  //Serial.println("Current Address: " + String(currentAddress));
  //Serial.println("Space remaining in page: " + String(spaceUntilEndOfPage));
  if(spaceUntilEndOfPage < length){
    //Serial.println("Writing to jump pages");
    status = flashStorage.write(currentAddress, buf, spaceUntilEndOfPage);
    currentAddress += spaceUntilEndOfPage;
    status = waitUntilFree();
    if(status != 0){
      #ifdef USE_SERIAL_OUTPUT
        Serial.println("write timout error: " + String(status));
      #endif
      return status;
    }
    return write(&buf[spaceUntilEndOfPage],length-spaceUntilEndOfPage);
  }
  status = flashStorage.write(currentAddress, buf, length);
  currentAddress += length;
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("WRITE write failure: " + String(status));
    #endif
    return status;
  }
  return length;
}

int FlashFAT::peek(){
  //return > 1 if more of file is available
  status = getRemainingReadSize();
  if(status > 0) return 1;
  return status;
}

ulong FlashFAT::getRemainingReadSize(){
  if(currentMode != READ){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Remaining size not in read mode error");
    #endif
    return -1;
  }
  return table.files[currentFD].size + table.files[currentFD].startAddress - currentAddress;
}

int FlashFAT::read(byte *buf, uint length){
  if(currentMode != READ){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Attempting READ when in mode: " + String(currentMode));
    #endif
    return WRONG_MODE;
  }
  status = waitUntilFree();
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("write timout error: " + String(status));
    #endif
    return status;
  }
  if(currentAddress + length > table.files[table.numFiles-1].startAddress + table.files[table.numFiles-1].size){
    length = table.files[table.numFiles-1].startAddress + table.files[table.numFiles-1].size - currentAddress;
  }
  status = flashStorage.read(currentAddress, buf, length);
  currentAddress += length;
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Read read failure: " + String(status));
    #endif
    return status;
  }
  return length;
}

int FlashFAT::eraseLastFile(){
  //remove it from the FAT table
  table.numFiles --;
  return writeNewFileAllocationTable();
}

int FlashFAT::close(){
  //close the modes and addresses
  table.files[table.numFiles-1].size = currentAddress - 1 - table.files[table.numFiles-1].startAddress;
  status = waitUntilFree();
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Close timout error: " + String(status));
    #endif
    return status;
  }
  status = writeNewFileAllocationTable();
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Close write table error " + String(status));
    #endif
    return status;
  }
  currentMode = STANDBY;
  currentAddress = -1;
  highestErasedAddress = -1;
  return 0;
}

int FlashFAT::open(FlashFAT::FILE_MODE mode, uint fd){
  if(currentMode != STANDBY){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("OPEN called when something else was open");
    #endif
    close();
  }
  currentMode = mode;
  switch(currentMode){
    case(READ):
      if(fd == 257) fd = table.numFiles-1;
      currentFD = fd;
      if(fd >= table.numFiles || fd < 0){
        #ifdef USE_SERIAL_OUTPUT
          Serial.println("File Descriptor unavailable, no such file!");
        #endif
        return NO_FILE;
      }
      currentAddress = table.files[currentFD].startAddress;
      break;
    case(WRITE):
      status = makeNewFile();
      if(status != 0){
        #ifdef USE_SERIAL_OUTPUT
          Serial.println("Make new file error: " + String(status));
        #endif
        return status;
      }
      currentFD = table.numFiles-1;
      currentAddress = table.files[table.numFiles-1].startAddress;
      //start erasing 4 kb for time purposes
      eraseNextPartOfWriteSector();
      break;
    case(STANDBY):
      //ummmmmm, idk what you want.....
      return IDIOT_ALERT;
      break;
  }
  return 0;
}

int FlashFAT::eraseNextPartOfWriteSector(){
  //go and erase the next 4kb sector for writing
  if(highestErasedAddress < currentAddress) highestErasedAddress = currentAddress;
  status = waitUntilFree();
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Erase next sector timout error: " + String(status));
    #endif
    return status;
  }
  Serial.println("Erasing sector at: " + String(highestErasedAddress));
  flashStorage.eraseSector(highestErasedAddress);
  highestErasedAddress += 1<<12; //add 4kb to the address;
}

int FlashFAT::makeNewFile(){
  //make a new file at the end of the last known file entry
  //reminder, must be a multiple of 4kb for erasing purposes

  if(table.numFiles > 0)
  table.files[table.numFiles].startAddress = (((table.files[(table.numFiles-1)].startAddress + table.files[table.numFiles-1].size) >> 12) + 1) << 12;
  else table.files[table.numFiles].startAddress = ((FILE_ALLOCATION_TABLE_ADDRESS >> 12) + 1) << 12;
  table.numFiles ++;
  //Serial.println("Next available address: " + String(start));
  //Serial.println("Number of files:" + String(reference.numberOfFiles));
  if(table.files[table.numFiles].startAddress >= 8388607L){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("NO space available for new file!");
    #endif
    return NO_AVAILABLE_SPACE;
  }
  //set the size to 0
  table.files[table.numFiles-1].size = 0;
  //write it
  return writeNewFileAllocationTable();
}

int FlashFAT::waitUntilFree(ulong timeoutMillis){
  ulong startMillis = millis();
  while((millis() - startMillis < timeoutMillis) & flashStorage.isBusy());
  if(!flashStorage.isBusy()) return 0;
  return STORAGE_MEDIUM_TIMEOUT;
}

int FlashFAT::getFileAllocationTable(FileAllocationTable *target){
  //copy over the stuff, don't give them the actual memory location
  target->numFiles = table.numFiles;
  for(uint i = 0; i < table.numFiles; i ++){
    target->files[i].startAddress = table.files[i].startAddress;
    target->files[i].size = table.files[i].size;
  }
  return 0;
}

int FlashFAT::init(){
  status = flashStorage.init(PA4,true);
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Storage medium init error: " + String(status));
    #endif
    return STORAGE_MEDIUM_INIT_FAILURE;
  }
  //go try to read the fat table
  status = readFileAllocationTableFromStorage();
  if(status == 0) return 0;
  if(status == NO_FILE_ALLOCATION_TABLE){
    //we need to create one... to follow
  }
  return 0;
}

int FlashFAT::createFATtable(){
  table.numFiles = 0;
  return writeNewFileAllocationTable();
}

int FlashFAT::recoveryRead(ulong address, byte *buf, uint length){
  //don't check modes or nothin, this is typically a user controlled operation
  status = waitUntilFree();
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("write timout error: " + String(status));
    #endif
    return status;
  }
  status = flashStorage.read(address, buf, length);
  return status;
}

int FlashFAT::writeNewFileAllocationTable(){
  //clear location
  status = waitUntilFree();
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Write File Allocation Table timout error: " + String(status));
    #endif
    return status;
  }
  flashStorage.eraseSector(FILE_ALLOCATION_TABLE_ADDRESS);
  //construct the buffer real quick
  tempBuffer[0] = table.numFiles;
  for(uint i = 0; i < table.numFiles; i ++){
    tempBuffer[6*i+1] = (table.files[i].startAddress >> 16) & 0xFF;
    tempBuffer[6*i+2] = table.files[i].startAddress >> 8 & 0xFF;
    tempBuffer[6*i+3] = table.files[i].startAddress >> 0 & 0xFF;
    tempBuffer[6*i+4] = table.files[i].size >> 16 & 0xFF;
    tempBuffer[6*i+5] = table.files[i].size >> 8 & 0xFF;
    tempBuffer[6*i+6] = table.files[i].size >> 0 & 0xFF;
  }
  status = waitUntilFree();
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Write File Allocation Table erase timout error: " + String(status));
    #endif
    return status;
  }
  //write it
  status = flashStorage.write256(FILE_ALLOCATION_TABLE_ADDRESS, &tempBuffer[0]);
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Write File Allocation Table write error: " + String(status));
    #endif
    return status;
  }
  return 0;
}

int FlashFAT::readFileAllocationTableFromStorage(){
  //read the first 2 bytes
  status = waitUntilFree();
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Read allocation table timout error: " + String(status));
    #endif
    return status;
  }

  //if(status != 0) return status;
  status = flashStorage.read(FILE_ALLOCATION_TABLE_ADDRESS, &tempBuffer[0], 256);
  if(status != 0){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("Read allocation table read error: " + String(status));
    #endif
    return status;
  }
  //store that in target
  //if(tempBuffer[0] == 255) makeFileAllocationTable();
  table.numFiles = (tempBuffer[0]);
  if(table.numFiles == 255){
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("No file allocation table found!");
    #endif
    return NO_FILE_ALLOCATION_TABLE;
  }
  #ifdef USE_SERIAL_OUTPUT
    Serial.println("Number of files found in the lookup table: " + String(table.numFiles));
  #endif
  //read the next appropriate number of bytes
  //byte tempStoreBuffer[target->numberOfFiles*6]; //just in case its bigger than 256
  #ifdef USE_SERIAL_OUTPUT
  //go ahead and print out the fat
    Serial.println("FAT table found:");
    for(int r = 0; r < 16; r ++){
      for(int c = 0; c < 16; c ++){
        Serial.print(String(tempBuffer[r*16+c]) + "\t");
      }
      Serial.println();
    }
  #endif
  //flash->read((FILE_ALLOCATION_TABLE_ADDRESS+1), &tempStoreBuffer[0], (target->numberOfFiles*6));
  //go and parse it
  for(uint i = 0; i < table.numFiles; i ++){
    table.files[i].startAddress = (tempBuffer[i*6+1] << 16) | (tempBuffer[i*6+2] << 8) | (tempBuffer[i*6+3]);
    table.files[i].size = (tempBuffer[i*6+4] << 16) | (tempBuffer[i*6+5] << 8) | (tempBuffer[i*6+6]);
    #ifdef USE_SERIAL_OUTPUT
      Serial.println("File found in allocation table: " + String(i) + " Start: " + String(table.files[i].startAddress) + " Size: " + String(table.files[i].size));
    #endif
  }
  return 0;
}
