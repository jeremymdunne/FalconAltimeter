#include <FLASH_FAT.h>


int FLASH_FAT::eraseAllFiles(){
  //while there are still files, go and erase them
  //make sure the refernce is updated
  while(flash->isBusy());
  readLookupTable(&reference);
  int numFilesToErase = reference.numberOfFiles;
  for(int i = 0; i < numFilesToErase; i ++){
    status = eraseLastFile();
    if(status != 0) return status;
  }
  return 0;
}

int FLASH_FAT::eraseLastFile(){
  //make sure the refernce is updated
  while(flash->isBusy());
  readLookupTable(&reference);
  //start at the last address + 4kb and work backwards in biggest index
  if(reference.numberOfFiles <= 0) return NO_FILE;
  ulong lastAddress = (((reference.files[reference.numberOfFiles-1].startAddress + reference.files[reference.numberOfFiles-1].size) >> 12) + 1) <<12;
  ulong firstAddress = reference.files[reference.numberOfFiles-1].startAddress;
  Serial.println("First Address: " + String(firstAddress));
  //ooh ooh ask me how I found this one out
  if(lastAddress > (1 << 16)){
    while((lastAddress - (1 << 16)) >= firstAddress){
      //actually wipe the 64kb
      while(flash->isBusy());
      Serial.println("Erasing 64kb block");
      Serial.println("Attempting to erase: " + String(lastAddress - (1<<16)));
      flash->eraseBlock64(lastAddress - (1 << 16));
      lastAddress -= (1<<16);
    }
  }
  if(lastAddress > 1<<15){
    while((lastAddress - (1 << 15)) >= firstAddress){
      //actually wipe the 64kb
      while(flash->isBusy());
      Serial.println("Erasing 32kb block");
      flash->eraseBlock64(lastAddress - (1 << 15));
      lastAddress -= (1<<15);
    }
  }
  while((lastAddress - (1 << 12)) >= firstAddress){
    //actually wipe the 64kb
    while(flash->isBusy());
    Serial.println("Erasing 4kb block");
    flash->eraseBlock64(lastAddress - (1 << 12));
    lastAddress -= (1<<12);
  }
  //go and update the changes
  reference.files[reference.numberOfFiles-1].size = 255|255|255;
  reference.files[reference.numberOfFiles-1].startAddress = 255|255|255;
  reference.numberOfFiles --;
  //go and write the new table
  return writeNewFileAllocationTable(&reference);
}

int FLASH_FAT::init(W25Q64FV *spiFlashChip, bool eraseChip){
  flash = spiFlashChip;
  if(eraseChip){
    eraseAll();
    while(flash->isBusy());
  }
  //read the file allocation table
  while(flash->isBusy());
  status = readLookupTable(&reference);
  if(status == NO_FILE_ALLOCATION_TABLE){
    //make one
    Serial.println("No File allocation table found! Making one!");
    makeFileAllocationTable();
  }
  return 0;
}

int FLASH_FAT::init(int ssPin, bool initSPI, bool eraseChip){
  flash = new W25Q64FV;
  flash->init(ssPin, initSPI);
  flash->waitUntilFree();
  if(eraseChip){
    eraseAll();
  }
  return 0;
}

int FLASH_FAT::eraseAll(bool saveFirstSector, bool makeLookupTable){
  //start a chip erase
  //TODO implement the saveFirstSector
  flash->eraseAll();
  while(flash->isBusy());
  makeFileAllocationTable();
  return 0;
}

int FLASH_FAT::checkMode(MODE wanted){
  //first, check the addresses
  if((currentAddress < 0)||(currentAddress >= 8388607L) || currentFileIndex < 0) return NO_FILE_OPENED;
  if(wanted != currentMode) return WRONG_MODE;
  return 0;
}

int FLASH_FAT::clearAtLeast32KbMemory(ulong address){
  //clear up to 64 kb of memory for writing
  //address dependent
  //first do 4kb wipes until able to do 32, then a 64
  //make sure not running into a max address issue
  ulong tempAddress = address;
  while(((tempAddress + (1<<12)) % (1<<15)) != 0){
    tempAddress += (1<<12);
    //check if it is a valid address
    if(tempAddress >= 8388607L){
      return NO_AVAILABLE_SPACE;
    }
    //wipe the address
    flash->waitUntilFree();
    flash->eraseSector(tempAddress);
    highestAddressErased = tempAddress + (1<<12);
  }
  //clear one 32 kb address
  tempAddress += (1 << 15);
  if(tempAddress >= 8388607L){
    return NO_AVAILABLE_SPACE;
  }
  flash->waitUntilFree();
  flash->eraseBlock32(tempAddress);
  highestAddressErased = tempAddress + (1<<15);
  return 0;
}

int FLASH_FAT::write(byte *buf, uint n){
  // cached write up to 256 bytes
  //first, sanity check we have and open file
  status = checkMode(WRITE);
  if(status != 0) return status;
  //do a safety check on the erase status
  if(currentAddress + n < highestAddressErased){
    status = clearAtLeast32KbMemory(highestAddressErased);
    if(status != 0) return status;
  }
  //the idea is to make a 'stream' of data in 256b buffers
  if(n + writeCacheIndex >= 255){
    //we has a problem!
    //transfer over data to fill up the cache buffer first
    for(int i = writeCacheIndex; i < 256; i ++){
      writeCacheBuffer[i] = buf[i-writeCacheIndex];
    }
    //write that
    writeBufferToFlash(currentAddress, &writeCacheBuffer[0],256);
    currentAddress += 256;
    //take care of the remaining data
    int indexInBuffer = 256 - writeCacheIndex; //points to the next data member in the buff to be written
    //check if you need to keep writing buffers
    while(n-indexInBuffer >= 255){
      //transfer over to the write buffer then write
      for(int i = 0; i < 256; i ++){
        writeCacheBuffer[i] = buf[indexInBuffer];
        indexInBuffer ++;
      }
      //write when safe
      int status = flash->waitUntilFree();
      if(status != 0) return status;
      status = writeBufferToFlash(currentAddress, &writeCacheBuffer[0], 256);
      if(status != 0) return status;
      currentAddress += 256;
    }
    //then chug the remaining into the write cache
    writeCacheIndex = 0;
    for(uint8 i = 0; i < n - indexInBuffer; i ++){
      writeCacheBuffer[i] = buf[indexInBuffer+i];
      writeCacheIndex ++;
    }
  }
  //otherwise, just pump it into the writeCache
  else{
    for(uint8 i = 0; i < n; i ++){
      writeCacheBuffer[writeCacheIndex] = buf[i];
      writeCacheIndex++;
    }
  }
  return 0;
}

int FLASH_FAT::openForRead(int fileEntry){
  //look in the file lookup table for the file address
  if(reference.numberOfFiles <= fileEntry) return NO_FILE;
  currentMode = READ;
  currentFileIndex = fileEntry;
  currentAddress = reference.files[currentFileIndex].startAddress;
  return 0;
}

int FLASH_FAT::readStream(byte *buf, uint maxNumber){

  //check if enough is available in the current file
  int actualSize = maxNumber;
  if(reference.files[currentFileIndex].startAddress + reference.files[currentFileIndex].size - currentAddress < maxNumber){
    actualSize = reference.files[currentFileIndex].startAddress + reference.files[currentFileIndex].size - currentAddress;
  }
  if(actualSize <= 0) return 0;
  //go read up to the actual size length
  while(flash->isBusy());
  status = flash->waitUntilFree();
  if(status != 0) return status;
  flash->read(currentAddress, buf, actualSize);
  currentAddress += actualSize;
  return actualSize;
}

int FLASH_FAT::close(){
  switch(currentMode){
    case(WRITE):
      //write the size to the allocation table
      //write the allocation table
      //change the mode and addresses
      if(writeCacheIndex != 0) writeBufferToFlash(currentAddress, &writeCacheBuffer[0], writeCacheIndex);
      currentAddress += writeCacheIndex;
      reference.files[reference.numberOfFiles-1].size = currentAddress - reference.files[reference.numberOfFiles-1].startAddress;
      //write it
      Serial.println("Closing Size: " + String(reference.files[reference.numberOfFiles-1].size));
      writeNewFileAllocationTable(&reference);
      //change modes, etc.
      currentMode = STANDBY;
      currentAddress = -1;
      highestAddressErased = -1;
      break;
    case(READ):
      //change the mode, change the current address

      break;
    case(STANDBY):
      break;
  }
  return 0;
}

int FLASH_FAT::openToWrite(){
  //yay! add new data member to the FAT table
  //assume our address is the next available 4kb block following the last used spot

  ulong start;
  if(reference.numberOfFiles > 0) start = (((reference.files[reference.numberOfFiles-1].startAddress + reference.files[reference.numberOfFiles-1].size) >> 12) + 1) << 12;
  else start = ((FILE_ALLOCATION_TABLE_ADDRESS >> 12) + 1) << 12;
  Serial.println("Next available address: " + String(start));
  Serial.println("Number of files:" + String(reference.numberOfFiles));
  if(start >= 8388607L) return NO_AVAILABLE_SPACE;
  //add it to the table
  currentMode = WRITE;
  currentAddress = start;
  reference.numberOfFiles ++;
  reference.files[reference.numberOfFiles-1].startAddress = start;
  writeNewFileAllocationTable(&reference);
  //go ahead and clear sectors up to 64 kb
  status = clearAtLeast32KbMemory(start);
  return 0;
}



bool FLASH_FAT::checkIfAddressHasData(ulong address, int numToCheck){
  //no matter what, we should not find 10 255 in a row.
  //look ahead, not behind
  flash->waitUntilFree();
  flash->read(address, &tempBuffer[0], 10);
  for(int i = 0; i < 10; i ++){
    if(tempBuffer[i] != 255){
      return true;
    }
  }
  return false;
}

int FLASH_FAT::attemptFileRecover(FILE_ALLOCATION_TABLE_STRUCTURE *mostComplete){
  //this is called in case the file allocation does not include a size (or the size is 255|255|255)
  //to find the last data member, look for where all the data turns into 255
  //simple binary search
  ulong startAddress = mostComplete->files[mostComplete->numberOfFiles-1].startAddress;
  ulong step = (1<<16);
  ulong testAddress = startAddress + (1 << 16);
  while(checkIfAddressHasData(testAddress)){
    testAddress += step;
  }
  //found a potentially non-memory used location
  step = (1<<15); //32kb
  while(checkIfAddressHasData(testAddress - step)){
    testAddress -= step;
  }
  //getting closer
  step = (1<<8);
  while(checkIfAddressHasData(testAddress - step,5)){
    testAddress -= step;
  }
  //when we're within 256 bytes, just grab all the data
  flash->waitUntilFree();
  flash->read(testAddress - 255, &tempBuffer[0], 256);
  step = (1);
  for(int i = 255; i >= 0; i ++){
    if(tempBuffer[i] != 255){
      //found it!
      mostComplete->files[mostComplete->numberOfFiles-1].size = (testAddress - i) - startAddress;
      return 0;
    }
  }
  return -1;
}

int FLASH_FAT::writeBufferToFlash(ulong address, byte *buf, int n){
  //write the tempBuffer to flash
  //if n != 256, write the remaining values as 255
  //fill out up to n then fill the rest with 255
  for(int i = 0; i < n; i ++){
    tempBuffer[i] = buf[i];
  }
  return writeBufferToFlash(address, n);
}

int FLASH_FAT::writeBufferToFlash(ulong address, int n){
  //write the tempBuffer to flash
  //if n != 256, write the remaining values as 255
  for(int i = n; i < 256; i ++){
    tempBuffer[i] = 255;
  }

  #ifdef USE_SERIAL_OUTPUT
    Serial.println("Writing Buffer to Flash at address " + String(address));
    for(int i = 0; i < 16; i ++){
      for(int p = 0; p < 16; p ++){
        Serial.print(String(tempBuffer[i*16 + p]) + "\t");
      }
      Serial.println();
    }
  #endif
  while(flash->isBusy());
  status = flash->waitUntilFree();
  if(status != 0) return status;
  return flash->write256(address, &tempBuffer[0]);
}

int FLASH_FAT::writeNewFileAllocationTable(FILE_ALLOCATION_TABLE_STRUCTURE *newTable){
  //erase the table and then add the new data
  while(flash->isBusy());
  flash->eraseSector(FILE_ALLOCATION_TABLE_ADDRESS);
  while(flash->isBusy());
  tempBuffer[0] = newTable->numberOfFiles;
  for(int i = 0; i < newTable->numberOfFiles; i ++){
    Serial.println("Writing start address " + String(newTable->files[i].startAddress) + " to FAT");
    //convert the table address values into 24 bit equivalent
    tempBuffer[6*i+1] = (newTable->files[i].startAddress >> 16) & 0xFF;
    tempBuffer[6*i+2] = newTable->files[i].startAddress >> 8 & 0xFF;
    tempBuffer[6*i+3] = newTable->files[i].startAddress >> 0 & 0xFF;
    tempBuffer[6*i+4] = newTable->files[i].size >> 16 & 0xFF;
    tempBuffer[6*i+5] = newTable->files[i].size >> 8 & 0xFF;
    tempBuffer[6*i+6] = newTable->files[i].size >> 0 & 0xFF;
  }
  return writeBufferToFlash(FILE_ALLOCATION_TABLE_ADDRESS, 1+newTable->numberOfFiles*6);
}

//WARNING: BE SURE TO CALL THIS ONLY IF NO TABLE ALREADY EXISTS
int FLASH_FAT::makeFileAllocationTable(){
  //do a sector wipe
  flash->eraseSector(FILE_ALLOCATION_TABLE_ADDRESS);
  //set the number of files indicator to 0
  //update tempBuffer to do it
  tempBuffer[0] = 0;
  for(int i = 1; i < 256; i ++){
    tempBuffer[i] = 255;
  }
  return writeBufferToFlash(FILE_ALLOCATION_TABLE_ADDRESS,256);
}

int FLASH_FAT::readLookupTableForNumberOfFiles(uint8 *numFiles){
  //go grab the first 2 bytes at the lookup file location
  status = flash->read(FILE_ALLOCATION_TABLE_ADDRESS, &tempBuffer[0], 1);
  if(status != 0) return status;
  *numFiles = (tempBuffer[0]);
  //real quick, if numFiles == 255|255, there is no file allocation table, so fix that and report it
  if(*numFiles == 255){
    makeFileAllocationTable();
    *numFiles = 0;
    reference.numberOfFiles = 0;
    return NO_FILE_ALLOCATION_TABLE;
  }
  reference.numberOfFiles = *numFiles;
  return 0;
}

int FLASH_FAT::readLookupTable(FILE_ALLOCATION_TABLE_STRUCTURE *target){
  //read the first 2 bytes
  while(flash->isBusy());
  //if(status != 0) return status;
  status = flash->read(FILE_ALLOCATION_TABLE_ADDRESS, &tempBuffer[0], 256);
  if(status != 0) return status;
  //store that in target
  //if(tempBuffer[0] == 255) makeFileAllocationTable();
  target->numberOfFiles = (tempBuffer[0]);
  Serial.println("Number of files found in the lookup table:" + String(target->numberOfFiles));
  //read the next appropriate number of bytes
  //byte tempStoreBuffer[target->numberOfFiles*6]; //just in case its bigger than 256
  status = flash->waitUntilFree();
  if(status != 0) return status;
  //go ahead and print out the fat
  Serial.println("FAT table found:");
  for(int r = 0; r < 16; r ++){
    for(int c = 0; c < 16; c ++){
      Serial.print(String(tempBuffer[r*16+c]) + "\t");
    }
    Serial.println();
  }
  //flash->read((FILE_ALLOCATION_TABLE_ADDRESS+1), &tempStoreBuffer[0], (target->numberOfFiles*6));
  //go and parse it
  for(int i = 0; i < target->numberOfFiles; i ++){
    target->files[i].startAddress = (tempBuffer[i*6+1] << 16) | (tempBuffer[i*6+2] << 8) | (tempBuffer[i*6+3]);
    target->files[i].size = (tempBuffer[i*6+4] << 16) | (tempBuffer[i*6+5] << 8) | (tempBuffer[i*6+6]);
    Serial.println("File found in allocation table: " + String(i) + " Start: " + String(target->files[i].startAddress) + " Size: " + String(target->files[i].size));
  }
  //check the last file's end address
  if(target->files[target->numberOfFiles-1].size == ((255|255)|255)){
    Serial.println("Suspected corrupted file! attempting recovery!");
    attemptFileRecover(target);
  }
}
