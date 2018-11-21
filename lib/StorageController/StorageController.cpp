#include <StorageController.h>

int StorageController::init(bool initSPI){
  status = fatSystem.init(PA4, initSPI, false);
  return status;
}

int StorageController::openToWrite(){
  //open the next available file, report an error if it exists
  status = fatSystem.openToWrite();
  if(status != 0) return status;
  currentMode = WRITE;
  return 0;
}

int StorageController::getFileInformation(FILE_ALLOCATION_TABLE_STRUCTURE *fileSystem){
  status = fatSystem.readLookupTable(fileSystem);
  if(status != 0) return status;
  return 0;
}

int StorageController::openToRead(int fd){
  if(currentMode == WRITE){
    status = fatSystem.close();
  }
  status = getFileInformation(&tempFileStructure);
  if(fd == -1){
    //open the last file
    if(tempFileStructure.numberOfFiles == 0) return STORAGE_CONTROLLER_NO_FILES_AVAILABLE;
    status = fatSystem.openForRead(tempFileStructure.numberOfFiles-1);
  }
  else{
    if(tempFileStructure.numberOfFiles <= fd) return STORAGE_CONTROLLER_FILE_DNE;
    status = fatSystem.openForRead(fd);
  }
  return status;
}

int StorageController::write(byte *buf, int n){
  //first check mode
  if(currentMode != WRITE){
    //try to recover
    status = close();
    if(status != 0) return status;
    status = openToWrite();
    if(status != 0) return status;
  }
  status = fatSystem.write(buf, n);
  return status;
}

int StorageController::read(byte *buf, int n){
  //check mode
  if(currentMode != READ){
    //complete failure, don't even try
    return STORAGE_CONTROLLER_WRONG_SYSTEM_MODE;
  }
  status = fatSystem.readStream(buf, n);
  return status;
}

int StorageController::close(){
  //close and set the mode to standby
  status = fatSystem.close();
  if(status != 0) return status;
  currentMode = STANDBY;
  return status;
}
