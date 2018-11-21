#ifndef _STORAGE_CONTROLLER_H_
#define _STORAGE_CONTROLLER_H_
#include <FLASH_FAT.h>

//file index does not exist
#define STORAGE_CONTROLLER_FILE_DNE -12
//no available space
#define STORAGE_CONTROLLER_SYSTEM_FULL -23
//no files found on the storage
#define STORAGE_CONTROLLER_NO_FILES_AVAILABLE -24
//wrong mode, i.e. trying to read while in write mode
#define STORAGE_CONTROLLER_WRONG_SYSTEM_MODE -25

//controlls the interface between writing and reading data to the storage method
class StorageController{
public:
  enum STORAGE_CONTROLLER_MODE{
    STANDBY, READ, WRITE
  };
  int init(bool initSPI = true);
  int openToWrite();
  int getFileInformation(FILE_ALLOCATION_TABLE_STRUCTURE *fileSystem);
  int openToRead(int fd = -1);
  int write(byte *buf, int n);
  int read(byte *buf, int n);
  int close();
private:
  FLASH_FAT fatSystem;
  FILE_ALLOCATION_TABLE_STRUCTURE tempFileStructure;
  STORAGE_CONTROLLER_MODE currentMode = STANDBY;
  int status;
};


#endif
