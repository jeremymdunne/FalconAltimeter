#ifndef _FLASH_FAT_H_
#define _FLASH_FAT_H_

#include <Arduino.h>
#include <W25Q64FV.h>

//implement a FAT-like file system
//reserve first 4 kb for the FAT table and general system characteristics
#define FILE_ALLOCATION_TABLE_ADDRESS (1<<12)
#define SYSTEM_INFORMATION_LOCATION_ADDRESS 0
#define STANDARD_TIMEMOUT_MILLIS 800
#define USE_SERIAL_OUTPUT

//errors and general reports
#define NO_FILE_ALLOCATION_TABLE 1
#define NO_AVAILABLE_SPACE -5
#define NO_FILE -6
#define NO_FILE_OPENED -7
#define MAX_ADDRESS_REACHED -8
#define WRONG_MODE -9
#define IDIOT_ALERT -13
#define STORAGE_MEDIUM_TIMEOUT -10
#define STORAGE_MEDIUM_INIT_FAILURE -11


class FlashFAT{
public:
  struct FileAllocationTableEntry{
    ulong startAddress;
    ulong size;
  };

  struct FileAllocationTable{
    uint numFiles;
    FileAllocationTableEntry files[256];
  };

  enum FILE_MODE{
    READ, WRITE, STANDBY
  };
  int init();
  int open(FILE_MODE mode, uint fd = 257);
  int readStrem(byte *buf, uint length);
  int write(byte *buf, uint length);
  int read(byte *buf, uint length);
  int close();
  int getSystemInformation();
  int getFileAllocationTable(FileAllocationTable *target);
  int eraseLastFile();
  ulong getRemainingReadSize();
  int peek();
  int eraseAllFiles();
  int recoveryRead(ulong address, byte *buf, uint length);
private:
  FileAllocationTable table;
  int readFileAllocationTableFromStorage();
  int eraseNextPartOfWriteSector();
  int waitUntilFree(ulong timeoutMills = STANDARD_TIMEMOUT_MILLIS);
  int createFATtable();
  int writeNewFileAllocationTable();
  int makeNewFile();
  FILE_MODE currentMode = STANDBY;
  W25Q64FV flashStorage;
  uint currentFD = 0;
  ulong currentAddress = -1;
  ulong highestErasedAddress = -1;
  byte tempBuffer[256];
  int status;
};
#endif
