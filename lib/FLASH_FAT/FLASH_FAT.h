#ifndef _FLASH_FAT_H_
#define _FLASH_FAT_H_

#include <Arduino.h>
#include <W25Q64FV.h>



//look up table is located on the second 32 kb adder = 4096?
//The first lookup entry tells how many 'files' there are
//Each file has two 24bit values associated with them, a start address and a size
//Warning, size may not always be filled out as this is a volatile system, a check needs
//to be performed every time before allocating a new file location
struct FILE_ENTRY{
  ulong startAddress;
  ulong size;
};
struct FILE_ALLOCATION_TABLE_STRUCTURE{
  uint8 numberOfFiles; //can be used to recovery flight x
  FILE_ENTRY files[256]; //maximum allowed
};

//implement a FAT-like file system
//reserve first 4 kb for the FAT table and general system characteristics
#define FILE_ALLOCATION_TABLE_ADDRESS (1<<12)
#define SYSTEM_INFORMATION_LOCATION_ADDRESS 0

//#define USE_SERIAL_OUTPUT

//errors and general reports
#define NO_FILE_ALLOCATION_TABLE 1
#define NO_AVAILABLE_SPACE -5
#define NO_FILE -6
#define NO_FILE_OPENED -7
#define MAX_ADDRESS_REACHED -8
#define WRONG_MODE -9

class FLASH_FAT{
public:
  enum MODE{
    WRITE,READ,STANDBY
  };
  int init(W25Q64FV *spiFlashChip, bool eraseChip=false);
  int init(int ssPin, bool initSPI=false, bool eraseChip=false);
  int readLookupTableForNumberOfFiles(uint8 *numFiles); //need to call this to allocate the correct size for number of files
  int readLookupTable(FILE_ALLOCATION_TABLE_STRUCTURE *target);
  int findNextAvailableFileLocation();
  int openToWrite();
  int close();
  int write(byte *buf, uint n);
  int openForRead(int fileEntry);
  int readStream(byte *buf, uint maxNumber);
  int eraseAll(bool saveFirstSector = false, bool makeLookupTable = true);
  int eraseLastFile();
  int eraseAllFiles();
  int makeFileAllocationTable();
private:
  bool checkIfAddressHasData(ulong address, int numToCheck = 10);
  int writeNewFileAllocationTable(FILE_ALLOCATION_TABLE_STRUCTURE *newTable);
  int attemptFileRecover(FILE_ALLOCATION_TABLE_STRUCTURE *mostComplete);
  int writeBufferToFlash(ulong address, int n);
  int writeBufferToFlash(ulong address, byte *buf, int n);
  int checkMode(MODE wanted);
  int clearAtLeast32KbMemory(ulong address);
  MODE currentMode;
  ulong currentAddress = -1;
  ulong highestAddressErased = -1;
  uint8 currentFileIndex = -1;
  byte tempBuffer[256]; //used to keep the program's dynamic memory allocation low
  byte writeCacheBuffer[256];
  uint8 writeCacheIndex = 0;
  int status = 0;
  W25Q64FV *flash;
  FILE_ALLOCATION_TABLE_STRUCTURE reference;
};

#endif
