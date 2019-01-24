#ifndef _CIRCULAR_BUFFER_INDEXER_H_
#define _CIRCULAR_BUFFER_INDEXER_H_

#include <Arduino.h>
class CircularBufferIndexer{
public:
  CircularBufferIndexer(uint size);
  CircularBufferIndexer(){}
  void init(uint size);
  int add(); //returns the index to place the member in
  void removeLast();
  int readFromStart(uint *buffer);
  int readFromEnd(uint *buffer);
  void clear();
private:
  uint counter; //points at the next member to fill
  uint minSizeCounter; //this is used if the available buffer is less than size
  uint size = 0;
};


#endif
