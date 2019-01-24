#include <CircularBufferIndexer.h>


CircularBufferIndexer::CircularBufferIndexer(uint size){
  this->size = size;
  counter = 0;
  minSizeCounter = 0;
}

void CircularBufferIndexer::init(uint size){
  this->size = size;
  counter = 0;
  minSizeCounter = 0;
}

int CircularBufferIndexer::readFromStart(uint *buffer){
  //read from the start index
  if(minSizeCounter >= size){
    for(uint i = 0; i < size; i ++){
      uint actual = counter + i;
      if(actual >= size) actual -= size;
      buffer[i] = actual;
    }
    return size; //tell them we gots a full buffer
  }
  else{
    //its going to read from 0 to the minSize
    for(uint i = 0; i < minSizeCounter; i ++){
      buffer[i] = i;
    }
    return minSizeCounter;
  }
}

int CircularBufferIndexer::readFromEnd(uint *buffer){
  //read from the start index
  if(minSizeCounter >= size){
    for(uint i = 0; i < size; i ++){
      uint actual = counter - i;
      if(actual < 0) actual += size;
      buffer[i] = actual;
    }
    return size; //tell them we gots a full buffer
  }
  else{
    //its going to read from 0 to the minSize
    for(uint i = minSizeCounter - 1; i >= 0; i --){
      buffer[minSizeCounter-1 - i] = i;
    }
    return minSizeCounter;
  }
}

void CircularBufferIndexer::removeLast(){
  //check bounds
  if(counter - 1 < 0){
    counter = size-1;
  }
  else{
    counter --;
  }
}

int CircularBufferIndexer::add(){
  //do 2* just in case removes are called frequently
  if(minSizeCounter<2*size) minSizeCounter ++;
  //check bounds
  if(counter + 1 > size){
    counter = 0;
    return size;
  }
  else{
    counter ++;
    return counter - 1;
  }
  return -1;
}

void CircularBufferIndexer::clear(){
  counter = 0;
  minSizeCounter = 0;
}
