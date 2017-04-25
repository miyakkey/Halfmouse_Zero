#include "SMA.h"
#include <deque>

float SMA::get(){
  return sumup / (float)buff_size ;
}

void SMA::add(float _data){
  buff.push_back(_data);
  sumup += _data;
  sumup -= buff.front();
  buff.pop_front();
  return ;
}
