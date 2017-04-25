#ifndef SMA_H
#define SMA_H

#include <deque>

class SMA {
public:
  SMA(int a = 10)
    : buff_size(a)
  {
    counta = 0 ;
    for ( int i = 0 ; i < buff_size ; i++ ){
      buff.push_back(0);
    }
  }
  float get() ;
  void add(float _data) ;

private:
  int buff_size ;
  int counta ;
  std::deque<float> buff ;
  float sumup ;
};

#endif
