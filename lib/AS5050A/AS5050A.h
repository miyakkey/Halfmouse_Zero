#ifndef AS5050A_H
#define AS5050A_H

#include "mbed.h"

/*
usage :: init -> read 
*/


class AS5050A {
  SPI& spi ;
  DigitalOut cs ;

public:
  AS5050A(SPI& _spi, PinName _cs) ;
  void init();
  float read();

private:
  uint8_t data_8bit_up, data_8bit_down ;
  uint16_t data_16 ;
} ;

#endif
