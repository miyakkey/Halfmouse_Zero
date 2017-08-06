#include "mbed.h"
#include "AS5050A.h"

AS5050A::AS5050A(SPI& _spi, PinName _cs) : spi(_spi), cs(_cs) {}

void AS5050A::init(){
  cs = 1 ;
  spi.format(8, 1) ;
  spi.frequency(1000000) ;

  wait_us(500);
  AS5050A::read();
}

float AS5050A::read(){
  cs = 0 ;
  wait_us(1) ; // this is not need, maybe...
  data_8bit_up = spi.write(0xff) ;
  data_8bit_down = spi.write(0xff) ;
  //There may be bug in mbed spi library especially using in 16 bit transmission
  wait_us(1) ;
  cs = 1 ;
  data_16 = data_8bit_up << 8 | data_8bit_down ;
  return float((data_16 << 2) >> 4)/1023.0*360.0 ;
  // the upper two bits (Alarm Lo and Alarm Hi) and the lower two bits (error flag and parity) are not need.
}
