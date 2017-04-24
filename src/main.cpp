#include "mbed.h"
#include "MPU6500.h"
#include <math.h>

#define NGBATT 3.6
#define MOTOR_RESISTOR 4.5
#define KE 0.0000748 //torque constant
#define KT 0.00064 //back electoromotive
#define GEAR_RATIO 4.0
#define VCC 3.29 //mcu power voltage
#define TREAD 0.037
#define INERTIA 1.0 //Moment of inertia
#define WHEEL_LOSS_R 0
#define WHEEL_LOSS_L 0
#define MASS 0.015
#define TIRE_R 0.05

// Inhelit Class
AnalogIn voltage_batt (PA_4);
Serial pc (PA_2, PA_3);
AnalogIn ltr4206[] = { PC_3, PC_2, PC_1, PC_0 };
DigitalOut osi5[] = { PB_6, PB_7, PB_8, PB_9 };
SPI mpu6500 ( PC_12, PC_11, PC_10 );
SPI AS5047P ( PA_7, PA_6, PA_5 );
DigitalOut cs1 (PC_4);
DigitalOut cs2 (PC_5);
DigitalOut leds[] = { PB_4 , PB_5 };
PwmOut enable[] = { PC_9 , PC_7 };
DigitalOut phase[] = { PC_8 , PC_6 }; // 0 -- right, 1 -- left
mpu6500_spi imu(mpu6500, PD_2);
Ticker output_motor;

// constant
const float F_C1 = (MOTOR_RESISTOR*MASS*TIRE_R)/(2.0*KT*GEAR_RATIO) ;
const float F_C2 = (MOTOR_RESISTOR*INERTIA*TIRE_R)/(TREAD*KT*GEAR_RATIO) ;
const float F_C3 = (60.0*GEAR_RATIO*KE)/(2.0*3.14*TIRE_R) ;
const float F_C4[2] = { ( (MOTOR_RESISTOR*TIRE_R*WHEEL_LOSS_R) / (KT*GEAR_RATIO)) , ( (MOTOR_RESISTOR*TIRE_R*WHEEL_LOSS_L) / (KT*GEAR_RATIO)) } ;

// fuction
void init();
float get_photovalue(int);
float get_Angle(bool);
void feadfoward(float *_duty);
void set_motor_value();

// grobal varience
int mode = 0 ;

// preset data
const int preset_size = 3 ;
float preset[preset_size][3] = { {1, 0, 1} , {0, 0, 1} , {-1, 0, 1} } ; //(accel, omega, time)

int main(){
  init();

  mode = 1 ;
  output_motor.attach(&set_motor_value,0.001);

  while (true) {
    if ( mode == 1 ){
      wait(1.0);
      if (voltage_batt * VCC * 2.0 <= NGBATT ) {
        output_motor.detach();
        mode = 0;
      }
    } else if ( mode == 0 ){
      //waiting task
      leds[1] = 1;
    }
  }
}

void init(){
  int error;

  cs1 = 1; cs2 = 1;
  for ( int i = 0 ; i < 2 ; i++ ){
    leds[i] = 0 ;
    enable[i] = 0 ;
  }
  pc.baud(115200);
  pc.printf("Reset\n\r");

  //MPU6500
  wait(1);
  error = imu.init(1,BITS_DLPF_CFG_5HZ);
  if(error != 0){  //INIT the mpu6000
    pc.printf("Couldn't initialize MPU6000 via SPI!, who am i is %d\n\r", error);
  } else {
    wait(0.1);
    pc.printf("WHOAMI=%u\n\r",imu.whoami()); //output the I2C address to know if SPI is working, it should be 104
    wait(0.1);
    pc.printf("Gyro_scale=%u\n\r",imu.set_gyro_scale(BITS_FS_1000DPS));    //Set full scale range for gyros
    wait(0.1);
    pc.printf("Acc_scale=%u\n\r",imu.set_acc_scale(BITS_FS_4G));          //Set full scale range for accs
    wait(0.1);
  }
  //AS5047P
  AS5047P.format(16, 1);
  AS5047P.frequency(1000000);
}

void set_motor_value(){
  static float duty[2];

  feadfoward(duty);
  for ( int i = 0 ; i < 2 ; i++ ){
    if ( duty[i] < 0 ){
      phase[i] = 1 ;
      enable[i] = fabsf(duty[i]) ;
    } else {
      phase[i] = 0 ;
      enable[i] = duty[i] ;
    }
  }

}

void feadfoward(float *_duty){
  static int past_time = 0;
  static int loadmatrix = 0;
  static float speed = 0.0 ;
  float t_accel, t_omega ;

  float vbat = voltage_batt * VCC * 2.0 ;

  if ( loadmatrix < preset_size ){
    //get preset data
    t_accel = preset[loadmatrix][0];
    t_omega = preset[loadmatrix][1];
    if ( float(past_time/1000) >= preset[loadmatrix][2] ){
      loadmatrix++;
      past_time = 0;
    } else {
      past_time++;
    }
    //calculate speed
    speed = speed + t_accel*0.001;
    //calculate duty
    for ( int i = 0 ; i < 2 ; i++ ) {
      _duty[i] = ( F_C1*t_accel + F_C2*t_omega + F_C3*speed + F_C4[i] ) / vbat ;
    }
  } else {
    _duty[0] = 0 ;
    _duty[1] = 0 ;
  }
}

float get_Angle(bool cs){
  uint16_t read_val;
  float return_val;

  if ( cs == 1 ){
    cs1 = 0 ;
    wait_us(10);
    read_val = AS5047P.write(0xFFFF);
    wait_us(10);
    cs1 = 1;
    wait_us(100);
    return_val = float( read_val & 0x3FFF ) * 360.0 / 16383.0 ;
  } else if ( cs == 2 ){
    cs2 = 0;
    wait_us(10);
    read_val = AS5047P.write(0xFFFF);
    wait_us(10);
    cs2 = 1;
    return_val = float( read_val & 0x3FFF ) * 360.0 / 16383.0 ;
  } else {
    return_val = -1 ;
  }
  return return_val;
}

float get_photovalue(int pin){
  // 0 ... left side, 3 ... right side
  float temp;

  if ( pin < 4 && pin >= 0 ){
    //temp = ltr4206[pin];
    osi5[pin] = 1;
    wait_ms(1);
    temp = ltr4206[pin];// - temp;
    osi5[pin] = 0;
    return temp;
  } else {
    return 0;
  }
}
