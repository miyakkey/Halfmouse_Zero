#include <math.h>
#include <vector>

#include "mbed.h"
#include "MPU6500.h"
#include "SMA.h"

#define NGBATT 3.6
#define TIMEOUT_PHOTOLED_US 50
#define TONE_MOTOR 0.001

#define MOTOR_RESISTOR 4.5
#define KE 0.0000748 //back electoromotive
#define KT 0.00064 //torque constant
#define GEAR_RATIO 4.0
#define VCC 3.29 //mcu power voltage
#define TREAD 0.037
#define INERTIA 0.00032 //Moment of inertia
#define WHEEL_LOSS_R 0.0403
#define WHEEL_LOSS_L 0.040
#define MASS 0.0156
#define TIRE_R 0.006
#define TIRE_MASS 0.0012

// Inhelit Class
AnalogIn voltage_batt (PA_4);
Serial pc (PA_2, PA_3);
AnalogIn ltr4206[] = { PC_0, PC_1, PC_2, PC_3 };
DigitalOut osi5[] = { PB_6, PB_7, PB_8, PB_9 };
SPI mpu6500 ( PC_12, PC_11, PC_10 );
SPI AS5047P ( PA_7, PA_6, PA_5 );
DigitalOut cs1 (PC_4);
DigitalOut cs2 (PC_5);
DigitalOut leds[] = { PB_4 , PB_5 };
PwmOut enable[] = { PC_9 , PC_7 };
DigitalOut phase[] = { PC_8 , PC_6 }; // 0 -- right, 1 -- left
mpu6500_spi imu(mpu6500, PD_2);
Ticker output_motor_task ;

SMA sma_vbat(3);
//SMA sma_ax(10);
SMA sma_ay(30);
SMA sma_omega(30);
SMA sma_photo[4] = { SMA(3), SMA(3), SMA(3), SMA(3) } ;

// stract
typedef struct {
  float vbat ;
  //float ax ;
  float ay ;
  float omegadot ;
  float speed ;
  float omega ;
  float photo[4] ;
  float offset_ay ;
  float offset_omegadot ;
} sensor_t ;

// Inherit struct
sensor_t sensor;

// constant
// for Fead Foaed
const float F_C1 = ( MOTOR_RESISTOR * ( MASS+(TIRE_MASS*2.0) ) * TIRE_R)/(2.0*KT*GEAR_RATIO) ;
const float F_C2 = ( (MOTOR_RESISTOR*TIRE_R)/(KT*GEAR_RATIO) ) * ( (INERTIA/TREAD)+(TIRE_MASS*TREAD/2.0) ) ;
const float F_C3 = (60.0*GEAR_RATIO*KE)/(2.0*3.14*TIRE_R) ;
const float F_C4[2] = { ( (MOTOR_RESISTOR*TIRE_R*WHEEL_LOSS_R) / (KT*GEAR_RATIO)) , ( (MOTOR_RESISTOR*TIRE_R*WHEEL_LOSS_L) / (KT*GEAR_RATIO)) } ;
// for Fead Back
const float Kp = 0 ;
const float Ki = 0 ;
const float Kd = 0 ;
// Speed etc
const float target_max_omega = 1.0 ;
const float target_max_speed = 0.8 ;
const float target_accel = 3.0 ;
const float taeget_omega_dot = 3.0 ;

// fuction
void init();
//float get_Angle(bool);
void feadfoward(float *_duty, float t_accel, float t_omega_dot, float _speed, float _omega ) ;
float feadback_s (float) ;
float feadback_w (float) ;
void set_sensor_value() ;
void set_motor_value() ;
void deside_offset() ;
void move(float*) ;

// grobal varience
int mode = 0 ;
float logdata[1500][2] ;

// preset data
//const int preset_size = 3 ;
//float preset[preset_size][3] = { { 0, 2.0, 250 } , { 0, 0, 250 } , { 0, -5.0, 250 } } ; //(accel, omega, time_ms)
//float preset[preset_size][3] = { { 3.0 , 0.0, 250 } , { 0, 0, 500 } , { -3.0, 0.0, 250 } } ; //(accel, omega, time_ms)
std::vector<float> preset_distance = { 300.0 , 0.0 } ;
std::vector<float> preset_radian = { 0.0 , 0.0 } ;


int main(){
  init();
  wait(0.1) ;
  //deside offset value
  deside_offset();
  for ( int i = 0 ; i < 30 ; i++ ){
    set_sensor_value() ;
  }
  wait(1.0) ;

  mode = 1 ;
  output_motor_task.attach(&set_motor_value, TONE_MOTOR) ;


  while (true) {
    set_sensor_value() ;
    if ( sensor.vbat <= NGBATT ) {
      mode = 0;
    }
    if ( mode == 1 ){
      // runnig task

      //wait(1.0);
    } else if ( mode == 0 ){
      //low power mode
      output_motor_task.detach();
      enable[0] = 0 ; enable[1] = 0;
      leds[0] = 0 ;
      while ( true ){
        leds[1] = !leds[1] ;
        wait(1.0) ;
      }
    } else if ( mode == 2 ){
      //logger and waiting mode
      if ( pc.getc() == 'L' ) {
        pc.printf("\n\r") ;
        for ( unsigned int i = 0 ; i < 1500 ; i++ ){
          pc.printf("%7.4f, %7.4f\n\r",logdata[i][0], logdata[i][1]) ;
        }
      }
      leds[0] = 1 ;
      wait(0.5) ;
    }
    //wait(0.001) ;
  }
}

void init(){
  int error;

  cs1 = 1; cs2 = 1;
  for ( int i = 0 ; i < 2 ; i++ ){
    leds[i] = 0 ;
    enable[i].period_us(10);
    enable[i] = 0 ;
  }
  sensor.speed = 0 ;
  sensor.omega = 0 ;
  pc.baud(115200);
  pc.printf("Reset\n\r");

  //print const value
  pc.printf("-----Feed Forward Constant Value-----\n\r") ;
  pc.printf("C1 = %f \n\r", F_C1 ) ;
  pc.printf("C2 = %f \n\r", F_C2 ) ;
  pc.printf("C3 = %f \n\r", F_C3 ) ;
  pc.printf("C4 = %f, %f \n\r", F_C4[0], F_C4[1] ) ;
  //Sensor test
  pc.printf("-----Sensor Test-----\n\r") ;
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

void deside_offset(){
  float mpu_val[3] ;
  float sumup_a = 0;
  float sumup_o = 0;

  for ( int i = 0 ; i < 100 ; i++ ){
    imu.read(mpu_val);
    sumup_a = sumup_a + mpu_val[0] ;
    sumup_o = sumup_o + mpu_val[2] ;
    wait_us(10) ;
  }
  sensor.offset_ay = sumup_a / 100.0 * -1.0 ;
  sensor.offset_omegadot = sumup_o /100.0 ;
  return ;
}

void set_sensor_value(){
  float mpu_val[3] ;
  static int photo_sw = 0 ;
  float photo_temp_value[2] = { 0, 0 } ;

  //read sensor value and add it to Simple Moving Average
  sma_vbat.add(voltage_batt * VCC * 2.0 );
  imu.read(mpu_val);
  sma_ay.add(mpu_val[0] * -1.0 - sensor.offset_ay ) ;
  sma_omega.add(mpu_val[2] - sensor.offset_omegadot ) ;

  //get value from SMA and push sensor kouzoutai
  sensor.vbat = sma_vbat.get() ;
  sensor.ay = sma_ay.get();
  sensor.omegadot = sma_omega.get();
  sensor.speed = sensor.speed + sensor.ay * TONE_MOTOR ;
  sensor.omega = sensor.omega + sensor.omegadot * TONE_MOTOR ;

  //get value from photo transistor
  if ( photo_sw == 0 ){
    photo_temp_value[0] = ltr4206[0].read();
    photo_temp_value[1] = ltr4206[2].read();
    osi5[0] = 1 ; osi5[2] = 1 ;
  } else if ( photo_sw == 1 ){
    photo_temp_value[0] = ltr4206[1].read();
    photo_temp_value[1] = ltr4206[3].read();
    osi5[1] = 1 ; osi5[3] = 1 ;
  }
  wait_us(50) ;
  if ( photo_sw == 0 ){
    sma_photo[0].add(ltr4206[0].read() - photo_temp_value[0]) ;
    sma_photo[2].add(ltr4206[2].read() - photo_temp_value[1]) ;
    osi5[0] = 0; osi5[2] = 0;
    photo_sw = 1 ;
    sensor.photo[0] = sma_photo[0].get();
    sensor.photo[2] = sma_photo[2].get();
  } else if ( photo_sw == 1 ) {
    sma_photo[1].add(ltr4206[1].read() - photo_temp_value[0]) ;
    sma_photo[3].add(ltr4206[3].read() - photo_temp_value[1]) ;
    osi5[1] = 0; osi5[3] = 0;
    photo_sw = 0 ;
    sensor.photo[1] = sma_photo[1].get();
    sensor.photo[3] = sma_photo[3].get();
  }
}

void set_motor_value(){
  float duty[2];
  //static int past_time = 0;
  static int load_location = 0 ;
  static float mouse_distance = 0 ;
  static float mouse_radian = 0 ;
  static float mouse_speed = 0 ;
  static float mouse_omega = 0 ;
  static float load_distance = preset_distance[0] ;
  static float load_radian = preset_radian[0] ;
  float speed, omega ;
  //Loger
  static int i = 0 ;

  //decide accel
  //calcurate mouse distance
  mouse_distance = mouse_distance + mouse_speed + TONE_MOTOR ;
  //load distance ( if need )
  if ( mouse_distance >= load_distance ) {
    if ( load_location < preset_distance.size() ){
      load_distance = preset_distance[load_location] ;
      mouse_distance = 0.0 ;
      load_location++;
    } else {
      // mode 1 -> 2
      mode = 2 ;
      duty[0] = 0.0 ; duty[1] = 0.0 ;
      move(duty);
      return ;
    }
  }
  //calcurate ideal accel
  if ( mouse_distance < load_distance ) {
    if ( mouse_speed < target_max_speed ) {
      speed = mouse_speed + target_accel * TONE_MOTOR ;
    } else {
      speed = target_max_speed ;
    }
  } else {
    if ( mouse_speed > 0 ){
      speed = mouse_speed - target_accel * TONE_MOTOR ;
    } else {
      speed = 0 ;
    }
  }
  //add feadback (PID control) and calculate accel
  accel = ( speed + feadback_s(mouse_speed) - mouse_speed ) / TONE_MOTOR ;
  //keep speed value
  //mouse_speed = speed ;

  //Attach Fead Foard and deside duty
  feadfoward( duty , accel , omega , speed , omega );
  mouse_speed = speed ;
  move(duty);

  logdata[i][0] = duty[0] ;
  logdata[i][1] = sensor.ay ;
  i++ ;

  return ;
}

void move(float *_duty){
  for ( int i = 0 ; i < 2 ; i++ ){
    if ( duty[i] < 0 ){
      phase[i] = 0 ;
      enable[i] = fabsf(duty[i]) ;
    } else {
      phase[i] = 1 ;
      enable[i] = duty[i] ;
    }
  }
  return ;
}

void feadfoward(float *_duty, float t_accel, float t_omega_dot, float _speed, float _omega ){
  //static float speed = 0.0 ;
  //static float omega = 0.0 ;
  float speed_at_tire[2] ;

  //calculate speed and omega
  //speed = speed + ( t_accel * TONE_MOTOR ) ;
  //omega = omega + ( t_omega_dot * TONE_MOTOR ) ;
  speed_at_tire[0] = _speed + _omega * ( TREAD / 2.0 ) ;
  speed_at_tire[1] = _speed - _omega * ( TREAD / 2.0 ) ;
  //calculate duty
  _duty[0] =  F_C1*t_accel + F_C2*t_omega_dot + F_C3 * ( speed_at_tire[0] ) ;
  _duty[1] =  F_C1*t_accel - F_C2*t_omega_dot + F_C3 * ( speed_at_tire[1] ) ;
  for ( int i = 0 ; i < 2 ; i++ ){
    if ( speed_at_tire[i] >= 0 ){
      _duty[i] = ( _duty[i] + F_C4[i] ) / sensor.vbat ;
    } else {
      _duty[i] = ( _duty[i] - F_C4[i] ) / sensor.vbat ;
    }
  }

  return ;
}

float feadback_s( float theory ){
  static float sumup_i  = 0 ;
  static float old_error = 0 ;
  float error , val ;

  error = theory - sensor.speed ;
  sumup_i = sumup_i + error * TONE_MOTOR ;
  val = ( Kp * error + Ki * sumup_i + Kd * ( error - old_error ) / TONE_MOTOR ) ;
  old_error = error ;
  return val ;
}

float feadback_w( float theory ){
  static float sumup_i  = 0 ;
  static float old_error = 0 ;
  float error , val ;

  error = theory - sensor.omega ;
  sumup_i = sumup_i + error * TONE_MOTOR ;
  val = ( Kp * error + Ki * sumup_i + Kd * ( error - old_error ) / TONE_MOTOR ) ;
  old_error = error ;
  return val ;
}

/*float get_Angle(bool cs){
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
}*/
