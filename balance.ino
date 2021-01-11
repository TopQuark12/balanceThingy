#include <Wire.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include "MPU6050.h"
#include "pid.h"
#include "Quaternion.hpp"

#define CALIBRATION_GYRO 0
#define PRINT_DATA 0
#define PRINT_ANGLES 0
#define PRINT_G_GRAVITY 0

HardwareSerial &odrive_serial = Serial5;
ODriveArduino odrive(odrive_serial);

const int gyro_address = 0x68; // MPU6050 I2C address
const float gRes = 250.0 / 32768.0;
const float aRes = 4.0f / 32768.0f;
const float pi = 3.141592653589793238462643383279502884f;
const float rad2deg = 180.0f / pi;
const float deg2rad = pi / 180.0f;

int16_t AccX_RAW, AccY_RAW, AccZ_RAW, GyroX_RAW, GyroY_RAW, GyroZ_RAW;
float AccX, AccY, AccZ, gyro_pitch_data_raw, angle_acc, angle_gyro, pitch, yaw, roll, GyroX, GyroY, GyroZ;
bool start = 0;
float speed = 0;
unsigned long cur_timestamp, last_timestamp;
float elapsed_time;

Quaternion orien(1.0f, 0., 0., 0.);
PID angle_PID(15, 0.1, 10, -4.35, 0.2, 16, false, 40); //s et_point , dead_zone, max_speed
float accelBias1[3] = {0.0f, 0.0f, 0.0f};
int32_t gyroBias1[3] = {0, 0, 0};

IntervalTimer PIDtimer;
int16_t gyro_raw[3] = {0};
int16_t accl_raw[3] = {0};
int16_t all_raw[7] = {0};
volatile bool intFlag = false;

void updatePID()
{
  angle_PID.PID_flag = true;
}

void myinthandler()
{
  intFlag = true;
}

void setup()
{
  odrive_serial.begin(115200);
  //pinMode(DIR_L, OUTPUT); pinMode(STEP_L, OUTPUT);  pinMode(DIR_R, OUTPUT); pinMode(STEP_R, OUTPUT);  pinMode(LED, OUTPUT);
  Serial.begin(115200); //Start the serial port at 9600 kbps
  Wire.begin();         //Start the I2C bus as master
  PIDtimer.begin(updatePID, 4000);
  TWBR = 12;                            //Set the I2C clock speed to 400kHz
  Wire.beginTransmission(gyro_address); //Start communication with the address found during search.
  Wire.write(0x6B);                     //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                     //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();               //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address); //Start communication with the address found during search.
  Wire.write(0x1B);                     //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                     //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();               //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address); //Start communication with the address found during search.
  Wire.write(0x1C);                     //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                     //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();               //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address); //Start communication with the address found during search
  Wire.write(0x1A);                     //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                     //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();               //End the transmission with the gyro

  writeByte(gyro_address, INT_PIN_CFG, 0x10); // any read to clear
  writeByte(gyro_address, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

  attachInterrupt(digitalPinToInterrupt(2), myinthandler, RISING); // define interrupt for intPin output of MPU9250 1

  last_timestamp = micros();
}

void loop()
{
  cur_timestamp = micros();

  if (intFlag)
  {
    intFlag = false;
    readAllData(gyro_address, all_raw);
    elapsed_time = (float)(cur_timestamp - last_timestamp) / 1000000.0f;
    last_timestamp = cur_timestamp;

//    Serial.println(elapsed_time, 10);
    memcpy(accl_raw, all_raw, sizeof(int16_t) * 3);
    memcpy(gyro_raw, &all_raw[4], sizeof(int16_t) * 3);

    AccX = accl_raw[2] * aRes;
    AccY = -accl_raw[0] * aRes;
    AccZ = accl_raw[1] * aRes;

    GyroX = -(gyro_raw[2] - gyroBias1[2]) * gRes * deg2rad;
    GyroY = (gyro_raw[0] - gyroBias1[0]) * gRes * deg2rad;
    GyroZ = -(gyro_raw[1] - gyroBias1[1]) * gRes * deg2rad;

#if PRINT_DATA == 1
    Serial.print("a/g:\t");
    Serial.print(AccX);
    Serial.print("\t");
    Serial.print(AccY);
    Serial.print("\t");
    Serial.print(AccZ);
    Serial.print("\t");
    Serial.print(GyroX,4);
    Serial.print("\t");
    Serial.print(GyroY,4);
    Serial.print("\t");
    Serial.println(GyroZ,4);
#endif

    Quaternion omega(0, GyroX, GyroY, GyroZ);
    Quaternion delta_q = orien * omega * 0.5f;
    orien = orien + (delta_q * elapsed_time);
    orien.normalize();

    Quaternion l_gravity(0, AccX, AccY, AccZ);
    Quaternion g_gravity = orien.rotate(l_gravity);

    Quaternion g_ref(0, 0, 0, -1);
    Quaternion q_acc = Quaternion::fromTwoVector(g_gravity, g_ref);
    float error_gravity = fabs(l_gravity.norm() - 1.0f);
    float gain_factor;
    if (error_gravity > 0.2) gain_factor = 0;
    else if (error_gravity < 0.1) gain_factor = 1;
    else
    {
      gain_factor = 1.0f - (error_gravity - 0.1)/0.1f;
    }
    q_acc = Quaternion::slerp(Quaternion(1,0,0,0), q_acc, gain_factor * 0.0015f);
    orien = q_acc * orien;
    orien.toEulerAngle(&roll, &pitch, &yaw);
    pitch = pitch * rad2deg;
    
#if PRINT_G_GRAVITY == 1
    Serial.print(g_gravity.w);
    Serial.print("\t");
    Serial.print(g_gravity.x);
    Serial.print("\t");
    Serial.print(g_gravity.y);
    Serial.print("\t");
    Serial.println(g_gravity.z);
#endif

#if PRINT_ANGLES == 1
    Serial.print("R\\P\\Y:\t");
//    Serial.print(roll * rad2deg);
//    Serial.print("\t");
    Serial.println(pitch * rad2deg,4);
//    Serial.print("\t");
//    Serial.println(yaw * rad2deg);
#endif
  }

    if (!start && pitch > -30 && pitch < 30) {
      start = 1;
      angle_PID.resetErrorSum();
    }
    
    if (pitch > 40 || pitch < -40) start = false;
    speed = angle_PID.updatePID(pitch);
    if (!start) speed = 0;
    odrive.SetVelocity(1, speed);

    Serial.print("P\t");
    Serial.println(pitch,4);
  
}
