#include <Wire.h>
#include <HardwareSerial.h>
//#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

class PID {
  public:
    PID(float Kp, float Ki, float Kd, double set_point, double dead_zone, double max_out, bool flip_out = false, float error_division_factor = 1.0f) :
      Kp(Kp), Ki(Ki), Kd(Kd), set_point(set_point), dead_zone(dead_zone), max_out(max_out), flip_out(flip_out), error_division_factor(error_division_factor)
    {
      default_set_point = set_point;
    }
    double updatePID(double cur_val) {
      if (!PID_flag) {
        if (flip_out) return -out;
        else return out;
      }
      error = (set_point - cur_val) / error_division_factor;
      error_d = error - p_error;
      error_sum += error * Ki;
      p_error = error;
      if (error_sum > max_out / 1.0) error_sum = max_out / 1.0;
      else if (error_sum < -(max_out / 1.0)) error_sum = -max_out / 1.0;
      out = error * Kp + error_d * Kd + error_sum;
      if (out > max_out) out = max_out;
      else if (out < -max_out) out = -max_out;
//      else if (out > -dead_zone && out < dead_zone) out = 0;
      PID_flag = false;
      if (flip_out) return -out;
      else return out;
    }
    void resetErrorSum() {
      error_sum = 0;
    }
    double set_point;
    double default_set_point;
    volatile bool PID_flag = false;
  private:
    float Kp, Ki, Kd;
    double dead_zone, error, p_error, error_d, error_sum, out, max_out;
    int error_division_factor;
    bool flip_out;
};

HardwareSerial& odrive_serial = Serial5;
ODriveArduino odrive(odrive_serial);

const int gyro_address = 0x68; // MPU6050 I2C address
int16_t AccX_RAW, AccY_RAW, AccZ_RAW, GyroX_RAW, GyroY_RAW, GyroZ_RAW;
float AccX, AccY, AccZ, gyro_pitch_data_raw, angle_acc, angle_gyro, pitch;
bool start = 0;
float Kp = 15, Ki = 2, Kd = 30, set_point = 0, dead_zone = 0.5, speed, max_speed = 15;
unsigned long elapsed_time;

PID angle_PID(14.2, 0, 10, -4.3, 0.2, 16, false, 40);

IntervalTimer PIDtimer;

void updatePID() {
  angle_PID.PID_flag = true;
}

//ISR(TIMER2_OVF_vect) {
//  angle_PID.PID_flag = true;
//  TCNT2 = 6;
//}

void setup() {
  odrive_serial.begin(115200);
  //pinMode(DIR_L, OUTPUT); pinMode(STEP_L, OUTPUT);  pinMode(DIR_R, OUTPUT); pinMode(STEP_R, OUTPUT);  pinMode(LED, OUTPUT);
  Serial.begin(115200);                                                       //Start the serial port at 9600 kbps
  Wire.begin();                                                             //Start the I2C bus as master
  PIDtimer.begin(updatePID, 4000);
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //stepper motor timer every 20us
  //TCCR1A = 0; TCCR1B = 0; TCCR1B |= (1 << CS10);  TIMSK1 |= (1 << OCIE1A);  OCR1A = 319;
  //PID controller refresh timer @ 250Hz
//  TCCR2A = 0; TCCR2B = 0; TCCR2B |= (1 << CS22) | (1 << CS21);  TIMSK2 |= (1 << TOIE2); TCNT2 = 6;
  elapsed_time = micros();
//  float gyro_y_cal = 0;
//  for (int i = 0; i < 20000; i++) {
//    Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
//    Wire.write(0x45);                                                         //Start reading at register 43
//    Wire.endTransmission();                                                   //End the transmission
//    Wire.requestFrom(gyro_address, 2);                                        //Request 4 bytes from the gyro
//    gyro_y_cal += (Wire.read() << 8 | Wire.read());                     //Combine the two bytes to make one integer
//  }
//  Serial.println(gyro_y_cal / 20000);
}

void loop() {
  //Get angle from IMU
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(gyro_address, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX_RAW = Wire.read() << 8 | Wire.read();
  AccY_RAW = Wire.read() << 8 | Wire.read() - 7;
  AccZ_RAW = Wire.read() << 8 | Wire.read();
  AccX = AccX_RAW / 16384.0; // X-axis value
  AccY = AccY_RAW / 16384.0; // Y-axis value
  AccZ = AccZ_RAW / 16384.0; // Z-axis value
  angle_acc = (atan(AccZ / abs(AccY)) * 180 / PI) + 10;           //Calculate the current angle according to the accelerometer
  if (!start && angle_acc > -0.5 && angle_acc < 0.5) {
    pitch = angle_acc;
    start = 1;
    angle_PID.resetErrorSum();
  } 
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 4 bytes from the gyro
  GyroX_RAW = Wire.read() << 8 | Wire.read() + 1;
  //gyro_pitch_data_raw = (Wire.read() << 8 | Wire.read()) - 50;                     //Combine the two bytes to make one integer
  pitch += GyroX_RAW * 0.000000008 * (micros() - elapsed_time);                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable
  elapsed_time = micros();
  pitch = pitch * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle
  if (pitch > 45 || pitch < -45) start = false;
  speed = angle_PID.updatePID(pitch);
  //Adjust set point to balance
//  angle_PID.set_point = angle_PID.default
  //if (speed < 0) angle_PID.set_point -= 0.003;
  //if (speed > 0) angle_PID.set_point += 0.003;
  //if (abs(speed) < dead_zone) speed = 0;
  //Set motor speed ------------------------------------------------------------------------------------------
  if (!start) speed = 0;
  odrive.SetVelocity(1, speed);
//  Serial.print(AccX);
//  Serial.print("/");
//  Serial.print(AccY);
//  Serial.print("/");
//  Serial.print(AccZ);
//  Serial.print("/");
//  Serial.print("speed : ");
//  Serial.print(speed);
  Serial.print("\t pitch :");
  Serial.println(pitch);
}
