#include "MPU9250.h"
#include "math.h"

MPU9250 mpu;

class Value_t{
  public:
    float getValue();
    void setValue(float m_value);
  private:
    float val;
    
};

void Value_t::setValue(float m_value){
  val = m_value;
}
float Value_t::getValue(){
  return val;
}

class Kinematics{
  public:
    void quaternionFromEuler(float roll, float pitch, float yaw);
    void sendHead();
    void sendEnd();
    void sendFloat(float val);
    void setRPY();
    void sendOxOyOzOw();
    void sendAxAyAz();
    Value_t roll, pitch, yaw;
    Value_t angular_x, angular_y, angular_z;
  private:
    Value_t ox,oy,oz,ow;
    Value_t px, py, pz;
    Value_t twist_x, twist_y;
};

void Kinematics::sendHead(){
  Serial.print(char(255));
}
void Kinematics::sendEnd(){
  Serial.print(char(255));
  Serial.print(char(255));
  Serial.print(char(254));
  Serial.print(char(254));
  Serial.print("\n");
}
void Kinematics::sendFloat(float val){
  byte * b = (byte *) &val;
  Serial.write(b[0]);
  Serial.write(b[1]);
  Serial.write(b[2]);
  Serial.write(b[3]);
}

void Kinematics::setRPY(){
  quaternionFromEuler(roll.getValue(), pitch.getValue(), yaw.getValue()); 
}

void Kinematics::sendOxOyOzOw(){
  Serial.print("ox");
  sendFloat(ox.getValue());
  Serial.print("oy");
  sendFloat(oy.getValue());
  Serial.print("oz");
  sendFloat(oz.getValue());
  Serial.print("ow");
  sendFloat(ow.getValue());
}

void Kinematics::sendAxAyAz(){
  Serial.print("ax");
  sendFloat(angular_x.getValue());
  Serial.print("ay");
  sendFloat(angular_y.getValue());
  Serial.print("az");
  sendFloat(angular_z.getValue());
}

void Kinematics::quaternionFromEuler(float roll, float pitch, float yaw){
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  ow.setValue(cy * cp * cr + sy * sp * sr);
  ox.setValue(cy * cp * sr - sy * sp * cr);
  oy.setValue(sy * cp * sr + cy * sp * cr);
  oz.setValue(sy * cp * cr - cy * sp * sr);
}



Kinematics kts = Kinematics();
float gyro_z_static_bias = 0;
void setup() {

  Serial.begin(460800);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
        Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        delay(5000);
      }
  }
  kts.yaw.setValue(0.0);
  mpu.calibrateAccelGyro();
  // unit of gyro_z_static_bias is deg/s
  gyro_z_static_bias = mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY;
}

void loop() {

  
  bool send = false;
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      // Set r,p,y
      kts.roll.setValue(mpu.getRoll()/180.0*3.1415926535);
      kts.pitch.setValue(mpu.getPitch()/180.0*3.1415926535*-1.0);
      //kts.yaw.setValue(mpu.getYaw()/180.0*3.1415926535);
      if(fabs(mpu.getGyroZ() - gyro_z_static_bias)>6){ // 0.1 rad/sec
        kts.yaw.setValue(kts.yaw.getValue() + (mpu.getGyroZ() - gyro_z_static_bias)/180.0*3.1415926535 * (millis()-prev_ms)/1000.0);
      }
 
      kts.angular_x.setValue((mpu.getGyroX() - mpu.getGyroBiasX())/180.0*3.1415926535);
      kts.angular_y.setValue((mpu.getGyroY() - mpu.getGyroBiasY())/180.0*3.1415926535);
      kts.angular_z.setValue((mpu.getGyroZ() - gyro_z_static_bias)/180.0*3.1415926535);

      kts.setRPY();//compute quternion
      prev_ms = millis();
      send = true;
    }
  }
  if(send){
    kts.sendHead();
    kts.sendOxOyOzOw();
    kts.sendAxAyAz();
    kts.sendEnd();
  }
 
  
}
