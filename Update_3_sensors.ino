#include <WiFi.h>
#include <Wire.h>
#include "ThingSpeak.h"
#include "arduino.h"
#include "kalman.h"

// Thingspeak is initialized at the end of wifiSetup()
// Writing to Thingspeak is to be implemented in part B

struct imu_sensor {
  float x=0;
  float y=0;
  float z=0;
  float temp_c=0;
  float temp_f=0;
};

struct pressure_sensor {
  float temp_c=0;
  float temp_f=0;
  float pressure_hp=0;
};

float ard_millis=0; // loop duration from arduino
float prev_millis=0; // For timers

imu_sensor a; // Structs for message parsing use
imu_sensor g; // Do not use
imu_sensor m;
pressure_sensor b;

imu_sensor accel; // Structs for manipulation
imu_sensor gyro;  // Use to get data from sensors
imu_sensor magno;
imu_sensor imu_temp;
pressure_sensor bmp_280;

float estimated_altitude = 0;




//*************************************************************************

 /**
 * @brief Called when I2C message is recieved.
 * 
 * @param len Length of incoming message.
 */
 
void onReceive(int len){
  // Creates char array to store message in
  char message[len];
  // Iterator
  char num = 0;
  // Writes each received char to array at index num
  while(Wire.available()){
    message[num] = Wire.read();
    num++;
  }
  // Parses message from char array to series of sensor values
  parseMessage(message, len);
}

void setAccel(){
  accel.x = a.x;
  accel.y = a.y;
  accel.z = a.z;
}

void setGyro(){
  gyro.x = g.x;
  gyro.y = g.y;
  gyro.z = g.x;
}

void setMagno(){
  magno.x = m.x;
  magno.y = m.y;
  magno.z = m.z;
}

void setIMUTemp(){
  imu_temp.temp_c = a.temp_c;
  imu_temp.temp_f = (a.temp_c * 9/5) + 32;
}

void setBMP280(){
  bmp_280.pressure_hp = b.pressure_hp;
  bmp_280.temp_c = b.temp_c;
  bmp_280.temp_f = (b.temp_c * 9/5) + 32;
}

void updateSensors(){
  setAccel();
  setGyro();
  setMagno();
  setBMP280();
  setIMUTemp();
}

 /**
 * @brief Parses Arduino I2C messages into suitable data.
 * 
 * @param message Message from the Arduino.
 * @param len Length of recieved message.
 * @return true If parse is succesful.
 * @return false If parse is unsuccesful.
 */
bool parseMessage(char message[], int len) {
  char *end_ptr;
  char msg[len-1];
  for(int i=1; i < len; i++){
    msg[i-1] = message[i];
  }
  switch (message[0]){
  case 'A':
    a.x = strtof(msg, &end_ptr);
    a.y = strtof(end_ptr, &end_ptr);
    a.z = strtof(end_ptr, NULL);
    return true;
  case 'G':
    g.x = strtof(msg, &end_ptr);
    g.y = strtof(end_ptr, &end_ptr);
    g.z = strtof(end_ptr, NULL);
    return true;
  case 'M':
    m.x = strtof(msg, &end_ptr);
    m.y = strtof(end_ptr, &end_ptr);
    m.z = strtof(end_ptr, NULL);
    return true;
  case 'P':
    b.temp_c = strtof(msg, &end_ptr);
    b.pressure_hp = strtof(end_ptr, &end_ptr);
    ard_millis = strtof(end_ptr, &end_ptr);
    a.temp_c = strtof(end_ptr, NULL);
    return true;
  default:
    return false;
  }
}
double accel_yaw_deg, ax, ay, az, gx, gy, gz, mx, my, mz, accel_roll,  accel_pitch, gyro_roll, gyro_roll_vel, prev_gyro_roll =0, tau, alpha, comp_roll;
double gyro_pitch, gyro_pitch_vel;
float grav =9.81;
float pitch_theta;
double ax_temp = 0, ay_temp = 0, az_temp = 0, gx_temp = 0, gy_temp = 0, gz_temp = 0,mx_temp = 0, my_temp = 0, mz_temp = 0;
double ax_cal, ay_cal,az_cal, gx_cal, gy_cal,gz_cal, mx_cal, my_cal,mz_cal, cal_time;
double ax_max = 0, ax_min = 0, ay_max = 0, az_max = 0, ay_min = 0, gx_max = 0, gx_min = 0, gy_max = 0, gy_min =0, gz_max = 0, gz_min =0, mx_max = 0, mx_min =0, my_max =0, my_min =0, mz_max =0, mz_min = 0;
double az_min = 0;
double previousTime = 0, currentTime = 0, elapsedTime;
//float dt = 0.2; // sampling period
float fs = 50;
unsigned long timer;
//unsigned long time;

/**
 * @brief Prints all measured values once per execution, in a comma seperated format.
 */
void printInputCSV(){
  Serial.print(a.x);
  Serial.print(',');
  Serial.print(a.y);
  Serial.print(',');
  Serial.print(a.z);
  Serial.print(',');
  Serial.print(g.x);
  Serial.print(',');
  Serial.print(g.y);
  Serial.print(',');
  Serial.print(g.z);
  Serial.println(',');
  /*Serial.print(m.x);
  Serial.print(',');
  Serial.print(m.y);
  Serial.print(',');
  Serial.print(m.z);
  Serial.print(',');
  Serial.print(b.temp_c);
  Serial.print(',');
  Serial.print(b.pressure_hp,4);
  Serial.print(',');
  Serial.print(ard_millis,0);
  Serial.print(',');
  Serial.println(a.temp_c);*/
}
void printCalibratedInputCSV(){
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.println(',');
  /*Serial.print(mx);
  Serial.print(',');
  Serial.print(my);
  Serial.print(',');
  Serial.print(mz);
  Serial.print(',');
  Serial.print(b.temp_c);
  Serial.print(',');
  Serial.print(b.pressure_hp,4);
  Serial.print(',');
  Serial.print(ard_millis,0);
  Serial.print(',');
  Serial.println(a.temp_c);*/
}
float curr_millis;
int cal_counter = 0;
void calibration() { //Seems like magnometer isnt very accurate in this calibration method... to be improved
  if (cal_counter > 2 & cal_counter < 50) {
    if (cal_counter == 4) {
      ax_max = ax_min = ax_temp;ay_max = ay_min = ay_temp;az_max = az_min = az_temp;
      gx_max = gx_min = gx_temp;gy_max = gy_min = gy_temp;gz_max = gz_min = gz_temp;
      mx_max = mx_min = mx_temp;my_max = my_min = my_temp;mz_max = mz_min = mz_temp;
      
    }
    if (cal_counter > 10) {
      ax_max = max(ax_max, ax_temp); ax_min = min(ax_min, ax_temp); ay_max = max(ay_max, ay_temp); ay_min = min(ay_min, ay_temp); az_max = max(az_max, az_temp); az_min = min(az_min, az_temp);
      gx_max = max(gx_max, gx_temp); gx_min = min(gx_min, gx_temp); gy_max = max(gy_max, gy_temp); gy_min = min(gy_min, gy_temp); gz_max = max(gz_max, gz_temp); gz_min = min(gz_min, gz_temp);
      mx_max = max(mx_max, mx_temp); mx_min = min(mx_min, mx_temp); my_max = max(my_max, my_temp); my_min = min(my_min, my_temp); mz_max = max(mz_max, mz_temp); mz_min = min(mz_min, mz_temp);
    }
    
    //temp initialization
    ax_temp = a.x, ay_temp = a.y, az_temp = a.z, gx_temp = g.x, gy_temp = g.y,gz_temp = g.z, mx_temp = m.x, my_temp = m.y, mz_temp = m.z;
  }
  //use midpoint as calibration level:
  ax_cal = (ax_max + ax_min)/2.0, ay_cal = (ay_max + ay_min)/2.0, az_cal = (az_max+az_min)/2.0, gx_cal = (gx_max + gx_min)/2.0, gy_cal = (gy_max + gy_min)/2.0, gz_cal = (gz_max+gz_min)/2.0, mx_cal = (mx_max + my_min)/2.0; my_cal = (my_max + my_min)/2.0, mz_cal = (mz_max+mz_min)/2.0;
  //calibrated values:
  ax = a.x - ax_cal, az = a.z - az_cal, ay = a.y - ay_cal, gx = g.x - gx_cal, gy = g.y - gy_cal, gz = g.z - gz_cal, mx = m.x - mx_cal, my = m.y - my_cal, mz = m.z - mz_cal;
  
  //ax = RAD_TO_DEG*ax;
  
  if (cal_counter < 60) {
   cal_counter++; 
  }
  /*Serial.print("az: "); Serial.println(az);
  Serial.print("az_cal: "); Serial.println(az_cal);
  Serial.print("az_min: "); Serial.println(az_min);
  Serial.print("az_max: "); Serial.println(az_max);
  */
}

void calc_roll() {
  accel_roll = atan2(a.y,a.z)*-1*RAD_TO_DEG; //accurate value with accelerometer
  gyro_roll_vel = g.x*-1*RAD_TO_DEG;
  
  if (gyro_roll > 180) {
    gyro_roll = gyro_roll - 360;
  }
        
    
  if (gyro_roll < -180) {
    gyro_roll = gyro_roll + 360;
  }
        
  //gyro_roll_vel = g.x*(-90.0/3.14); //17.5;
  if ((gyro_roll_vel <= 0.5) && (gyro_roll_vel >=-0.5) ) {
  }
  else {
    gyro_roll = (gyro_roll+(gyro_roll_vel)*elapsedTime); 
    
  }
  
}
float ap; 
void calc_pitch() {
  ap = a.x/9.81;
  accel_pitch = atan(-a.x/sqrt(a.y*a.y+a.z*a.z))*RAD_TO_DEG; //accurate value with accelerometer
  gyro_pitch_vel = g.y*RAD_TO_DEG;
  
  /*if (gyro_pitch > 180) {
    gyro_pitch = gyro_pitch - 360;
  }
        
    
  if (gyro_pitch < -180) {
    gyro_pitch = gyro_pitch + 360;
  }*/
        
  
  if ((gyro_pitch_vel <= 0.7) && (gyro_pitch_vel >=-0.7) ) {
    
  }
  else {
    gyro_pitch = (gyro_pitch+(gyro_pitch_vel)*elapsedTime); 
    
  }
  
}
Kalman kalmanX;
Kalman kalmanY;
double gyroXangle, gyroYangle;
double compAngleX, compAngleY;
double kalAngleX, kalAngleY;

void setup() {
  // Begins Serial
  Serial.begin(115200);
  // Begins WiFi
  
  // Sets I2C message reception behavior
  Wire.onReceive(onReceive);
  // Begins I2C
  Wire.setPins(5,4);
  Wire.begin(uint8_t(0x1A));
  delay(200); //stabilization for sensor
  
  calc_pitch();
  calc_roll();
  kalmanX.setAngle(accel_roll);
  kalmanY.setAngle(accel_pitch);
  gyroXangle = accel_roll;
  gyroYangle = accel_pitch;
  compAngleX = accel_roll;
  compAngleY = accel_pitch;

 /* // Initialize prev_millis
  prev_millis = millis();
  //Initialize curr_millis
  curr_millis = millis();*/
  timer = millis();
  delay(500);

}
void print_roll() {
  Serial.print(accel_roll);
  Serial.print(",");
  Serial.print(gyro_roll);
  Serial.print(",");
  //Serial.println(gyro_roll_vel*dt);
}
void print_pitch() {
  Serial.print(accel_pitch);
  Serial.print(",");
  //Serial.println(gyro_pitch_vel);
  //Serial.print(",");
  Serial.println(gyro_pitch);
}
double solarV = 0;
double readSolar() {
  solarV = analogRead(2)*3.3/4095.0;
  return solarV;
}
double currSolar = 0;
double readCurrent() {
  currSolar = analogRead(3)*3.3/4095.0;
  return currSolar;
}
double pwr = 0;
double powSolar(double x, double y) {
  pwr =  x*y;
  return pwr; 
}


int counter = 0;
void loop() {
  
  double dt = (millis() - timer)/1000;
  timer = millis();
  // Sets Accel, Gyro, Magno and BMP280 structs once per loop from most recent data.
  updateSensors();
  
  calc_roll();
  calc_pitch();
  readSolar();
  powSolar(solarV, currSolar);

  //calculate angle using kalman filter
  kalAngleX = kalmanX.getAngle(accel_roll, gyro_roll_vel, dt);
  kalAngleY = kalmanY.getAngle(accel_pitch, gyro_pitch_vel, dt);
  //comp filter
  compAngleX = 0.93*(compAngleX+gyro_roll_vel*dt) + 0.07*accel_roll;
  compAngleY = 0.93*(compAngleY + gyro_pitch_vel*dt) + 0.07*accel_pitch;
  //handling gyroscopic drift
  if (gyroXangle < -180 || gyroXangle > 180) {
    gyroXangle = kalAngleX;
  }
  if (gyroYangle < -180 || gyroYangle > 180) {
    gyroYangle = kalAngleY;
  }
  
  Serial.print((kalAngleX));
  Serial.print(", ");
  Serial.print((kalAngleY));
  Serial.print(", ");
  Serial.print((pwr));
  Serial.print(":");

 

  
 

  delay(1000); 

}

