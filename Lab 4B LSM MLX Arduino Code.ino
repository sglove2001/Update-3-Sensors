#include "Adafruit_LSM6DSOX.h" 
#include "Adafruit_MLX90393.h"
#include "Adafruit_BMP280.h"
#include <Wire.h>

Adafruit_BMP280 bmp; // I2C Default
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
sensors_event_t temp_event, pressure_event;
#define BMP280 0x77 // I2C of BMP280
// Constructor for MLX, Default I2C Adress 0x0C
Adafruit_MLX90393 mlx;
Adafruit_LSM6DSOX imu; // Default Adress 0x6A
sensors_event_t accel, gyro, temp;
float mx=0, my=0, mz=0;
char string_temp[32];
char write_buffer[64];

unsigned long prev_micros = 0;

void setup() {
    // Begin I2C
    Wire.begin();
    //Serial.begin(9600);
    //while (!Serial) {
    //    delay(10);
    //}
    // Begin Magno
    if (! mlx.begin_I2C()) {
        //while (1) { 
        //    Serial.println("MLX Error");
        //    delay(10); 
        //}
    }
    // Set Magno Gain
    mlx.setGain(MLX90393_GAIN_2_5X);
    // Set resolution, per axis
    mlx.setResolution(MLX90393_X, MLX90393_RES_19);
    mlx.setResolution(MLX90393_Y, MLX90393_RES_19);
    mlx.setResolution(MLX90393_Z, MLX90393_RES_16);
    // Set oversampling
    mlx.setOversampling(MLX90393_OSR_2);
    // Set digital filtering
    mlx.setFilter(MLX90393_FILTER_6);

    // Begin BMP280
    if (!bmp.begin()) {
        //while (1) {
        //    Serial.println("IMU Error");
        //    delay(10); 
        //}
    }
    // Set BMP280 Settings
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */

    // Begin IMU
    imu.begin_I2C(uint8_t(0x6B));
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
    delay(10);
}
 

void loop() {
    // Get Accel and Gyro Measurements
    imu.getEvent(&accel, &gyro, &temp);
    // Read Magno Data
    mlx.readData(&mx, &my, &mz);

    // Build Accel String
    dtostrf(accel.acceleration.x,6,4,string_temp);
    sprintf(write_buffer, "%c %s",'A',string_temp);
    dtostrf(accel.acceleration.y,6,4,string_temp);
    sprintf(write_buffer, "%s %s",write_buffer,string_temp);
    dtostrf(accel.acceleration.z,6,4,string_temp);
    sprintf(write_buffer, "%s %s",write_buffer,string_temp);
    // Write Accel String
    //Serial.println(write_buffer);
    Wire.beginTransmission(0x1A);
    Wire.write(write_buffer);
    Wire.endTransmission(true);

    // Build Gyro String
    dtostrf(gyro.gyro.x,6,4,string_temp);
    sprintf(write_buffer, "%c %s",'G',string_temp);
    dtostrf(gyro.gyro.y,6,4,string_temp);
    sprintf(write_buffer, "%s %s",write_buffer,string_temp);
    dtostrf(gyro.gyro.z,6,4,string_temp);
    sprintf(write_buffer, "%s %s",write_buffer,string_temp);
    // Write Gyro String
    //Serial.println(write_buffer);
    Wire.beginTransmission(0x1A);
    Wire.write(write_buffer);
    Wire.endTransmission(true);

    // Build Magno String
    dtostrf(mx,6,4,string_temp);
    sprintf(write_buffer, "%c %s",'M',string_temp);
    dtostrf(my,6,4,string_temp);
    sprintf(write_buffer, "%s %s",write_buffer,string_temp);
    dtostrf(mz,6,4,string_temp);
    sprintf(write_buffer, "%s %s",write_buffer,string_temp);
    // Write Magno String
    //Serial.println(write_buffer);
    Wire.beginTransmission(0x1A);
    Wire.write(write_buffer);
    Wire.endTransmission(true);

    // Get BMP280 Data
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);
    // Build BMP280 String
    dtostrf(temp_event.temperature,5,2,string_temp);
    sprintf(write_buffer, "%c %s",'P',string_temp);
    dtostrf(pressure_event.pressure, 6,3,string_temp);
    sprintf(write_buffer, "%s %s",write_buffer,string_temp);
    dtostrf(micros() - prev_micros, 7,1,string_temp);
    prev_micros = micros();
    sprintf(write_buffer, "%s %s",write_buffer, string_temp);
    dtostrf(imu.temperature, 5,2, string_temp);
    sprintf(write_buffer, "%s %s",write_buffer, string_temp);

    // Write BMP280 String
    //Serial.println(write_buffer);
    Wire.beginTransmission(0x1A);
    Wire.write(write_buffer);
    Wire.endTransmission(true);
    delay(10);
}
