#include <Adafruit_BMP280.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_Sensor.h>

Adafruit_BMP280 bmp280;   // temperature
Adafruit_LSM6DS lsm6ds;   //Accelerometer and Gyroscope

float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float temperature;

void setup(void) {
  Serial.begin(115200); 
  Serial.println("Data Stream output"); //Print the beginning message upon serial opening 

  //initialize the sensors
  bmp280.begin(); //temperature sensor  
  lsm6ds.begin_I2C(); //Accel and Gyro Sensor communication through I2C
}

void loop(void) {
 //----------------Collecting the Data-----------------------

 //Grab temperature sensor data and convert to Fahrenheit 
 temperature = bmp280.readTemperature();
 //temperature = ((9/5)*(temperature))+32; 
 
 sensors_event_t accel; 
 sensors_event_t gyro; 
 sensors_event_t temp; 

 lsm6ds.getEvent(&accel, &gyro, &temp);
 accel_x = accel.acceleration.x;
 accel_y = accel.acceleration.y;
 accel_z = accel.acceleration.z;
 gyro_x = gyro.gyro.x;
 gyro_y = gyro.gyro.y;
 gyro_z = gyro.gyro.z;

 //----------------Displaying the data---------------------
 Serial.print("Temperature: ");
 Serial.print(temperature);
 Serial.println(" F");
 
 Serial.print("Acceleration: ");
 Serial.print(accel_x);
 Serial.print(" ");
 Serial.print(accel_y);
 Serial.print(" ");
 Serial.print(accel_z);
 Serial.println(" m/s^2");
 
 Serial.print("Gyro: ");
 Serial.print(gyro_x);
 Serial.print(" ");
 Serial.print(gyro_y);
 Serial.print(" ");
 Serial.print(gyro_z);
 Serial.println(" dps");
 
}
