#include <Adafruit_BMP280.h>
#include <LSM6.h>
#include <Wire.h>


Adafruit_BMP280 bmp280;   // temperature
LSM6 lsm6ds33;   //Accelerometer and Gyroscope

float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float temperature;

//--------------------------------SETUP-----------------------------------------------
void setup(void) {
  
  //Initialize LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  //Set up serial communication 
  Serial.begin(115200); 
  Serial.println("Data Stream output"); //Print the beginning message upon serial opening 

  //Initialization of sensors 
  bmp280.begin(); //temperature sensor  
  Wire.begin(); //Set up communication for I2C with the Accel and Gyro Sensor

  
  //Ensure that the gyroscope and accelerometer sensor has been initialized
  //If not been initialized, then wait for initilization
  if(!lsm6ds33.init())
  {
    Serial.println("Failed to detect and initilize LSM6DS33!");
    while(1); 
  }
  lsm6ds33.enableDefault(); 

  digitalWrite(LED_BUILTIN, HIGH);  //Turn LED on to indicate the start of sensors reading data
}

//-----------------------------------READING VALUES----------------------------------------------
void loop(void) {

 //Grab temperature sensor data and convert to Fahrenheit 
 temperature = bmp280.readTemperature();
 temperature = ((9/5)*(temperature))+32; 

 //Grab Accel and gyroscope raw data and convert to logical data 
 lsm6ds33.read(); 
 accel_x = lsm6ds33.a.x;
 accel_y = lsm6ds33.a.y;
 accel_z = lsm6ds33.a.z;
 gyro_x = lsm6ds33.g.x;
 gyro_y = lsm6ds33.g.y;
 gyro_z = lsm6ds33.g.z;

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
