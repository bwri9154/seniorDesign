//Include the different libraries 
#include <Adafruit_BMP280.h>    //Adafruit temp sensor Library 
#include <LSM6.h> //Adafruit accelerometer library
#include <SPI.h>
#include <SD.h> //SD Card library
#include "RTClib.h"


  
//Temp initialization
Adafruit_BMP280 bmp280;   // Use I2C interface
Adafruit_Sensor *bmp280_temp = bmp280.getTemperatureSensor(); 


//Accel and Gyro Initializationg
LSM6 lsm6ds33;   //Accelerometer and Gyroscope

//SD Card Initialization
//const int chipSelect = 10;
//Sd2Card card; 
//SdVolume volume;
//SdFile root; 
File myFile; 
RTC_PCF8523 rtc;
char timestamp[30];

//Variables
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float temperature;
char report[80]; //Array of length 80
uint32_t period = 1 * 60000L;       // 1 minutes

//------------------Setting up Real Time Clock-------------------------
void dateTime(uint16_t* date, uint16_t* time) {
  DateTime now = rtc.now();
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day()); 
  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}


//--------------------------------SETUP-----------------------------------------------
void setup(void) {
  
  //Initialize RED LED as an output
  pinMode(LED_BUILTIN, OUTPUT);
  
  //Set up serial communication 
  Serial.begin(9600); 
  while(!Serial);  //If the Serial console is not open, wait

  //Sensor Setup 
  bmp280.begin(); //temperature sensor  
  if(!lsm6ds33.init())  //Ensure that the gyroscope and accelerometer sensor has been initialized. If not been initialized, then wait for initilization
  {
    Serial.println("Failed to detect and initilize LSM6DS33!");
    while(1)
      delay(10);  
  }
  lsm6ds33.enableDefault(); 

  //SDCard Setup
 Serial.print("Initializing SD Card...");
  //if(!SD.begin(10)){
    //Serial.println("SD Card Initialization Failed.");
    //return; 
    //}
    
  Serial.println("Initialization Complete.");
  Serial.print("Creating Text File...");
  myFile = SD.open("SDTest1.txt", FILE_WRITE);   // open the file. Note: that only one file can be open at a time so you have to close this one before opening another. 
  Serial.println("Text File Created.");

  //if (! rtc.begin()) {
    //Serial.println("Couldn't find RTC");
    //Serial.flush();
    //abort();
  //}
  
  // set date time callback function
  rtc.start(); 
  SdFile::dateTimeCallback(dateTime);

//--------------------------------Collecting Data and recording it-------------------------------------
for( uint32_t tStart = millis();  (millis()-tStart) < period;  ){
  Serial.print("Collecting data..."); //Print the beginning message upon serial opening 
  digitalWrite(LED_BUILTIN, HIGH);  //Turn LED on to indicate the start of sensors reading data
    
  //Grab temperature sensor data and convert to Fahrenheit 
  temperature = bmp280.readTemperature();
  temperature = ((9/5)*(temperature))+32; 
  
  //Grab Accel and Gyro raw data and convert to logical data 
  lsm6ds33.read(); //Takes a reading from both the accelerometer and gyro and stores the values in the vectors a and g
  
  snprintf(report, sizeof(report), "Accel: %6d %6d %6d , Gyro: %6d %6d %6d",
     lsm6ds33.a.x, lsm6ds33.a.y, lsm6ds33.a.z,
     lsm6ds33.g.x, lsm6ds33.g.y, lsm6ds33.g.z);
      
  Serial.print("done. ");
    
  //Write all data to the SD Card
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print(" Writing to File...");
    myFile.print("Temp: ");
    myFile.print(temperature);
    myFile.print(" , ");
    myFile.println(report);
    Serial.println("done.");
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("Error opening File!");
  }
  }
  // close the file:
    digitalWrite(LED_BUILTIN, LOW);  //Turn LED on to indicate the start of sensors reading data
    myFile.close();
    Serial.println("done.");
    return; 
}

void loop(void) {
//Leave Empty
}
