//Operation of the electronic system: The Adafruit board will be powered by a 5V source (from a buck boost converter). Upon Power up, a 

//Include the different libraries 
#include <Adafruit_BMP280.h>    //Adafruit temp sensor Library 
#include <LSM6.h> //Adafruit accelerometer library
#include <SPI.h>
#include <SD.h> //SD Card library
#include "RTClib.h"
#include <Adafruit_NeoPixel.h>

//NEOPIXEL Light 
#define NEOPIXELPIN 8
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);
  
//Temp initialization
Adafruit_BMP280 bmp280;   // Use I2C interface
Adafruit_Sensor *bmp280_temp = bmp280.getTemperatureSensor(); 


//Accel and Gyro Initializating
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
uint32_t period = 5 * 60000L;       // 5 minutes
#define BUTTON 7

//Setting up Real Time Clock
void dateTime(uint16_t* date, uint16_t* time) {
  DateTime now = rtc.now();
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day()); 
  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}


//--------------------------------SETUP-----------------------------------------------
void setup(void) {
  WheelDisplay(1000);//Display GREEN
  
  //Initialize Neopixel Set up 
  strip.begin();

  //Initialize the pushbutton pin as an input
  pinMode(BUTTON, INPUT_PULLUP);
  
  //Set up serial communication 
  Serial.begin(115200); 
    
  //Sensor Setup 
  bmp280.begin(); //temperature sensor  
  if(!lsm6ds33.init())  //Ensure that the gyroscope and accelerometer sensor has been initialized. If not been initialized, then wait for initilization
  {
    Serial.println("Failed to detect and initilize LSM6DS33!");
    while(1)
      delay(10);  
  }
  lsm6ds33.enableDefault(); 

  // set date time callback function
  rtc.start(); 
  SdFile::dateTimeCallback(dateTime);
  
  //SDCard Setup
//  if(!SD.begin(10)){
//    WheelDisplay(5);//Display RED
//    abort(); 
//    }
    
  myFile = SD.open("SDTest1.txt", FILE_WRITE);   // open the file. Note: that only one file can be open at a time so you have to close this one before opening another. 
  Serial.println("Text File Created.");
  delay(10);
  
  while(digitalRead(BUTTON) == HIGH); //Wait for button press to initiate Beginning of Travel Code; LED LIGHT SHOULD BE GREEN
  delay(1000);

for( uint32_t tStart = millis();  (millis()-tStart) < period;  ){
    WheelDisplay(2000); //Display Yellow
    SensorData();
    SDCARDWrite(myFile); 
    delay(400);

  if(digitalRead(BUTTON) == LOW){
    WheelDisplay(5);//Display red
    // close the file:
    myFile.close();
    BLE(); 
    Serial.print("DONE");
    abort();
  }
}
}

void loop(void){
  //LEAVE EMPTY
}
}
//-----------------------------------------------Function to Display Different Colors on the Color Wheel 
void WheelDisplay(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    strip.setPixelColor(0, strip.Color(255 - WheelPos * 3, 0, WheelPos * 3));
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    strip.setPixelColor(0, strip.Color(0, WheelPos * 3, 255 - WheelPos * 3));
  }
  WheelPos -= 170;
  strip.setPixelColor(0, strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0));
  strip.setBrightness(20);
  strip.show();
  return;
}

//------------------------------------------------Function to begin to collect sensor Data 
void SensorData(void){
  //Grab temperature sensor data and convert to Fahrenheit 
  temperature = bmp280.readTemperature();
  temperature = ((9/5)*(temperature))+32; 
  
  //Grab Accel and Gyro raw data and convert to logical data 
  lsm6ds33.read(); //Takes a reading from both the accelerometer and gyro and stores the values in the vectors a and g
  
  snprintf(report, sizeof(report), "Accel: %6d %6d %6d , Gyro: %6d %6d %6d",
     lsm6ds33.a.x, lsm6ds33.a.y, lsm6ds33.a.z,
     lsm6ds33.g.x, lsm6ds33.g.y, lsm6ds33.g.z);
 Serial.println("Data Collected");
  return;
}

//---------------------------------------------------Function to Write data to the SD Card 
void SDCARDWrite(File myFile){
  //Write all data to the SD Card
  // if the file opened okay, write to it:
  if (myFile) {
    myFile.print("Temp: ");
    myFile.print(temperature);
    myFile.print(" , ");
    myFile.println(report);
  } 
  Serial.println("Data Stored");
  return; 
}
void BLE(){

}
