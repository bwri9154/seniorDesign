//Operation of the electronic system: The Adafruit board will be powered by a 5V source (from a buck boost converter). Upon Power up, a 

//Include the different libraries 
#include <Adafruit_BMP280.h>    //Adafruit temp sensor Library 
#include <LSM6.h> //Adafruit accelerometer library
#include <SPI.h>
#include <SD.h> //SD Card library
#include "RTClib.h"
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>  //Library that allows communication to OneWire devices (DS18B20 in our case)
#include <DallasTemperature.h>//Library containing temperature functions related to Maxim Temperature IC's

// Data wire is plugged into pin 5 on the Adafruit Feather 
#define ONE_WIRE_BUS 5
#define TdsSensorPin A0
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point

// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

//Variables related to the TDS sensor
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;


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
char report[80]; //Array of length 80
uint32_t period = 30000;       // .5 minutes
#define BUTTON 7
float intemp;
uint32_t tStart; 

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
  pinMode(TdsSensorPin,INPUT);
  
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
  
//  SDCard Setup
//  if(!SD.begin(10)){
//    WheelDisplay(5);//Display RED
//    abort(); 
//    }

  sensors.begin();
    
  myFile = SD.open("SDTest2.txt", FILE_WRITE);   // open the file. Note: that only one file can be open at a time so you have to close this one before opening another. 
  Serial.println("Text File Created.");
  delay(10);
  
  while(digitalRead(BUTTON) == HIGH){
    Serial.println(digitalRead(BUTTON)); 
   //Wait for button press to initiate Beginning of Travel Code; LED LIGHT SHOULD BE GREEN while waiting
  }
  delay(1000);

  tStart = millis();
  WheelDisplay(2000); //Display Yellow

}

void loop(void){
  if((millis()-tStart) < period){ //Run for .5 minutes 
      SensorData(myFile); 
      Serial.println(digitalRead(BUTTON));
      //delay(400);
  
      if(digitalRead(BUTTON) == LOW){
        CloseFile(myFile); 
      }
  }
  else{
    CloseFile(myFile);
  }
}

//--------------------Close file------------------
void CloseFile(File myFile){
     WheelDisplay(5);//Display red
     // close the file:
     myFile.close();
     BLE(); 
     //Serial.print("DONE");
     //delay(1000);
     abort();
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
void SensorData(File myFile){
  //Grab ON temperature sensor data and convert to Fahrenheit 
  intemp = bmp280.readTemperature();
  intemp = ((9/5)*(intemp))+32; 
  
  //Grab Accel and Gyro raw data and convert to logical data 
  lsm6ds33.read(); //Takes a reading from both the accelerometer and gyro and stores the values in the vectors a and g
  snprintf(report, sizeof(report), "Accel: %6d %6d %6d , Gyro: %6d %6d %6d",
     lsm6ds33.a.x, lsm6ds33.a.y, lsm6ds33.a.z,
     lsm6ds33.g.x, lsm6ds33.g.y, lsm6ds33.g.z);  //Write all data to the SD Card

  //Read Turbidity
    int sensorValue = analogRead(A1);// read the input on analog pin 1:
    float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float turbidity = ((-1120*(voltage * voltage)) + (5742.3 * voltage) - 4352.9);

  //Grab OFF temperature sensor data
    sensors.requestTemperatures(); // Send the command to get temperature readings  
    
  // if the file opened okay, write to it:
  if (myFile) {
    myFile.print("ONTemp: ");
    myFile.print(intemp);
    myFile.print("F");
    myFile.print(" , ");
    myFile.print(report);
    myFile.print(" , ");
    myFile.print("VoltTurb: ");
    myFile.print(voltage); // print out the value you read:
    myFile.print("V");
    myFile.print(" , ");
    myFile.print("Turbidity: ");
    myFile.print(turbidity);
    myFile.print("NTU");
    myFile.print(" , ");
    myFile.print("OFFTemp: ");
    myFile.print(sensors.getTempCByIndex(0)); // Why "byIndex"?
    myFile.println("C");
  }
  return;
}

//Function for the BLE
void BLE(void){
  //WheelDisplay(1000);//Display GREEN
  
}

int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}
