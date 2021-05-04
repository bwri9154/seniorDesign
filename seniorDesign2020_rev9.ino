//******seniorDesignSp2021 Sensor Suite ON Board Sensor code*********
//Pupose: Using the AdaFruit Feather NRF5240 Sense board, the on board accelerometer/gryroscope (LSM6DS33) and the temperature sensors were manipulated to display 
//specific readings. A red LED was indicated HIGH whenever the board was collecting data. 
//Additional offboard sensors to obtain the turbidity, salinity and temperature of a water sample
//Operation of the electronic system: The Adafruit board will be powered by a 5V source (from a buck boost converter). Upon Power up, a green LED will activate
//indicating the system is ready. See User Manual for specific LED meanings  

//Include the different libraries 
#include <Adafruit_BMP280.h>    //Adafruit temp sensor Library 
#include <LSM6.h> //Adafruit accelerometer library
#include <SPI.h>
#include <SD.h> //SD Card library
#include "RTClib.h"
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>  //Library that allows communication to OneWire devices (DS18B20 in our case)
#include <DallasTemperature.h>//Library containing temperature functions related to Maxim Temperature IC's
#include <bluefruit.h>

// Data wire is plugged into pin 5 on the Adafruit Feather 
#define ONE_WIRE_BUS 5
#define TdsSensorPin A0
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
// 0x004C is Apple (for example)
#define MANUFACTURER_ID   0x004C 

// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


//BLE
 //AirLocate UUID: E2C56DB5-DFFB-48D2-B060-D0F5A71096E0
uint8_t beaconUuid[16] =
{
  0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2,
  0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0,
};
// A valid Beacon packet consists of the following information:
// UUID, Major, Minor, RSSI @ 1M
BLEBeacon beacon(beaconUuid, 0x0001, 0x0000, -54);


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
uint32_t period = 300000;       // 5 minutes
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
  WheelDisplay('g');//Display GREEN
  
  //Initialize Neopixel Set up 
  strip.begin();

  //Initialize the pushbutton pin as an input
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(TdsSensorPin,INPUT);
  
  //Set up serial communication 
  Serial.begin(115200);
  //while(!Serial); 
    
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
  if(!SD.begin(10)){
    WheelDisplay('r');//Display RED
    abort(); 
    }

  sensors.begin();
    
  myFile = SD.open("SDTest2.txt", FILE_WRITE);   // open the file. Note: that only one file can be open at a time so you have to close this one before opening another. 
  Serial.println("Text File Created.");
  delay(10);

  //BLE();
  
  while(digitalRead(BUTTON) == HIGH){
    Serial.println(digitalRead(BUTTON)); 
   //Wait for button press to initiate Beginning of Travel Code; LED LIGHT SHOULD BE GREEN while waiting
  }
  delay(1000);

  tStart = millis();
  WheelDisplay('y'); //Display Yellow

}

void loop(void){
  if((millis()-tStart) < period){ //Run for 5 minutes 
      SensorData(myFile); 
      Serial.println(digitalRead(BUTTON));
      //delay(400);
  
      if(digitalRead(BUTTON) == LOW){
        EndSensors(); 
        Serial.print("BLE TIME"); 
        WheelDisplay('b');//Display green
        delay(3000);
        // close the file:
        myFile.close();
        BLE();
        while(digitalRead(BUTTON) == HIGH); //Wait for button Press 
        WheelDisplay('r');//Display red
        Bluefruit.Advertising.stop(); //Stop the BLE from reading 
        abort(); 
      }
  }
  else{
        EndSensors(); 
        Serial.print("BLE TIME"); 
        WheelDisplay('b');//Display green
        // close the file:
        myFile.close();
        BLE();
        while(digitalRead(BUTTON) == HIGH); // Wait for button Press 
        WheelDisplay('r');//Display red
        Bluefruit.Advertising.stop(); //Stop the BLE From reading
        abort(); 
  }
}
//-------------------------------End all Sensors----------------------------
void EndSensors(void){
  return;
  
}
//-----------------------------------------------Function to Display Different Colors on the Color Wheel 
void WheelDisplay(char WheelPos){
  if(WheelPos == 'r') {
    strip.setPixelColor(0, strip.Color(255, 0, 0));
  }
  else if(WheelPos == 'g') {
    strip.setPixelColor(0, strip.Color(0, 255, 0));
  }
  else if(WheelPos == 'b'){
    strip.setPixelColor(0, strip.Color(0, 0, 255));
  }
  else if(WheelPos == 'y'){
  strip.setPixelColor(0, strip.Color(2000 * 3, 255 - 2000 * 3, 0));
  }
  
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
 Bluefruit.begin();
 
  // off Blue LED for lowest power consumption
  //Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(8);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");
 
  // Manufacturer ID is required for Manufacturer Specific Data
  beacon.setManufacturer(MANUFACTURER_ID);
 
  // Setup the advertising packet
  startAdv();
 
  Serial.println("Broadcasting beacon, open your beacon app to test");
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

void startAdv(void)
{  
  // Advertising packet
  // Set the beacon payload using the BLEBeacon class populated
  // earlier in this example
  Bluefruit.Advertising.setBeacon(beacon);
 
  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
 
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * Apple Beacon specs
   * - Type: Non connectable, undirected
   * - Fixed interval: 100 ms -> fast = slow = 100 ms
   */
  //Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_ADV_NONCONN_IND);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 160);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}
