//******seniorDesignSp2021 Sensor Suite ON Board Sensor code*********
//Pupose: Using the AdaFruit Feather NRF5240 Sense board, the on board accelerometer/gryroscope (LSM6DS33) and the temperature sensors were manipulated to display 
//specific readings. A red LED was indicated HIGH whenever the board was collecting data. 
//Additional offboard sensors to obtain the turbidity, salinity and temperature of a water sample
//REV3 inclusions: More commentary within the code, different method to gather on board sensor readings by creating an array 
//REV4 inclusions: Integration of offboard sensor code into overall code as well as additional comments.
//--------------------------Variables and Libraries----------------------------------
//Libraries to be included
#include <Adafruit_BMP280.h> //Library for the Temperature, Pressure and Altitude sensors (Only using temperature sensor)
#include <OneWire.h>  //Library that allows communication to OneWire devices (DS18B20 in our case)
#include <DallasTemperature.h>//Library containing temperature functions related to Maxim Temperature IC's
#include <LSM6.h>

// Data wire is plugged into pin 5 on the Adafruit Feather 
#define ONE_WIRE_BUS 5
#define TdsSensorPin A0
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point

Adafruit_BMP280 bmp280;   // Temperature Sensor
LSM6 lsm6ds33;   //Accelerometer and Gyroscope Sensor


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


float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float intemp;

char report[80]; //Array of length 80

//--------------------------------SETUP-----------------------------------------------
void setup(void) {
  
  //Initialize LED
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TdsSensorPin,INPUT);
  
  //Set up serial communication 
  Serial.begin(115200); 
  while(!Serial)  //If the Serial console is not open, wait
    delay(10); 

  //Initialization of sensors 
  bmp280.begin(); //temperature sensor  
    
  //Ensure that the gyroscope and accelerometer sensor has been detected
  //If not been detected, then wait for detection
  if(!lsm6ds33.init())
  {
    Serial.println("Failed to detect and initilize LSM6DS33!");
    while(1)
      delay(10);  
  }
  lsm6ds33.enableDefault(); 
  sensors.begin();
  
}

//-----------------------------------READING VALUES----------------------------------------------
void loop(void) {
  
  while(Serial){  //If the Serial console is open, start readings   
    readTDS();
    readTurbidity();
        
    Serial.println("Data Stream output"); //Print the beginning message upon serial opening 
    digitalWrite(LED_BUILTIN, HIGH);  //Turn LED on to indicate the start of sensors reading data
    
    //Grab temperature sensor data and convert to Fahrenheit 
    intemp = bmp280.readTemperature();
    intemp = ((9/5)*(intemp))+32; 
    Serial.print("Temperature: ");
    Serial.print(intemp);
    Serial.println(" F");
  
    //Grab Accel and gyroscope raw data and convert to logical data 
    lsm6ds33.read(); //Takes a reading from both the accelerometer and gyro and stores the values in the vectors a and g
    snprintf(report, sizeof(report), "Accel: %6d %6d %6d    Gyro: %6d %6d %6d",
       lsm6ds33.a.x, lsm6ds33.a.y, lsm6ds33.a.z,
       lsm6ds33.g.x, lsm6ds33.g.y, lsm6ds33.g.z);  
    Serial.println(report);
    Serial.println();  //Print linefeed 

    readTemperatureSensor();  
    delay(500); //Wait a certain amount of milliseconds until next loop
  }
  
  while(!Serial){  //If the Serial console is not open, end readings 
    digitalWrite(LED_BUILTIN, LOW);  //Turn LED off to indicate the end of sensors reading data
    Serial.end();
   
  }
}

//Function to read and convert the value from the TDS sensor
void readTDS(void)
{
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
    {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
    } 
}

//Function to read and convert the value from the Turbidity sensor
//Sensor is hooked up to pin A1 on the Adafruit Feather
void readTurbidity(void)
{
    int sensorValue = analogRead(A1);// read the input on analog pin 1:
    float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float turbidity = ((-1120*(voltage * voltage)) + (5742.3 * voltage) - 4352.9);
    Serial.println(voltage); // print out the value you read:
    Serial.println("V");
    Serial.println(turbidity);
    Serial.println("NTU");
}

//Function to read and convert the value obtained from the offboard temperature sensor
void readTemperatureSensor(void)
{
    Serial.print(" Requesting temperatures..."); 
    sensors.requestTemperatures(); // Send the command to get temperature readings 
    Serial.println("DONE"); 
    /********************************************************************/
    Serial.print("Temperature is: "); 
    Serial.println(sensors.getTempCByIndex(0)); // Why "byIndex"?  
    // You can have more than one DS18B20 on the same bus.  
    // 0 refers to the first IC on the wire 
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
