//******seniorDesignSp2021 Sensor Suite ON Board Sensor code*********
//Pupose: Using the AdaFruit Feather NRF5240 Sense board, the on board accelerometer/gryroscope (LSM6DS33) and the temperature sensors were manipulated to display 
//specific readings. A red LED was indicated HIGH whenever the board was collecting data. 
//REV3 inclusions: More commentary within the code, different method to gather on board sensor readings by creating an array 

//--------------------------Variables and Libraries----------------------------------
#include <Adafruit_BMP280.h>
#include <LSM6.h>

Adafruit_BMP280 bmp280;   // Temperature Sensor
LSM6 lsm6ds33;   //Accelerometer and Gyroscope Sensor

float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float temperature;

char report[80]; //Array of length 80

//--------------------------------SETUP-----------------------------------------------
void setup(void) {
  
  //Initialize LED
  pinMode(LED_BUILTIN, OUTPUT);
  
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

  
}

//-----------------------------------READING VALUES----------------------------------------------
void loop(void) {
  
  while(Serial){  //If the Serial console is open, start readings
    Serial.println("Data Stream output"); //Print the beginning message upon serial opening 
    digitalWrite(LED_BUILTIN, HIGH);  //Turn LED on to indicate the start of sensors reading data
    
    //Grab temperature sensor data and convert to Fahrenheit 
    temperature = bmp280.readTemperature();
    temperature = ((9/5)*(temperature))+32; 
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" F");
  
    //Grab Accel and gyroscope raw data and convert to logical data 
    lsm6ds33.read(); //Takes a reading from both the accelerometer and gyro and stores the values in the vectors a and g
    snprintf(report, sizeof(report), "Accel: %6d %6d %6d    Gyro: %6d %6d %6d",
       lsm6ds33.a.x, lsm6ds33.a.y, lsm6ds33.a.z,
       lsm6ds33.g.x, lsm6ds33.g.y, lsm6ds33.g.z);  
    Serial.println(report);
    Serial.println();  //Print linefeed 
  
    delay(100);
  }
  
  while(!Serial){  //If the Serial console is not open, end readings 
    digitalWrite(LED_BUILTIN, LOW);  //Turn LED off to indicate the end of sensors reading data
    Serial.end();
   
  }
}
