//******seniorDesignSp2021 Sensor Suite ON Board Sensor code*********
//Pupose: Using the AdaFruit Feather NRF5240 Sense board, the on board accelerometer/gryroscope (LSM6DS33) and the temperature sensors were manipulated to display 
//specific readings. A red LED was indicated HIGH whenever the board was collecting data. 
//REV4 inclusions: Integration of the SD Cardd

//--------------------------Variables and Libraries----------------------------------
#include <SD.h>
 
File myFile;
SdFile root; 
const int chipSelect = 6; 

void setup()
{
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
  
   pinMode(6, OUTPUT);
 
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);
 
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
  // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}
 
void loop()
{
  // nothing happens after setup
}
