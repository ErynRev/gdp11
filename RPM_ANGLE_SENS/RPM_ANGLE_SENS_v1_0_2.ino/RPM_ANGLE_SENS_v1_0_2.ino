/*AMS Rotary Sensor AS5147
 Measures absolute angle position referenced at a set NORTH

 Circuit
 UNO: MOSI pin 11
      MISO pin 12 
      CLK  pin 13
      CSN  pin 10

 Mega: MOSI pin 51
       MISO pin 50 
       CLK  pin 52
       CSN  pin 53  



TEENSY 4.1 -- Hall Effect RPM sensor
TO DO:
--  Add SD card writer 
--  Copy and redo for all 4 motors


Created by @CheeseDontDie on arduino.cc
Modified by Eryn


 */

#include <SPI.h>
#include <SD.h>

File datalog;


/*
Pins on Teensy Board for 2 propellers
Set Slave Select Pin
MOSI, MISO, CLK are handeled automatically
Pins can be changed for whatever -- best to keep together
*/

/* .Prop Example
int CSN = 10;
int SO = 12;
int SI = 11;
int CLK = 13 ;
*/

//Prop Wiring List
// Number of Props used 
// int n = 2;


int CSN[2] = {10, 14};
int SO[2] = {12, 16};
int SI[2] = {11, 15};
int CLK[2] = {13, 17};


unsigned int angle[2];

void setup() {
  //A7AA-4FEE
  Serial.begin(9600);
  
  //Set Pin Modes
  for(int i = 0; i < 2; i++) {
    pinMode(CSN[i], OUTPUT);
    pinMode(SI[i], OUTPUT);
    pinMode(SO[i], INPUT);
    pinMode(CLK[i], OUTPUT);
  }
  
  //Set Slave Select High to Start i.e disable chip
  digitalWrite(CSN, HIGH);
  //Initialize SPI 
  SPI.begin();
  //Initialise SD Card
  SD.begin();

  SD.exists("datalog.txt");
  datalog = SD.open("datalog.txt", FILE_WRITE);

}

void loop() {

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
  int pos[2];
 
  
  
  for(int i = 0; i < 2; i++) {
    //Send the Command Frame
    digitalWrite(CSN[i], LOW);
    delayMicroseconds(1);
    SPI.transfer16(0xFFFF);
    digitalWrite(CSN[i],HIGH);
  
    //Read data frame
    digitalWrite(CSN[i], LOW);
    delayMicroseconds(1);

    
    angle[i] = SPI.transfer16(0xC000);
    

    digitalWrite(CSN[i], HIGH);
  
    SPI.endTransaction();
  
  
    angle[i] = (angle[i] & (0x3FFF));
    

    
    pos[i] = ( (unsigned long) angle[i])*360UL/16384UL;
    

    Serial.println(pos[i]);
    datalog.println(pos[i], angle[i]);
  }

  delay(1000);

}


