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


//Set Slave Select Pin
//MOSI, MISO, CLK are handeled automatically
int CSN = 10;
int SO = 12;
int SI = 11;
int CLK = 13 ;
unsigned int angle;

void setup() {
    A7AA-4FEE
  Serial.begin(9600);

  //Set Pin Modes
  pinMode(CSN, OUTPUT);
  pinMode(SI, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(CLK, OUTPUT);
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
  
  //Send the Command Frame
  digitalWrite(CSN, LOW);
  delayMicroseconds(1);
  SPI.transfer16(0xFFFF);
  digitalWrite(CSN,HIGH);

  //Read data frame
  digitalWrite(CSN, LOW);
  delayMicroseconds(1);
  angle = SPI.transfer16(0xC000);
  digitalWrite(CSN, HIGH);
  SPI.endTransaction();

  angle = (angle & (0x3FFF));
  
  int pos = ( (unsigned long) angle)*360UL/16384UL;

  Serial.println(pos);
  datalog.println(pos, angle);

  delay(1000);

}