// For function sprintf
#include <stdio.h>

// Variables to collect RPM sensors rising:
volatile unsigned int front1Ticker = 0;
volatile unsigned int front2Ticker = 0;
volatile unsigned int rear1Ticker = 0;
volatile unsigned int rear2Ticker = 0;

// Variables used to store calculated RPM values:
volatile unsigned int frontRPM = 0;
volatile unsigned int rearRPM = 0;

// Count of magnetic poles on the RPM sensor:
float magnetPoles = 12.0;

// Pins for the RPM Sensor
int frontRPM1pin = 2;
int frontRPM2pin = 3;
int rearRPM1pin = 18;
int rearRPM2pin = 19;

// Set up the built-in LED:
int LEDpin = LED_BUILTIN;
bool ledState = true;

// Time related variables to trigger RPM calculations:
unsigned long t_step = 50;
unsigned long currentTime = millis();
static long previousTime = millis();

// Time step to print to serial:
unsigned long t_print = 1000;
static long printTime = millis();

// Debug variables:
unsigned long loopCount = 0;
char tbs[22];

void setup() {
    // Start the serial bus and pause:
    Serial.begin(115200); 
    delay(400);

    // Set up the built-in LED:
    pinMode(LEDpin, OUTPUT);
    digitalWrite(LEDpin, ledState);

    // Set up the RPM Sensors:
    pinMode(frontRPM1pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(frontRPM1pin), frontRPM1_Interupt, CHANGE);
    pinMode(frontRPM2pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(frontRPM2pin), frontRPM2_Interupt, CHANGE);
    pinMode(rearRPM1pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rearRPM1pin), rearRPM1_Interupt, CHANGE);
    pinMode(rearRPM2pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rearRPM2pin), rearRPM2_Interupt, CHANGE);
}

void loop() {
    currentTime = millis();
    if ( currentTime - previousTime > t_step ) {
        frontRPM = ((front1Ticker + front2Ticker)/magnetPoles)*(1000.0/(currentTime - previousTime))*60.0;
        front1Ticker = 0;
        front2Ticker = 0;

        rearRPM = ((rear1Ticker + rear2Ticker)/magnetPoles)*(1000.0/(currentTime - previousTime))*60.0;
        rear1Ticker = 0;
        rear2Ticker = 0;

        loopCount++;
        ledState = !ledState;
        digitalWrite(LEDpin, ledState);
        previousTime = millis();
    }

    if ( currentTime - printTime > t_print )  {
        sprintf(tbs, "F: %06d | R: %06d\n", frontRPM, rearRPM);
        Serial.print(tbs);
        printTime = millis();
    }
}

void frontRPM1_Interupt() {
  front1Ticker++;
}

void frontRPM2_Interupt() {
  front2Ticker++;
}

void rearRPM1_Interupt() {
  rear1Ticker++;
}

void rearRPM2_Interupt() {
  rear2Ticker++;
}
