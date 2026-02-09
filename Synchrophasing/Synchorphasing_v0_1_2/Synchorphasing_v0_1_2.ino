/*
This code will use the RPM and Phase Angle sensor on two propellers.
Synchrophasing will be used to control and match the RPM yet leave the phase angle
at a degree where the sound waves shall destructively interfere.

This means, the code will need to use control methods, will need to look at prior
knowledge for this.

 how can i make it so using karol's code better

*/

// Use RPM ANGLE SENS v1.0.2 INPUT HERE
// Just change one keep one good -- decide on which gives best data. set this as 0th element of lists CSN[0] and PWM[0]
// or change code if easier


#include <SPI.h>
#include <SD.h>


//_______________________SET UP STRUCTURE_______________________
typedef struct{
    // whats needed? CSN pins, PWN pins, phase angle, rpm
    int CSN; // RPM Output Signal
    int PWM; // ESC power pin
    unsigned int phase;
    signed int rpm;
    signed int RPMreadings[3];
}Prop;

Prop add_prop(Prop* p, int CSN, int PWM, unsigned int phase, signed int rpm, signed int RPMreadings[3]) {
    p->CSN = CSN;
    p->PWM = PWM;
    p->phase = phase;
    p->rpm = rpm;
    for(int i = 0; i < 3; i++) {
        p->RPMreadings[i] = RPMreading;
    }
}

//_________________________CALCULATE RPM_______________________
// from sensor

signed int RPMSensor(Prop* p, int CSN) {

//Send the Command Frame
    int pos[2];
    int angle[2]
    
    for(int i = 0; i<2; i++) {
        //Send the Command Frame
        digitalWrite(CSN[i], LOW);
        delayMicroseconds(1);
        SPI.transfer16(0xFFFF);
        digitalWrite(CSN[i],HIGH);
        
        //Read data frame
        digitalWrite(CSN[i], LOW);

        delayMicroseconds(1);

        angle = SPI.transfer16(0xC000);
            

        digitalWrite(CSN, HIGH);
        
        SPI.endTransaction();
        
        angle[i] = (angle & (0x3FFF)); // Get Angle

        delayMicroseconds(5);
    }
    // calc pos outside of run 
    for(int i = 0; i < 2; i++) {
    pos[i] = ( (unsigned long) angle[i])*360UL/16384UL;
    }
     // in total between read times 
      
    return (signed int) ((pos[2]-pos[1])/(0.007)); // Positions Difference / Over time taken 
    
    // unsure if this works tbh

}

signed int AVERAGERPM(Prop* p, int CSN) {
    // cant trust that the above was completely reliable especially cus unsure how quick code will run
    int rpmaverage = 0;

    for(int i = 0; i < 2; i++) {
        rpmaverage += RPMSensor(p, CSN);
        delayMicroseconds(10);
    }
    return (rpmaverage)/2;
}










signed int ESCtoRPM_readings[3](Prop* p, int CSN, int PWM) { // Returns RPMreadings[3] for each prop
    // This converts the ESC to RPM using the readings of the specific Motor
    // Initialises at 3 points to get a rough linear approximation for the ESC to RPM
    // Take three Throttle values between 1000 and 2000 (min and max PWM for )
    
    // Using values 1200, 1500, 1800
    int PWMvalues[3] = {1200, 1500, 1800};
    signed int Readings[3];

    for(int i = 0; i < 3; i++){
        
        digitalWrite(PWM, PWMvalues[i]);
        delay(1000); // delay for a second for warm up
        Readings[i] = AverageRPM(p, CSN) // (calling function for finding RPM);
    }
    return Readings[3];


}

void setup() {
    Serial.begin(9600);

    // INITIALISATION OF PROPS 
    Prop props[2];
    // Set CSN, PWM pins, and rpms and phase to start at zero;
    add_prop(&props[0], 10, 11, 0, 0, {0, 0, 0});
    add_prop(&props[0], 15, 16, 0, 0, {0, 0, 0});
    

    // SETUP Pins

    
    for(int i = 0; i < 2, i++>) {
        //Set Pin Modes
        pinMode(props[i].CSN, OUTPUT);
        pinMode(props[i].PWM, OUTPUT);

        //Set Slave Select High to Start i.e disable chip
        digitalWrite(props[i].CSN, HIGH);
    }
 
    //Initialize SPI 
    SPI.begin();
    //Initialise SD Card
    SD.begin();

    SD.exists("datalog.txt");
    datalog = SD.open("datalog.txt", FILE_WRITE);

    // Initialise RPM and ESC Conversion for Calc later
    for(int i = 0; i< 2; i++) {
        for(int j = 0; i<3; j++){
            props[i].RPMreadings[j] = ESCtoRPM_readings[j](&props[i], &props[i].CSN, &props[i].PWM)
        }
    }
    


}
/*
void loop() {
    

    for(int i = 1; i<2; i++) {
    
        while( rpm[0] <  1250) { // check rpm is atleast above a certain value first
            while(rpm[i] < (rpm[0] - 30) && rpm[i] > (rpm[0] + 30)) {
                print()
                // slow down the motor using esc code
                // Change PWM SENSOR SIGNAL - Servo.write()
                // can we look at arduino code? - this is gonna be on there
                // NEEDS to be < 1Hz out (60rpm)
                if(rpm[i] < (rpm[0] - 30)) {
                    PWM[i] += 10; 
                    /*whatever this uses change by incremental steps, preferably large enough
                    that it doesnt take forever, BUT not too big that it can not fine adjust within.
                    
                    
                    
                    
                }
                
                else if(rpm[i] > (rpm[0] + 30)) {
                    PWM[i] -= 10; 
                }


            }


        }
    
    }



}
*/