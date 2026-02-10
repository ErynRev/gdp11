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
    int phase; // phase of current propeller, to check the phase and then ensure that they are out of phase
    unsigned int rpm; // rpm currently, this will be to synchronise the RPMS of each propeller
    unsigned int RPMreadings[3]; // This will give each propeller its own ESC to RPM Reading conversion, 
                                 // we can use this to see how much each Prop will need its power increased by
                                 // (or decreased) for the RPMs to match.
    int grad; // Gradient for each prop worked out from RPM Readings and ESC Values
}Prop;

void add_prop(Prop* p, int CSN, int PWM, int phase, unsigned int rpm, unsigned int RPMreadings[3], int grad) {
    p->CSN = CSN;
    p->PWM = PWM;
    p->phase = phase;
    p->rpm = rpm;
    for(int i = 0; i < 3; i++) {
        p->RPMreadings[i] = RPMreadings[i];
    }
    p->grad = grad
    
}

//_________________________CALCULATE RPM_______________________
// from sensor

unsigned int RPMSensor(Prop* p, int CSN) {

//Send the Command Frame
    int pos[2] = {0,0};
    int angle[2] = {0,0};
    unsigned long StartTime = micros();
    
    for(int i = 0; i<2; i++) {
        //Send the Command Frame
        digitalWrite(CSN, LOW);
        delayMicroseconds(1);
        SPI.transfer16(0xFFFF);
        digitalWrite(CSN,HIGH);
        
        //Read data frame
        digitalWrite(CSN, LOW);

        delayMicroseconds(1);

        angle[i] = SPI.transfer16(0xC000);
        

        digitalWrite(CSN, HIGH);
        
        SPI.endTransaction();
        
        angle[i] = (angle[i] & (0x3FFF)); // Get Angle

        delayMicroseconds(5);
    }
    unsigned long CurrentTime = micros();
    unsigned long ElapsedTime = (CurrentTime - StartTime - 5); // minus extra 5 from end delay
    // calc pos outside of run 
    for(int i = 0; i < 2; i++) {
    pos[i] = ( (unsigned long) angle[i])*360UL/16384UL;
    }
     // in total between read times 
        
    return (signed int) ((pos[1]-pos[0])/(ElapsedTime)); // Positions Difference / Over time taken 
    
    // unsure if this works tbh

}

unsigned int AverageRPM(Prop* p, int CSN) {
    // cant trust that the above was completely reliable especially cus unsure how quick code will run
    int rpmaverage = 0;

    for(int i = 0; i < 2; i++) {
        rpmaverage += RPMSensor(p, CSN);
        delayMicroseconds(10);
    }
    return (rpmaverage)/2;
}










unsigned int ESCtoRPM_readings(Prop* p, int CSN, int PWM, int PWMVal) { // Returns RPMreading for each prop and PWMValue
    // This converts the ESC to RPM using the readings of the specific Motor
    // Initialises at 3 points to get a rough linear approximation for the ESC to RPM
    // Take three Throttle values between 1000 and 2000 (min and max PWM for )
    
    // Using values 1200, 1500, 1800
    
    unsigned int Reading;

    digitalWrite(PWM, PWMVal);
    delay(1000); // delay for a second for warm up
    Reading = AverageRPM(p, CSN); // (calling function for finding RPM);

    return (Reading);


}


unsigned int ESCfromRPM(Prop* p, int CSN, int PWM, int grad){
    /* This function will call the RPM readings and PWMVals used to create a correlation between them.
    By doing this we are assuming that it is a linear Correlation between the RPM and PWM.
    From just checking it seems like the case with a few outliers, but either way this should give
    a base value that we can then edit to get the right amount later in the Feedback Loop program.
    */
    // The Gradient is worked out in the Setup Fn
    

}

void setup() {
    Serial.begin(9600);

    // INITIALISATION OF PROPS 
    Prop props[2];
    // Set CSN, PWM pins, and rpms and phase to start at zero;
    unsigned int RPMReads[3] = {0,0,0}; // start all at 0 
    add_prop(&props[0], 10, 11, 0, 0, RPMReads, 0);
    add_prop(&props[1], 15, 16, 0, 0, RPMReads, 0);
    // A reminder that the structure is PROP(CSN,PWM,current phase, current rpm, RPM Readings, and Gradient of ESC to RPM )

    // SETUP Pins

    
    for(int i = 0; i < 2; i++) {
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
    //datalog = SD.open("datalog.txt", FILE_WRITE);
    
    // Initialise RPM and ESC Conversion for Calc later
    Serial.println("Initialisation is now running. Please don't adjust anything.")
    for(int i = 0; i < 2; i++) {
        int PWMvalues[3] = {1200, 1500, 1800};
        
        for(int j = 0; j < 3; j++){
            unsigned int ESCtoRPM = ESCtoRPM_readings(&props[i], props[i].CSN, props[i].PWM, PWMvalues[j]);
            props[i].RPMreadings[j] = ESCtoRPM;
            if(ESCtoRPM == 0) {
                // Check if RPM Sensors areworking for these values
                Serial.println("The RPM recorded is zero for these ESC values, please check the RPM sensors or code is working");
            }
            else {
                Serial.print("The RPM for Prop");
                Serial.print(i);
                Serial.print("at PWM Value"); // i hate how this works
                Serial.print(PWMvalues[j]);
                Serial.print("is");
                Serial.println(ESCtoRPM);
            }
        }
    }
    Serial.println("RPM to ESC Readings have now been measured, this will be used for conversions")

}

void loop() {
    //_____________________________WHAT RPM AND PHASE DIFF DO YOU WANT________________________________
    // This will set the RPM for all motors, and the phase difference required between
    // __RPM__
    int SETRPM = 1200; // This will start by setting the ESC/PWM value required for the RPM in MOTOR 1







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
                    // whatever this uses change by incremental steps, preferably large enough
                    // that it doesnt take forever, BUT not too big that it can not fine adjust within.
                    
                    
                    
                    
                }
                
                else if(rpm[i] > (rpm[0] + 30)) {
                    PWM[i] -= 10; 
                }


            }


        }
    
    }



}
*/