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


/*
This section is for each error means in the Serial Print, just to make it easier to find

Error 101 - PWMfromRPM Function
Error 102 - void setup - RPM Prop initialisation
Error 103 - void loop - Decrease Feedback - Case 3 
Error 104 - Wrong Feedback Selected

*/



#include <SPI.h>
#include <SD.h>
#include <Servo.h>



//_______________________SET UP STRUCTURE_______________________
typedef struct{
    // whats needed? CSN pins, PWN pins, phase angle, rpm
    Servo motor;
    int CSN; // RPM Output Signal
    int PWM; // ESC power pin
    int phase; // phase of current propeller, to check the phase and then ensure that they are out of phase
    unsigned int rpm; // rpm currently, this will be to synchronise the RPMS of each propeller
    unsigned int RPMreadings[3]; // This will give each propeller its own ESC to RPM Reading conversion, 
                                 // we can use this to see how much each Prop will need its power increased by
                                 // (or decreased) for the RPMs to match.
    int grad; // Gradient for each prop worked out from RPM Readings and ESC Values
}Prop;

void add_prop(Prop* p, Servo motor, int CSN, int PWM, int phase, unsigned int rpm, unsigned int RPMreadings[3], int grad) {
    p->motor = motor;
    p->CSN = CSN;
    p->PWM = PWM;
    p->phase = phase;
    p->rpm = rpm;
    for(int i = 0; i < 3; i++) {
        p->RPMreadings[i] = RPMreadings[i];
    }
    p->grad = grad;
    
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
        Serial.print("angle pre");
        Serial.println(angle[i]);

        digitalWrite(CSN, HIGH);
        
        SPI.endTransaction();
        
        angle[i] = (angle[i] & (0x3FFF)); // Get Angle
        
        delayMicroseconds(5);
        Serial.print("angle post");
        Serial.println(angle[i]);
    }
    unsigned long CurrentTime = micros();
    unsigned long ElapsedTime = (CurrentTime - StartTime - 5); // minus extra 5 from end delay
    // calc pos outside of run 
    for(int i = 0; i < 2; i++) {
        pos[i] = ( (unsigned long) angle[i])*360UL/16384UL;
        Serial.print("pos");
        Serial.println(pos[i]);
    }
    
    
     // in total between read times 
        
    Serial.println(((pos[1]-pos[0])/(ElapsedTime)) * 166670);
    return ((pos[1]-pos[0])/(ElapsedTime)) * 166670; // Positions Difference / Over time taken 
    
    // unsure if this works tbh

}

unsigned int AverageRPM(Prop* p, int CSN) {
    // cant trust that the above was completely reliable especially cus unsure how quick code will run
    int rpmaverage = 0;

    for(int i = 0; i < 2; i++) {
        rpmaverage += RPMSensor(p, CSN);
        delayMicroseconds(100);
        Serial.println((rpmaverage));
    }
    Serial.println((rpmaverage)/2);
    return (rpmaverage)/2;
}






unsigned int PWMtoRPM_readings(Prop* p, Servo motor, int CSN, int PWM, int PWMVal) { // Returns RPMreading for each prop and PWMValue
    // This converts the ESC to RPM using the readings of the specific Motor
    // Initialises at 3 points to get a rough linear approximation for the PWM to RPM
    // Take three Throttle values between 1000 and 2000 (assumed min and max PWM for this motor, typical for industry)
    
    // Using values 1200, 1500, 1800
    
    unsigned int Reading;

    delay(1000); // delay for a second for warm up
    Reading = AverageRPM(p, CSN); // (calling function for finding RPM);

    return (Reading);


}

int PWM_RPMgradient(Prop* p, int PWMVals[3], int RPMVals[3]){
    int grad = 0;
    for(int i = 0; i < 2; i++){
        grad += ((RPMVals[i+1])-(RPMVals[i]))/((PWMVals[i+1])-(PWMVals[i]));
    }
    return grad/2;

}


unsigned int PWMfromRPM(Prop* p, int CSN, int PWM, int RPM, int gradient){
    /* This function will call the RPM readings and PWMVals used to create a correlation between them.
    By doing this we are assuming that it is a linear Correlation between the RPM and PWM.
    From just checking it seems like the case with a few outliers, but either way this should give
    a base value that we can then edit to get the right amount later in the Feedback Loop program.
    */
    // The Gradient is worked out in Setup using equation above
    int PWMValue = 0;
    // We assume x intercept is at 1000 with this set up, No power should not Spin the RPM,- No power is at 1000
    // So y = 0 at x = 1000, then using grad, yintercept C:
    int yintercept = 0 - (1000 * gradient);

    // Now we have the full linear equation y = mx + C

    PWMValue = gradient * RPM + yintercept;
    
    if(!(PWMValue > 1000 && PWMValue < 2000)){
        Serial.println("Error 101 - PWM Value not calculated correctly, please check code");
        return 0;
    }
    else {
        return PWMValue; // Value outputted should be between 1000 and 2000
    }
}


Prop props[2];
Servo motor, motor1;

void setup() {
    Serial.begin(9600);
    
    // INITIALISATION OF PROPS 
    
    // Set CSN, PWM pins, and rpms and phase to start at zero;
    unsigned int RPMReads[3] = {0,0,0}; // start all at 0 
    add_prop(&props[0], motor1, 7, 9, 0, 0, RPMReads, 0);
    //add_prop(&props[1], 2, 3, 0, 0, RPMReads, 0);
    // A reminder that the structure is PROP(motor, CSN,PWM,current phase, current rpm, RPM Readings, and Gradient of ESC to RPM )
    motor1.attach(props[0].PWM, 1000, 2000);

    // SETUP Pins

    for(int i = 0; i < 1; i++) {
        //Set Pin Modes
        pinMode(props[i].CSN, OUTPUT);
        

        //Set Slave Select High to Start i.e disable chip
        digitalWrite(props[i].CSN, HIGH);
        motor1.writeMicroseconds(1000);
        delay(50);

    }
 
    //Initialize SPI 
    SPI.begin();
    //Initialise SD Card
    SD.begin();
    // Make Data file if not already existing not done yet
    SD.exists("datalog.txt");
    //datalog = SD.open("datalog.txt", FILE_WRITE);
    
    // Initialise RPM and PWM Conversion for Calc later
    int j = 0;
    
    Serial.println("Initialisation is running. Please don't adjust anything.");
    
    for(int i = 0; i < 1; i++) {
        int PWMvalues[3] = {1070, 1080, 1090}; // change this as see fit, more PWM values will be good but make sure to change gradient code
        
        for(j = 0; j < 3; j++){
            motor1.writeMicroseconds(PWMvalues[j]);
            unsigned int PWMtoRPM = PWMtoRPM_readings(&props[i], props[i].motor, props[i].CSN, props[i].PWM, PWMvalues[j]);
            props[i].RPMreadings[j] = PWMtoRPM;

            if(PWMtoRPM == 0) {
                // Check if RPM Sensors areworking for these values
                Serial.println("Error 102 - The RPM recorded is zero for these PWM values, please check the RPM sensors or code is working");
            }
            else {
                Serial.println("Initialisation is now running. Please don't adjust anything.");
                Serial.print("The RPM for Prop ");
                Serial.print(i);
                Serial.print(" at PWM Value "); // i hate how this works
                Serial.print(PWMvalues[j]);
                Serial.print(" is ");
                Serial.println(PWMtoRPM);
            }
            delay(5000);
        }
        
        
    }


    Serial.println("ALL RPM to PWM Readings have now been measured, these will be used for conversions");

}

void loop() {
    //_____________________________WHAT RPM AND PHASE DIFF DO YOU WANT________________________________
    // This will set the RPM for all motors, and the phase difference required between
    // __RPM__
    int SETRPM = 2000; // This will start by setting the ESC/PWM value required for the RPM in MOTOR 1
    int i;
    // Therefore required PWM value for each prop can be calculated and set
    for(i = 0; i < 2;  i++) {
        int PWMVal = PWMfromRPM(&props[i], props[i].CSN, props[i].PWM, SETRPM, props[i].grad);
        digital.Write(props[i].PWM, PWMVal);
    }




    // Golden Search Ratio
    // go 0.33 and 0.66 ahead to then calc where the bracket would be inside those ranges
    //

    


}





/*
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