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






unsigned int PWMtoRPM_readings(Prop* p, int CSN, int PWM, int PWMVal) { // Returns RPMreading for each prop and PWMValue
    // This converts the ESC to RPM using the readings of the specific Motor
    // Initialises at 3 points to get a rough linear approximation for the PWM to RPM
    // Take three Throttle values between 1000 and 2000 (assumed min and max PWM for this motor, typical for industry)
    
    // Using values 1200, 1500, 1800
    
    unsigned int Reading;

    digitalWrite(PWM, PWMVal);
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
        Serial.println("PWM Value not calculated correctly, please check code");
        return 0;
    }
    else {
        return PWMValue; // Value outputted should be between 1000 and 2000
    }
}


Prop props[2];


void setup() {
    Serial.begin(9600);

    // INITIALISATION OF PROPS 
    
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
    // Make Data file if not already existing not done yet
    SD.exists("datalog.txt");
    //datalog = SD.open("datalog.txt", FILE_WRITE);
    
    // Initialise RPM and PWM Conversion for Calc later
    Serial.println("Initialisation is now running. Please don't adjust anything.");
    for(int i = 0; i < 2; i++) {
        int PWMvalues[3] = {1200, 1600, 2000}; // change this as see fit, more PWM values will be good but make sure to change gradient code
        
        for(int j = 0; j < 3; j++){
            unsigned int PWMtoRPM = PWMtoRPM_readings(&props[i], props[i].CSN, props[i].PWM, PWMvalues[j]);
            props[i].RPMreadings[j] = PWMtoRPM;
            if(PWMtoRPM == 0) {
                // Check if RPM Sensors areworking for these values
                Serial.println("The RPM recorded is zero for these PWM values, please check the RPM sensors or code is working");
            }
            else {
                Serial.print("The RPM for Prop");
                Serial.print(i);
                Serial.print("at PWM Value"); // i hate how this works
                Serial.print(PWMvalues[j]);
                Serial.print("is");
                Serial.println(PWMtoRPM);
            }
        }
        
        
    }


    Serial.println("ALL RPM to PWM Readings have now been measured, these will be used for conversions");

}

void loop() {
    
    int i = 0; // gonna use i a lot so may as well initialise now  
    int n = 0; // number of propellers needed to be corrected, if motor 1 is off SetRPM we can increase this one first. 
        // Alternatively we can Base off MOTOR 1s rpm instead.
    int j = 0;

    //_____________________________WHAT RPM AND PHASE DIFF DO YOU WANT________________________________
    // This will set the RPM for all motors, and the phase difference required between
    // __RPM__
    int SETRPM = 2000; // This will start by setting the ESC/PWM value required for the RPM in MOTOR 1
    int PWMVal[2] = {0,0}; // current PWM Values

    // make a new 2 int array with the current RPM and PWM
    int RPMarray[2] = {0 , 0};
    int PWMarray[2] = {1000, 1000}; // 1000 again is minimum

    int RPMdiff = 0; // RPM difference from SETRPM and current RPM

    int goldRPM[2] = {0, 0}; // golden ratio RPMs
    int goldPWM[2] = {0, 0}; // golden ratio PWMs
    int goldDiff[2] = {0, 0};

    int grad = 0; // gradient


    // Therefore required PWM value for each prop can be calculated and set
    for(int i = 0; i < 2;  i++) {
        PWMVal[i] = PWMfromRPM(&props[i], props[i].CSN, props[i].PWM, SETRPM, props[i].grad);
        digitalWrite(props[i].PWM, PWMVal[i]);
        delay(5000); // delay 5 seconds, allow time for prop to get up to speed
        props[i].rpm = AverageRPM(&props[i], props[i].CSN);
        // Print off what RPM value it is
        Serial.print("Prop ");
        Serial.print(i);
        Serial.print(" has RPM:");
        Serial.println(props[i].rpm);
    }


    while(PWMVal[0] > 1000) { // Make sure theyre actually running
        if(!(props[0].rpm > (SETRPM + 200 ) && props[0].rpm < (SETRPM - 200))) { // Checks how close Motor 1 rpm is, if 200, then doesnt 
        // run control for MOTOR 1 - Just a side thing to ensure getting the closest value.

            n = 1;
            SETRPM = props[0].rpm;
        }
        
        for(i = n; i < 2;  i++) {
            /*checks to see if rpm is right or atleast within that zone, this is most likely to happen as
            that linear assumption of PWM to RPM is not likely to be correct.
            To fix this, we could set a new gradient of the current RPM and ESC Setting with a previous one,
            In essence, we would be constantly correcting the linearity and increasing (or decreasing) until
            the RPM was within 30.

            We will use a golden search ratio, i.e. checking 0.33 and 1 (or 0.67 from 0.33) along the line 
            from the new point and intercept. 

            (this new line will have a new gradient) -> this will then give us two more points
            which will depend on if the RPM is within the 0 - 0.33 or 0.33 - 1(current as 0 and 1 as M1 RPM)
            
            continue the same now with these values - constantly searching the golden ratio till either
            bracket falls into the +-30RPM (+-0.5Hz)
            
            */
            
            while(!(props[i].rpm > (SETRPM + 30) && props[i].rpm < (SETRPM - 30))) {
                if(props[i].rpm > (SETRPM + 30)){
                    RPMArray[0] = SETRPM;
                    RPMArray[1] = props[i].rpm

                    //RPMDifference
                    RPMdiff = (props[i].rpm - SETRPM + 30);


                    // decrease
                    
                    grad = ((RPMarray[1])-(RPMarray[0]))/((PWMarray[1])-(PWMarray[0]));


                    //0.33
                    goldRPM[0] = props[i].rpm - 0.33 * RPMdiff;
                    goldPWM[0] = PWMfromRPM(&props[i], props[i].CSN, props[i].PWM, goldRPM[1], grad);


                    //0.67
                    goldRPM[1] = props[i].rpm - 0.67 * RPMdiff;
                    goldPWM[1] = PWMfromRPM(&props[i], props[i].CSN, props[i].PWM, goldRPM[2], grad);

                    for(i = 0; i < 2; i++){
                        digitalWrite(props[i].PWM, goldPWM[i]);
                        delay(500);
                        goldRPM[i] = AverageRPM(&props[i], props[i].CSN);
                        goldDiff[i] = SETRPM - goldRPM;
                        if(goldDiff[i] < 0) {
                            j += 1;
                            goldDiff[i] *= -1;
                        }
                        delay(10);
                    
                    }
                  
                    
                    if(goldDiff[1] < goldDiff[0]) {
                        RPMArray[1] = goldRPM[0];
                        
                        if(j == 1){
                            RPMarray[0] = goldRPM[1];
                        }
                    
                    }

                    else if(goldDiff[0] < goldDiff[1]) {
                        RPMArray[1] = goldRPM[1];
                        if(j == 1){
                            RPMArray[0] = goldRPM[0];
                        }

                    }

                    
                    j = 0;
                
                }

                else if(props[i].rpm < (SETRPM-30)) {
                    // increase

                    //RPMDifference
                    RPMdiff = (props[i].rpm - SETRPM+30);


                    // increase
                    RPMarray[1] = props[i].rpm;
                    grad = ((RPMarray[1])-(RPMarray[0]))/((PWMarray[1])-(PWMarray[0]));
                    

                    //0.33
                    goldRPM[0] = props[i].rpm - 0.33 * RPMdiff;
                    goldPWM[0] = PWMfromRPM(&props[i], props[i].CSN, props[i].PWM, goldRPM[1], grad);


                    //0.67
                    goldRPM[1] = props[i].rpm - 0.67 * RPMdiff;
                    goldPWM[1] = PWMfromRPM(&props[i], props[i].CSN, props[i].PWM, goldRPM[2], grad);

                    for(i = 0; i < 2; i++){
                        digitalWrite(props[i].PWM, goldPWM[i]);
                        delay(500);
                        goldRPM[i] = AverageRPM(&props[i], props[i].CSN);
                        goldDiff[i] = SETRPM - goldRPM;
                        if(goldDiff[i] < 0) {
                            j += 1;
                            goldDiff[i] *= -1;
                        }
                        delay(10);
                    
                    }
                  
                    
                    if(goldDiff[1] < goldDiff[0]) {
                        
                        RPMArray[1] = goldRPM[1];
                        if(j == 1){
                            RPMarray[1] = goldRPM[0]; 
                            RPMarray[0] = goldRPM[1];
                        }
                    
                    }

                    else if(goldDiff[0] < goldDiff[1]) {
                        RPMArray[0] = goldRPM[1];
                        if(j == 1){
                            RPMArray[1] = goldRPM[1];
                        }

                    }

                    
                    j = 0;
                



                }
                
            }
        }
    }
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