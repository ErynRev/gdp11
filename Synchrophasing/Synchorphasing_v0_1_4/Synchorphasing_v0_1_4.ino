
#include <SPI.h>
#include <SD.h>
#include <Servo.h>



//_______________________SET UP STRUCTURE_______________________
typedef struct{
    // whats needed? CSN pins, PWN pins, phase angle, rpm
    int CSN; // RPM Output Signal
    int PWM; // ESC power pin
    int phase; // phase of current propeller, to check the phase and then ensure that they are out of phase
    float rpm; // rpm currently, this will be to synchronise the RPMS of each propeller
    float RPMreadings[3]; // This will give each propeller its own ESC to RPM Reading conversion, 
                                 // we can use this to see how much each Prop will need its power increased by
                                 // (or decreased) for the RPMs to match.
    int grad; // Gradient for each prop worked out from RPM Readings and ESC Values
}Prop;

void add_prop(Prop* p, int CSN, int PWM, int phase, float rpm, float RPMreadings[3], int grad) {
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

uint16_t getAngle(Prop* p, int CSN) {
    uint16_t angle;

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // start slow

    digitalWrite(CSN, LOW);
    delayMicroseconds(2);
    SPI.transfer16(0xFFFF);         // command frame for angle (0x3FFF read)
    digitalWrite(CSN, HIGH);

    delayMicroseconds(2);

    digitalWrite(CSN, LOW);
    delayMicroseconds(2);
    angle = SPI.transfer16(0x0000); // read response
    digitalWrite(CSN, HIGH);

    SPI.endTransaction();

    return angle & 0x3FFF; // 14-bit angle

}


float RPMSensor(Prop* p, int CSN) {

//Send the Command Frame
    int anglediff = 0;
    unsigned int angle[2] = {0,0};
    unsigned long StartTime = micros();
    


    for(int i = 0; i<2; i++) {
        angle[i] = getAngle(p, CSN);
        delayMicroseconds(2);
    }
    unsigned long CurrentTime = micros();
    float ElapsedTime = (CurrentTime - StartTime - 2) *1e-6;

    anglediff = angle[1] - angle[0];
        

    if (anglediff > 8192) anglediff -= 16384;
        

    if (anglediff < -8192) anglediff += 16384;

    float Revs = (float) anglediff / 16384.0;
    

     // in total between read times 
        
    return  (Revs/ElapsedTime) * 60 ; // Positions Difference / Over time taken 
    
    // unsure if this works tbh

}

static float rpmEma = 0.0f;
static bool  rpmInit = false;
const float ALPHA = 0.04f;   // 0.05..0.30 (smaller = smoother)

inline float ema(float x) {
    if (!rpmInit) { rpmEma = x; rpmInit = true; }
    else          { rpmEma += ALPHA * (x - rpmEma); }
    return rpmEma;
}

float AverageRPM(Prop* p, int CSN) {
    // cant trust that the above was completely reliable especially cus unsure how quick code will run
    float rpmaverage = 0;
    const int N = 4; // number of averages 

    for(int i = 0; i < N; i++) {
        rpmaverage += RPMSensor(p, CSN);
        delayMicroseconds(10);
    }

    rpmaverage /= N;
    if(rpmaverage < 0) rpmaverage *= -1.0;
    return (rpmaverage); 
}






float PWMtoRPM_readings(Prop* p, Servo mot, int CSN, int PWM, int PWMVal) { // Returns RPMreading for each prop and PWMValue
    // This converts the ESC to RPM using the readings of the specific Motor
    // Initialises at 3 points to get a rough linear approximation for the PWM to RPM
    // Take three Throttle values between 1000 and 2000 (assumed min and max PWM for this motor, typical for industry)
    
    // Using values 1200, 1500, 1800
    
    float Reading;

    mot.writeMicroseconds(PWMVal);
    delay(5000); // delay for a second for warm up
    Reading = AverageRPM(p, CSN); // (calling function for finding RPM);


    return (Reading);


}

float PWM_RPMgradient(Prop* p, float PWMVals[3], float RPMVals[3]){
    float grad = 0;
    int n = 0;
    for(int i = 0; i < 2; i++){
        if(RPMVals[i] < 50) i++;
        grad += ((RPMVals[i+1])-(RPMVals[i]))/((PWMVals[i+1])-(PWMVals[i]));
        n++;   
    }
    return grad/n;

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
    // So y = 0 at x < 1070ish, then using grad, yintercept C:
    float yintercept = 0 - (1060 * gradient);

    // Now we have the full linear equation y = mx + C

    PWMValue = (RPM - yintercept) / gradient;
    
    if(!(PWMValue > 1000 && PWMValue < 2000)){
        Serial.println("Error 101 - PWM Value not calculated correctly, please check code");
        return 0;
    }
    else {
        return PWMValue; // Value outputted should be between 1000 and 2000
    }
}

// _________________________ Actual Control CODE ____________________________

//this part is for Diagonal or Adjacent drone props

int diagonal = 0; // set as 1 if you want diagonal phase difference
int adjacent = 1; // set as 1 if you want adjacent phase difference


Prop props[2];
Servo motor[2];


void setup() {
    Serial.begin(9600);

    // INITIALISATION OF PROPS 
    
    // Set CSN, PWM pins, and rpms and phase to start at zero;
    float RPMReads[3] = {0,0,0}; // start all at 0 
    add_prop(&props[0], 18, 19, 0, 0, RPMReads, 0);
    add_prop(&props[1], 6, 5, 0, 0, RPMReads, 0);
    // A reminder that the structure is PROP(CSN,PWM,current phase, current rpm, RPM Readings, and Gradient of ESC to RPM )

    // SETUP Pins

    for(int i = 0; i < 2; i++) {
        //Set Pin Modes
        pinMode(props[i].CSN, OUTPUT);
        motor[i].attach(props[i].PWM, 1000, 2000);
        

        //Set Slave Select High to Start i.e disable chip
        digitalWrite(props[i].CSN, HIGH);
        motor[i].writeMicroseconds(1000);
        
    }

    delay(3000); // 3 second delay
 
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
        motor[i].writeMicroseconds(1000);
        float PWMvalues[3] = {1070, 1080, 1090}; // change this as see fit, more PWM values will be good but make sure to change gradient code
        Serial.print("Motor ");
        Serial.println(i);
        for(int j = 0; j < 3; j++){
            float PWMtoRPM = 0;
            PWMtoRPM = PWMtoRPM_readings(&props[i], motor[i], props[i].CSN, props[i].PWM, PWMvalues[j]);
            delay(1000);
            props[i].RPMreadings[j] = PWMtoRPM;
            
            if(PWMtoRPM == 0) {
                // Check if RPM Sensors areworking for these values
                Serial.println("Error 102 - The RPM recorded is zero for these PWM values, please check the RPM sensors or code is working");
            
            }
            else {
            
                Serial.print("The RPM for Prop");
                Serial.print(i + 1);
                Serial.print(" at PWM Value  "); // i hate how this works
                Serial.print(PWMvalues[j]);
                Serial.print(" is ");
                Serial.println(PWMtoRPM);
            }
        }
        props[i].grad = PWM_RPMgradient(&props[i], PWMvalues, props[i].RPMreadings);
        Serial.print("Gradient is thus ");
        Serial.println(props[i].grad);
        
        
    }


    Serial.println("ALL RPM to PWM Readings have now been measured, these will be used for conversions");
    Serial.println("Loop mode");

}

void loop() {
    
    int i = 0; // gonna use i a lot so may as well initialise now  
    int n = 0; // number of propellers needed to be corrected, if motor 1 is off SetRPM we can increase this one first. 
        // Alternatively we can Base off MOTOR 1s rpm instead.
    int j = 0;




    //_____________________________WHAT RPM AND PHASE DIFF DO YOU WANT________________________________
    // This will set the RPM for all motors, and the phase difference required between
    // __RPM__
    float SETRPM = 1200; // This will start by setting the ESC/PWM value required for the RPM in MOTOR 1
    float PWMVal[2] = {0,0}; // current PWM Values, the number in this series corresponds to the number of motors
    float PWMSave = 0;


    // make a new 2 int array with the current RPM and PWM
    float RPMArray[2] = {0 , 0};
    float PWMArray[2] = {1000, 1000}; // 1000 again is minimum

    float RPMinit = 0; // Initial RPM as controlling starts
    float RPMdiff = 0; // RPM difference from SETRPM and current RPM

    float goldRPM[2] = {0, 0}; // golden ratio RPMs
    float goldPWM[2] = {0, 0}; // golden ratio PWMs
    float goldDiff[2] = {0, 0};
    

    float grad = 0; // gradient


    float phasediff = 90; // What phase difference do we want between the 

    float curr_phasediff = 0; // phase difference currently

    float phasetol = 10; // What phase tolerence can you allow

    float PWMphase = 0;






    // Therefore required PWM value for each prop can be calculated and set
    for(int i = 0; i < 2;  i++) {
        PWMVal[i] = PWMfromRPM(&props[i], props[i].CSN, props[i].PWM, SETRPM, props[i].grad);
        motor[i].writeMicroseconds(PWMVal[i]);
        delay(5000); // delay 5 seconds, allow time for prop to get up to speed
    
        props[i].rpm = AverageRPM(&props[i], props[i].CSN);
        // Print off what RPM value it is
        Serial.print("Motor ");
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
            FROM NOW ON I WILL REFER TO THESE VALUES AS RPM Initial, 0.33 and 1

            (this new line will have a new gradient) -> this will then give us two more points
            which will depend on if the RPM is within the 0 - 0.33 or 0.33 - 1(current as 0 and 1 as M1 RPM)
            
            continue the same now with these values - constantly searching the golden ratio till either
            bracket falls into the +-30RPM (+-0.5Hz)
            
            */
            
            for(j = 0; j < 1; j++) {
                // only want this to happen once
                RPMArray[0] = 0;
                RPMArray[1] = props[i].rpm;
                PWMArray[0] = 1000;
                PWMArray[1] = PWMVal[i];                
            }
                
            while(!(props[i].rpm > (SETRPM + 30) && props[i].rpm < (SETRPM - 30))) {
                if(props[i].rpm > (SETRPM + 30)){
                    

                    /* this is where the RPM is too high compared to the bracket 
                    PWM must decrease
                    */
                    
                    RPMinit = props[i].rpm;

                    //RPMDifference
                    RPMdiff = ( RPMinit - SETRPM + 30);


                    // decrease
                    delay(40);
                    
                    grad = ((RPMArray[1])-(RPMArray[0]))/((PWMArray[1])-(PWMArray[0]));
                    Serial.print("Gradient is ");
                    Serial.println(grad);

                    //0.33
                    goldRPM[0] = RPMinit - 0.33 * RPMdiff;
                    goldPWM[0] = PWMfromRPM(&props[i], props[i].CSN, props[i].PWM, goldRPM[0], grad);


                    //0.67
                    goldRPM[1] = RPMinit - 0.67 * RPMdiff;
                    goldPWM[1] = PWMfromRPM(&props[i], props[i].CSN, props[i].PWM, goldRPM[1], grad);


                    /*
                    How? 
                    The next part of the code will work out which has a closer RPM to SETRPM of the following:
                    The initial RPM,
                    The 0.33 along the new line,
                    Or where the new line proposes that the SETRPM should be.

                    Worked out using the RPM Difference (RPMDiff andd goldDiff)


                    This closer value will be chosen, and set as the now current speed for the motor, for the process
                    to be repeated again.

                    However, IF TWO of the RPM Diff values give different signs, these will then be used as the new 
                    bracket with the lower value now replacing the 0 in the first part of RPMArray. The PWM value
                    MUST appear within this and will be stopped solving once one of the values is placed within
                    the +-30RPM bracket

                    1. RPM Values along new line
                    2. RPM Diff and Gold Diff
                    3. NEW RPMArray set.
                    4. Repeat.

                    WHAT CASES are there!!!!! think of them all, you _WILL_ run into an edge case.
                    
                    CASE 1 - 0.33 > SETRPM > 1  -- between
                    CASE 2 - RPMInit > SETRPM > 0.33 -- between
                    CASE 3 - SETRPM < 1 -- still lower
                    CASE 4 - RPMInit < SETRPM - ERROR 

                    */
                    delay(10);
                    
                    for(j = 0; j < 2; j++){
                        motor[i].writeMicroseconds(goldPWM[j]);
                        delay(500);
                        goldRPM[j] = AverageRPM(&props[j], props[j].CSN);
                        goldDiff[j] = SETRPM - goldRPM[j];
                        if(goldDiff[j] < 0) {
                            goldDiff[j] *= -1;
                        }
                        delay(40);
                    }

                    // Which Value is closer?
                    // go through each check, 
                    //is RPM Diff bigger than goldDiff[0]

                                       
                    //_____ Case 1 - RPM between gold values 0.33 and 1 _______
                    // 1.1 - 0.33 is closer
                    // this would infer that the value is now inbetween 1 and 0.33
                    // (most likely case)
                    if( (SETRPM < goldRPM[0] && SETRPM > goldRPM[1])) {
                        RPMArray[0] = goldRPM[0];
                        RPMArray[1] = goldRPM[1];
                        
                        if(goldDiff[0] < goldDiff[1]) {
                            /* This is the part of the case when the 0.33 case is closest, 
                            
                            Setting the 0.33 value as the new RPM as it is closest to the 
                            wanted RPM.
                            */
                            motor[i].writeMicroseconds(goldPWM[0]);
                            
                        }

                        else {
                            // 1 value is the closest.
                            motor[i].writeMicroseconds(goldPWM[1]);
                        }
                    }

                    //_____ Case 2 - RPM between gold values 0 and 0.33 _______
                    
                    else if( RPMinit > SETRPM && SETRPM < goldRPM[0]) {
                        RPMArray[0] = RPMinit;
                        RPMArray[1] = goldRPM[0];
                        if(RPMdiff < goldDiff[0]) {
                            // 0 / RPMinit is closest
                            motor[i].writeMicroseconds(PWMVal[i]);
                            // IF THIS HAPPENS WHAT DO WE DO? - THIS CASE MEANS A COSNTANT 
                            // REPEAT -- must solve for this, i.e. decrease gradient?
                        
                            Serial.println("Error 103 - Initial RPM is closest not correct for this CASE, CHECK CODE");
                            
                        }

                        else {
                            // 0.33 value is the closest.
                            motor[i].writeMicroseconds(goldPWM[0]);
                        }


                    }

                    //_____ Case 3 - RPM below highest val 1 still _______

                    else if( goldRPM[1] < SETRPM) {
                        // this isnt an error in this part just a less likely case, and means that it should just run again
                        // from this value
                        RPMArray[1] = goldRPM[1];
                        motor[i].writeMicroseconds(goldPWM[1]);
                    }
                    
                    //_____ Case 4 - RPM above RPMinit _______
                    else if ( RPMinit < SETRPM) {
                        // Code has broken 
                        Serial.println("Error 104 - Wrong Feedback Selected, Check Code");
                    }         


                  
                                  
                    
                
                }






                if(props[i].rpm < (SETRPM - 30)){   

                    /* this is where the RPM is too high compared to the bracket 
                    PWM must decrease
                    */
                    
                    RPMinit = props[i].rpm;

                    //RPMDifference
                    RPMdiff = ( RPMinit - SETRPM + 30);


                    // decrease
                    
                    grad = ((RPMArray[1])-(RPMArray[0]))/((PWMArray[1])-(PWMArray[0]));


                    //0.33
                    goldRPM[0] = RPMinit + 0.33 * RPMdiff;
                    goldPWM[0] = PWMfromRPM(&props[i], props[i].CSN, props[i].PWM, goldRPM[0], grad);


                    //0.67
                    goldRPM[1] = RPMinit + 0.67 * RPMdiff;
                    goldPWM[1] = PWMfromRPM(&props[i], props[i].CSN, props[i].PWM, goldRPM[1], grad);


                    /*
                    How? 
                    The next part of the code will work out which has a closer RPM to SETRPM of the following:
                    The initial RPM,
                    The 0.33 along the new line,
                    Or where the new line proposes that the SETRPM should be.

                    Worked out using the RPM Difference (RPMDiff andd goldDiff)


                    This closer value will be chosen, and set as the now current speed for the motor, for the process
                    to be repeated again.

                    However, IF TWO of the RPM Diff values give different signs, these will then be used as the new 
                    bracket with the lower value now replacing the 0 in the first part of RPMArray. The PWM value
                    MUST appear within this and will be stopped solving once one of the values is placed within
                    the +-30RPM bracket

                    1. RPM Values along new line
                    2. RPM Diff and Gold Diff
                    3. NEW RPMArray set.
                    4. Repeat.

                    WHAT CASES are there!!!!! think of them all, you _WILL_ run into an edge case.
                    
                    CASE 1 - 0.33 < SETRPM < 1  -- between
                    CASE 2 - RPMInit < SETRPM < 0.33 -- between
                    CASE 3 - SETRPM > 1 -- still lower
                    CASE 4 - RPMInit > SETRPM - ERROR 

                    */
                    
                    for(j = 0; j < 2; j++){
                        motor[i].writeMicroseconds(goldPWM[j]);
                        delay(500);
                        goldRPM[j] = AverageRPM(&props[j], props[j].CSN);
                        goldDiff[j] = SETRPM - goldRPM[j];
                        delay(40);
                    
                    }

                    // Which Value is closer?
                    // go through each check, 
                    //is RPM Diff bigger than goldDiff[0]

                                       
                    //_____ Case 1 - RPM between gold values 0.33 and 1 _______
                    // 1.1 - 0.33 is closer
                    // this would infer that the value is now inbetween 1 and 0.33
                    // (most likely case)
                    if( (SETRPM > goldRPM[0] && SETRPM < goldRPM[1])) {
                        RPMArray[0] = goldRPM[0];
                        RPMArray[1] = goldRPM[1];
                        
                        if(goldDiff[0] < goldDiff[1]) {
                            /* This is the part of the case when the 0.33 case is closest, 
                            
                            Setting the 0.33 value as the new RPM as it is closest to the 
                            wanted RPM.
                            */
                            motor[i].writeMicroseconds(goldPWM[0]);
                            
                        }

                        else {
                            // 1 value is the closest.
                            motor[i].writeMicroseconds(goldPWM[1]);
                        }
                    }

                    //_____ Case 2 - RPM between gold values 0 and 0.33 _______
                    
                    else if( RPMinit < SETRPM && SETRPM > goldRPM[0]) {
                        RPMArray[0] = RPMinit;
                        RPMArray[1] = goldRPM[0];
                        if(RPMdiff < goldDiff[0]) {
                            // 0 / RPMinit is closest
                            motor[i].writeMicroseconds(PWMVal[i]);
                            // IF THIS HAPPENS WHAT DO WE DO? - THIS CASE MEANS A COSNTANT 
                            // REPEAT -- must solve for this, i.e. decrease gradient?
                        
                            Serial.println("Error 103 - Initial RPM is closest not correct for this C                                                                                                  ASE, CHECK CODE");
                            
                        }

                        else {
                            // 0.33 value is the closest.
                            motor[i].writeMicroseconds(goldPWM[0]);
                        }


                    }

                    //_____ Case 3 - RPM below highest val 1 still _______

                    else if( goldRPM[1] > SETRPM) {
                        // this isnt an error in this part just a less likely case, and means that it should just run again
                        // from this value
                        RPMArray[1] = goldRPM[1];
                        motor[i].writeMicroseconds(goldPWM[1]);
                    }
                    
                    //_____ Case 4 - RPM above RPMinit _______
                    else if ( RPMinit > SETRPM) {
                        // Code has broken 
                        Serial.println("Error 104 - Wrong Feedback Selected, Check Code");
                    }        
                }
            }
            Serial.print("RPMs should now be matching, RPM of PROP ");
            Serial.print(i);
            Serial.print(" is : ");
            Serial.println(AverageRPM(&props[i], props[i].CSN));
            



            //_______________________________PHASE CONTROL_____________________________





            if(i > 0) { // dont want this running for motor 1
                if(adjacent == 1){
                    PWMSave = PWMVal[i];
                    Serial.println("Adjacent Propeller Phase Control will now begin.");

                    props[i-1].phase = getAngle(&props[i-1], props[i-1].CSN);
                    props[i].phase = getAngle(&props[i], props[i].CSN);
                    curr_phasediff = props[i].phase - props[i-1].phase;
                    Serial.print("Current Phase Difference "); // Just a quick check
                    Serial.print(curr_phasediff);
                    Serial.print("  with a difference from the wanted as ");

                    float dpd = curr_phasediff - phasediff;


                    Serial.print(dpd);

                    float pd_tolbelow = phasediff - phasetol;
                    float pd_tolabove = phasediff - phasetol;

                    if((curr_phasediff > pd_tolbelow) && (curr_phasediff < pd_tolabove)) {
                        Serial.print("Phase difference is already met at ");
                        Serial.println(curr_phasediff);
                    }

                    //reset pwm and rpm array for grad
                    RPMArray[0] = 0;
                    
                    PWMArray[1] = PWMVal[i];
                    PWMArray[0] = 0;

                    float degrees_per_sec = 50 * 360 / 60;
                    
                    if(dpd < 0 ) dpd += 360;
                    if(dpd > 360) dpd -= 360;

                    float delaytime = ((dpd) / degrees_per_sec) * 1e6;

                    

                    if( delaytime < 0) {
                        Serial.println("Error delay time for phase change is negative.");
                    }
                    

                    grad = ((RPMArray[1])-(RPMArray[0]))/((PWMArray[1])-(PWMArray[0]));


                    while(!((curr_phasediff > pd_tolbelow) && (curr_phasediff < pd_tolabove))){
                        // How can i decrease/ increase phase gap.
                        /*
                        No matter if phasediff is bigger or smaller, may be better to decrease RPM until necessary as
                        lower RPMs have more stable PWM.
                        decrease by 100 rpm equivalent of PWM (600 degree diff per sec)
                        */
                        
                        RPMinit = AverageRPM(&props[i], props[i].CSN);
                        float RPMdiff = AverageRPM(&props[i], props[i].CSN) - 50;
                        
                        PWMphase = PWMfromRPM(&props[i], props[i].CSN, props[i].CSN, RPMdiff, grad);
                        if((PWMphase - PWMSave) != 0) {
                            motor[i].writeMicroseconds(PWMphase); // RPM for changing phase (50 less)
                            delayMicroseconds(delaytime);  // delay time needed to bring to phase
                            motor[i].writeMicroseconds(PWMSave);   // back to normal RPM
                            delay(40);
                        }
                        else{
                            Serial.println("Phase is now in sync.");
                            
                        }
                    }
                }
            }
        }
    }
}
    
        /*
        // reset for loop for phase for diagonal as this will have to wait for all to be done, 2 relies on 4, 3 relies on 1
        if(diagonal == 1) {
            fo        */