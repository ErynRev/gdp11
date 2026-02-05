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


typedef struct{
    // whats needed? CSN pins, PWN pins, phase angle, rpm
    int CSN;  
    int PWM;
    float angle;
    signed int rpm;
}Prop;

Prop add_prop(Prop* p, int CSN, int PWM, float angle, signed int rpm) {
    p->CSN = CSN;
    p->PWM = PWM;
    p->angle = angle;
    p->rpm = rpm;
}



void setup() {
    Serial.begin(9600);

    // INITIALISATION OF PROPS 
    Prop props[2];
    // Set CSN, PWM pins, and rpms and angles to start at zero;
    add_prop(&props[0], 10, 11, 0, 0);
    add_prop(&props[0], 15, 16, 0, 0);
    

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

}










int main (void) {
    
    while(){//motor is running -- PWM[0] is above a certain rpm

        for(int i = 1; i<2; i++) {
        
            while(rpm[i] < (rpm[0] - 30) && rpm[i] > (rpm[0] + 30)) {
                // slow down the motor using esc code
                // Change PWM SENSOR SIGNAL - Servo.write()
                // can we look at arduino code? - this is gonna be on there
                // NEEDS to be < 1Hz out (60rpm)
                if(rpm[i] < (rpm[0] - 30)) {
                    PWM[i] += 10; 
                    /*whatever this uses change by incremental steps, preferably large enough
                    that it doesnt take forever, BUT not too big that it can not fine adjust within.
                    
                    
                    */
                    
                }
                
                if(rpm[i] > (rpm[0] + 30)) {
                    PWM[i] -= 10; 
                }

            }



        }
    }



}
