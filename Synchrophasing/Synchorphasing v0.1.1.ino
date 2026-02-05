/*
This code will use the RPM and Phase Angle sensor on two propellers.
Synchrophasing will be used to control and match the RPM yet leave the phase angle
at a degree where the sound waves shall destructively interfere.

This means, the code will need to use control methods, will need to look at prior
knowledge for this.

*/

// Use RPM ANGLE SENS v1.0.2 INPUT HERE
// Just change one keep one good -- decide on which gives best data. set this as 0th element of lists CSN[0] and PWM[0]
// or change code if easier

PWM



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
