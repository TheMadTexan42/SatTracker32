#define HOST_SERIAL_BAUD_RATE 19200
#define MOTOR_SERIAL_BAUD_RATE 250000
#define R_SENSE 0.11         //Measured resistance of the sense resistors on the driver board
#define MAXLENGTH 80         // longest message we'll get from rotctld.  80 is a guess.  In practice they seem to be much shorter.

#define AZMOTOR_STEP 12       //Labeled D2
#define AZMOTOR_DIR 14        //Labeled D3
#define AZMOTOR_ENABLE 27    //  FIX THIS
#define AZMOTOR_MICROSTEP 0  // 1/(this value) = the step size for this motor
#define AZGEAR 10.0          //Gear ration on the Azimuth gear
#define MAX_AZ 360.0         //Maximum allowable value for azimuth
#define MIN_AZ 0.0           //minimum allowable azimuth

#define ELMOTOR_STEP 0     //Labeled D0
#define ELMOTOR_DIR 2       //Labeled D1
#define ELMOTOR_ENABLE 4    //  FIX THIS
#define ELMOTOR_MICROSTEP 0 // 1/(this value) = the step size for this motor
#define ELGEAR 20.0         //Gear ratio on the elevation axis
#define MAX_EL 90.0         //maximum valid elevation input
#define MIN_EL 0.0          //minimum valid elevation input

#define DEG_PER_STEP 1.8     //degrees per full step of the stepper motors (both motors need to be the same - only setup for one of these params)
#define STEP_DELAY   500    //Minimum wait time between HI and LO for one step. 
#define CW 1                 //HI/LO on the DIR PIN to make the code easier to read
#define CCW 0                //HI/LO on the DIR PIN to make the code easier to read
#define UP 1                 //HI/LO on the DIR PIN to make the code easier to read
#define DOWN 0               //HI/LO on the DIR PIN to make the code easier to read
#define ENABLE 0             //HI/LO on the ENABLE PIN to make the code easier to read
#define DISABLE 1            //HI/LO on the ENABLE PIN to make the code easier to read
