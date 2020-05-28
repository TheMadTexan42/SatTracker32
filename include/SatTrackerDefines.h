#define SERIAL_BAUD_RATE 19200
#define WIFI_TIMEOUT 10000
#define HOSTNAME "SatTracker32"
#define R_SENSE 0.11
#define MAXLENGTH 80       // longest message we'll get from rotctld.  80 is a guess.  In practice they seem to be much shorter.

#define AZMOTOR_DIR 0      //Labeled D3
#define AZMOTOR_STEP 4     //Labeled D2
#define AZMOTOR_ENABLE 12  //Labeled D6
#define AZGEAR 10.0        //Gear ration on the Azimuth gear
#define MAX_AZ 360.0       //Maximum allowable value for azimuth
#define MIN_AZ 0.0         //minimum allowable azimuth

#define ELMOTOR_DIR 5      //Labeled D1
#define ELMOTOR_STEP 16    //Labeled D0
#define ELMOTOR_ENABLE 14  //Labeled D5
#define ELGEAR 20.0        //Gear ratio on the elevation axis
#define MAX_EL 90.0        //maximum valid elevation input
#define MIN_EL 0.0         //minimum valid elevation input

#define DEG_PER_STEP 1.8   //degrees per full step of the stepper motors (both motors need to the same - only setup for one of these params)
#define STEP_DELAY   500  //Minimum wait time between HI and LO for one step. 
#define CW 1               //HI/LO on the DIR PIN to make the code easier to read
#define CCW 0              //HI/LO on the DIR PIN to make the code easier to read
#define UP 1               //HI/LO on the DIR PIN to make the code easier to read
#define DOWN 0             //HI/LO on the DIR PIN to make the code easier to read
#define ENABLE 0           //HI/LO on the ENABLE PIN to make the code easier to read
#define DISABLE 1          //HI/LO on the ENABLE PIN to make the code easier to read