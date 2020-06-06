//Params for communicating with rotctld
#define MAXLENGTH 80           // longest message we'll get from rotctld.  80 is a guess.  In practice they seem to be much shorter.
#define HOST_BAUD   19200      //Serial speed for communicating with rotctld

//Motor global attributes.  The steppers need to match and have the same driver.
#define MOTOR_BAUD  57600      //Serial speed for communicating with the TMC2208 drivers
#define R_SENSE     0.109      //Measured resistance of the sense resistors on the driver board
#define HOLD_MULTIPLIER 0.03125// Scales down the holding current from run current
#define MAX_SPEED   800        //Maximum steps/sec for the motor
#define DEG_PER_STEP 1.8       //degrees per full step of the stepper motors
#define MICROSTEP 4            // 1/(this value) = the step size for the motors
#define MOTOR_CURRENT 200      //RMS current limit (mA) for the motors
 
//Azimuth motor specifics
#define AZMOTOR_RX   25        //Serial RX pin for the AZ motor TMC2208
#define AZMOTOR_TX   26        //Serial TX pin for the AZ motor TMC2208
#define AZMOTOR_STEP 12        //IO pin for STEP
#define AZMOTOR_DIR 14         //IO pin for DIR
#define AZMOTOR_ENABLE 27      //IO pin for ENABLE
#define AZGEAR 10.0            //Gear ration on the Azimuth gear
#define MAX_AZ 360.0           //Maximum allowable value for azimuth
#define MIN_AZ 0.0             //minimum allowable azimuth

//Elevation motor specifics
#define ELMOTOR_RX  16         //Serial RX pin for EL motor TMC2208
#define ELMOTOR_TX  17         //Serial TX pin for EL motor TMC2208
#define ELMOTOR_STEP 0         //IO pin for STEP
#define ELMOTOR_DIR 2          //IO pin for DIR
#define ELMOTOR_ENABLE 4       //IO pin for ENABLE
#define ELGEAR 20.0            //Gear ratio on the elevation axis
#define MAX_EL 90.0            //maximum valid elevation input
#define MIN_EL 0.0             //minimum valid elevation input

//This structure is needed to pass the timing settings for StealthChop
//to the TMC2208 driver chips, but it's a constant so it's in here
typedef struct {
  uint8_t toff;
  int8_t hend;
  uint8_t hstrt;
} chopper_timing_t;
static constexpr chopper_timing_t chopper_timing = { 3, -1, 1 };
