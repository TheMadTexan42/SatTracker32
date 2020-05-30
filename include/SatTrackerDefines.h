
#define MAXLENGTH 80           // longest message we'll get from rotctld.  80 is a guess.  In practice they seem to be much shorter.

#define HOST_BAUD   19200
#define MOTOR_BAUD  57600
#define R_SENSE     0.109      //Measured resistance of the sense resistors on the driver board
#define HOLD_MULTIPLIER 0.2    // Scales down the holding current from run current
#define MAX_SPEED   800
#define ACCELERATION 800

#define AZMOTOR_RX   25
#define AZMOTOR_TX   26
#define AZMOTOR_CURRENT 300
#define AZMOTOR_MICROSTEP 4  // 1/(this value) = the step size for this motor
#define AZMOTOR_STEP 12       //Labeled D2
#define AZMOTOR_DIR 14        //Labeled D3
#define AZMOTOR_ENABLE 27    //  FIX THIS
#define AZGEAR 10.0          //Gear ration on the Azimuth gear
#define MAX_AZ 360.0         //Maximum allowable value for azimuth
#define MIN_AZ 0.0           //minimum allowable azimuth

#define ELMOTOR_RX  16
#define ELMOTOR_TX  17
#define ELMOTOR_CURRENT 300
#define ELMOTOR_MICROSTEP 4 // 1/(this value) = the step size for this motor
#define ELMOTOR_STEP 0     //Labeled D0
#define ELMOTOR_DIR 2       //Labeled D1
#define ELMOTOR_ENABLE 4    //  FIX THIS
#define ELGEAR 20.0         //Gear ratio on the elevation axis
#define MAX_EL 90.0         //maximum valid elevation input
#define MIN_EL 0.0          //minimum valid elevation input

#define DEG_PER_STEP 1.8     //degrees per full step of the stepper motors (both motors need to be the same - only setup for one of these params)

typedef struct {
  uint8_t toff;
  int8_t hend;
  uint8_t hstrt;
} chopper_timing_t;
static constexpr chopper_timing_t chopper_timing = { 3, -1, 1 };
