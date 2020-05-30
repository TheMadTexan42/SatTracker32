#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Arduino.h>
#include <TMCStepper.h>

#include <AccelStepper.h>
#include <RemoteDebug.h>
#include <HardwareSerial.h>

#define AZMOTOR_STEP      12 
#define AZMOTOR_DIR       14 
#define AZMOTOR_ENABLE    27
#define AZ_TX
#define ELMOTOR_STEP 0  
#define ELMOTOR_DIR 2   
#define ELMOTOR_ENABLE 4

#define HOLD_MULTIPLIER    0.2  // Scales down the holding current from run current
#define INTERPOLATE       true  // Interpolate X/Y/Z_MICROSTEPS to 256
#define MOTOR_BAUD        57600
#define HOSTNAME          "TMCTest"
#define R_SENSE           0.109f 

#define CHOPPER_TIMING   { 3, -1, 1 } 

HardwareSerial AzMotor_serial(1);
HardwareSerial ElMotor_serial(2);

TMC2208Stepper AzMotor_driver(&AzMotor_serial, R_SENSE);
TMC2208Stepper ElMotor_driver(&ElMotor_serial, R_SENSE);

AccelStepper AzMotor(AccelStepper::DRIVER ,AZMOTOR_STEP, AZMOTOR_DIR);
AccelStepper ElMotor(AccelStepper::DRIVER ,ELMOTOR_STEP, ELMOTOR_DIR);

RemoteDebug Debug;

typedef struct {
  uint8_t toff;
  int8_t hend;
  uint8_t hstrt;
} chopper_timing_t;

static constexpr chopper_timing_t chopper_timing = CHOPPER_TIMING;

struct {
  int rms_current;
  float holding_mult;
  int microsteps;
  float speed;
} motor_values;


void debug_project_callback()
{
  char command[20];
  char *a_char;
  int newval=0;
  float newvalf=0.0;

  Debug.getLastCommand().toCharArray(command, 20);

  a_char = &command[0];  
  switch (*a_char)
  {
  case 'S':  //speed
    newvalf = atof(a_char+1);
    debugD("New speed is: %f", newvalf);
    motor_values.speed = newvalf;
    break;

  case 'H':  //new holding current value
    newvalf = atof(a_char+1);
    debugD("New hold multipler is: %f", newvalf);
    motor_values.holding_mult = newvalf;
    break;

  case 'M':  // microsteps
    newval = atoi(a_char+1);
    debugD("New steps are: %d", newval);
    motor_values.microsteps = newval;
    break;

  case 'I':  //Current limit
    newvalf = atof(a_char+1);
    debugD("New current is: %d", newval);
    motor_values.rms_current = newvalf;
    break;

  default:
    debugD("Didn't get a match on the command, but will ping the driver");
    break;
  }
 
  AzMotor_driver.microsteps(motor_values.microsteps);
  AzMotor_driver.rms_current(motor_values.rms_current, motor_values.holding_mult);
  AzMotor_driver.GSTAT(0b111); // Clear

  ElMotor_driver.microsteps(motor_values.microsteps);
  ElMotor_driver.rms_current(motor_values.rms_current, motor_values.holding_mult);
  ElMotor_driver.GSTAT(0b111);
  
  //Give the system time to do the serial stuff
  delay(20);

  AzMotor.setSpeed(motor_values.speed);
  ElMotor.setSpeed(motor_values.speed);
}

void setup() 
{
#if(true)
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin("HOME", "6104469871");
  if(!(WiFi.waitForConnectResult() == WL_CONNECTED))
  {
    //The WiFi connection didn't materialize, so wait 5 sec and try again
    delay(5000);
    ESP.restart();
  }
  WiFi.setAutoReconnect(true);

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME);

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
  });

  ArduinoOTA.onEnd([]() {
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
  });

  ArduinoOTA.begin();

  Debug.begin(HOSTNAME);
  Debug.setCallBackProjectCmds(&debug_project_callback);
#endif

  AzMotor_serial.begin(MOTOR_BAUD, SERIAL_8N2, 25, 26);
  tmc_init(AzMotor_driver, 200, 0);
  AzMotor.setEnablePin(AZMOTOR_ENABLE);
  AzMotor.setPinsInverted(false, false, true);
  AzMotor.setMaxSpeed(1000.0);
  AzMotor.setAcceleration(500.0);
  AzMotor.setSpeed(0.0);
  AzMotor.enableOutputs();
  
  ElMotor_serial.begin(MOTOR_BAUD, SERIAL_8N2, 16, 17);
  tmc_init(ElMotor_driver, 200, 0);
  ElMotor.setEnablePin(ELMOTOR_ENABLE);
  ElMotor.setPinsInverted(false, false, true);
  ElMotor.setMaxSpeed(1000.0);
  ElMotor.setAcceleration(500.0);
  ElMotor.setSpeed(0.0);
  ElMotor.enableOutputs();
}

void loop() 
{
  AzMotor.runSpeed();
  ElMotor.runSpeed();

  debugHandle();
  ArduinoOTA.handle();
}  

void tmc_init(TMC2208Stepper &st, const uint16_t mA, const uint16_t microsteps) {
TMC2208_n::GCONF_t gconf{0};
gconf.pdn_disable = true; // Use UART
gconf.mstep_reg_select = true; // Select microsteps with UART
gconf.i_scale_analog = false;
gconf.en_spreadcycle = false;
st.GCONF(gconf.sr);
//st.stored.stealthChop_enabled = true;

TMC2208_n::CHOPCONF_t chopconf{0};
chopconf.tbl = 0b01; // blank_time = 24
chopconf.toff = chopper_timing.toff;
chopconf.intpol = INTERPOLATE;
chopconf.hend = chopper_timing.hend + 3;
chopconf.hstrt = chopper_timing.hstrt - 1;
st.CHOPCONF(chopconf.sr);

st.rms_current(mA, HOLD_MULTIPLIER);
st.microsteps(microsteps);
st.iholddelay(10);
st.TPOWERDOWN(128); // ~2s until driver lowers to hold current

TMC2208_n::PWMCONF_t pwmconf{0};
pwmconf.pwm_lim = 12;
pwmconf.pwm_reg = 8;
pwmconf.pwm_autograd = true;
pwmconf.pwm_autoscale = true;
pwmconf.pwm_freq = 0b01;
pwmconf.pwm_grad = 14;
pwmconf.pwm_ofs = 36;
st.PWMCONF(pwmconf.sr);

st.GSTAT(0b111); // Clear
delay(200);
}

