#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Arduino.h>
#include <TMCStepper.h>
#include <RemoteDebug.h>
#include <AccelStepper.h>
#include <HardwareSerial.h>
#include "SatTrackerDefines.h"
#include "credentials.h"

//Remote debuggin services since we are out of serial outputs.
RemoteDebug Debug;

//Buffers for the serial messages sent to/from rotctld
char rec_msg[MAXLENGTH];  // array that will hold command string we get from rotctld
char ret_msg[MAXLENGTH];  //holds the message we will send back to rotctld
int rec_msg_index;        //tracked to prevent buffer overruns

//Objects for access to the serial ports
HardwareSerial rotctl(0);
HardwareSerial AzMotor_serial(1);
HardwareSerial ElMotor_serial(2);

//Objects for the TMC 2208 driver chips
TMC2208Stepper AzMotor_driver(&AzMotor_serial, R_SENSE);
TMC2208Stepper ElMotor_driver(&ElMotor_serial, R_SENSE);

//Abstraction for the stepper motors
AccelStepper AzMotor(AccelStepper::DRIVER ,AZMOTOR_STEP, AZMOTOR_DIR);
AccelStepper ElMotor(AccelStepper::DRIVER ,ELMOTOR_STEP, ELMOTOR_DIR);

void setup() {

  //Setup WiFi and connect
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(SSID, PASSWORD);

  while (!(WiFi.waitForConnectResult() == WL_CONNECTED))
  {
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA.setHostname(HOSTNAME);

  ArduinoOTA.onStart([]() {
  });
  ArduinoOTA.onEnd([]() {
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  ArduinoOTA.onError([](ota_error_t error) {
  });

  ArduinoOTA.begin();

  //initialize the remote debugger object
  Debug.begin(HOSTNAME);

  //Fire up the serial ports
  rotctl.begin(HOST_BAUD); 
  AzMotor_serial.begin(MOTOR_BAUD, SERIAL_8N2, AZMOTOR_RX, AZMOTOR_TX);  
  ElMotor_serial.begin(MOTOR_BAUD, SERIAL_8N2, ELMOTOR_RX, ELMOTOR_TX);  

  //Initialize the TMC2208 driver chips
  tmc_init(AzMotor_driver, MOTOR_CURRENT, MICROSTEP);
  tmc_init(ElMotor_driver, MOTOR_CURRENT, MICROSTEP);

  //Initialize the AccelStepper classes
  accel_stepper_init(AzMotor, AZMOTOR_ENABLE);
  accel_stepper_init(ElMotor, ELMOTOR_ENABLE);

  //Home the mechanism so we know where we are
  home();
}

void loop() {
  //If there is data in the host serial buffer then go get it
  while ( rotctl.available() )
  {
    //Check to make sure that the buffer still has room for more characters
    if(rec_msg_index < MAXLENGTH)
    {
      //Buffer is still OK, so get the next character from the serial port
      rec_msg[rec_msg_index++] = rotctl.read();

      //Check the character we just got.  A CR or LF indicates a complete message.
      if (rec_msg[rec_msg_index-1] == 10 || rec_msg[rec_msg_index-1] == 13)
      {
        if (rec_msg_index > 1)  //make sure this message isn't just a blank line
        {
          debugD("Got a message of %s", rec_msg);

          //Call the message parser
          ParseMessage();

          //Send the return message from the parser to rotctld via Serial
          debugD("Returning: %s", ret_msg);
          rotctl.print(ret_msg);
        }

        //Prep the buffers for the next message
        clear_buffers();
      }
    }
    else
    {
      //Buffer overflow.  Log, clear and start over
      debugE("Buffer overflow!  Buffer was %s: ", rec_msg);
      clear_buffers();

      //Additionally, flush the rest of the contents of the hardware buffer
      rotctl.flush();

      //We at the end of the while loop, so technically the break isn't necessary, but here it is anyway:
      break;
    }
  }

//Tasks to do every loop cycle.  MAKE SURE NONE OF THESE THINGS ARE BLOCKING!

  //Run the stepper motors and update the current position
  AzMotor.run();
  ElMotor.run();

  //Process the background tasks for remote dedugging and OTA updates
  ArduinoOTA.handle();
  Debug.handle();

  //Let the ESP run off and do ESP stuff if it needs to (which it normally doesn't) because 
  //timing out the watchdog timer will hose everything up fast...
  yield();
}

void ParseMessage()
{
  char *token; //used in the strtok function
  float temp;
  
  //The first character will either be an "A" for an AX/EL query or command, or an "S" for a stop moving command
  if (rec_msg[0] == 'A')
  {
  //AZ/EL scenario
  //if character 3 is a space, then this is just AZ EL as a query.  Return the values
      if (rec_msg[2] == ' ')
      {
        sprintf(ret_msg,"AZ%3.1f EL%3.1f", get_azimuth(), get_elevation());
      }
      else
      {
        //This is an AZ EL command and we have new values.  Unfortunately these are not fixed with
        //so we have to go digging for the spaces
        token = strtok(rec_msg, " ");

        if (token != NULL)                          //We found a space character
        {
          token+=2;                                 //skip the AZ characters
          temp = atof(token);                       //Convert the substring to a float
          if (temp >= MIN_AZ && temp <= MAX_AZ)     //Check to see that the request is in range before accepting it
          {
            debugI("Requested AZ now:%f3.1", temp);
            set_azimuth_target(temp);
          }
          else
          {
            debugE("Got a bad azimuth of %f", temp);   //Throw an error if the new azimuth was out of range
          }

          //keep going for the altitude.  The NULL source means continue where you last left off
          token = strtok(NULL, " ");
          if (token != NULL)
          {
            token+=2;                               //Skip the EL characters
            temp = atof(token);                     //Convert the substring to a float
            if (temp >= MIN_EL && temp <= MAX_EL)   //Check to see if the new value is in range before accespting it
            {
              debugI("Requested EL now: %f3.1", temp);
              set_elevation_target(temp);
            }
            else
            {
              debugE("Got a bad altitude of %f", temp);  //Throw an error if the new altitude was out of range
            }
          }
        }
        //we'll respond to the request for new with the current position, same as above
        sprintf(ret_msg,"AZ%3.1f EL%3.1f", get_azimuth(), get_elevation());
      }
    }
    else
    {
      //Stop scenario. Echo the message and home the tracker
      sprintf(ret_msg, rec_msg);
      home();
    }
}

float get_elevation()
{
  //Number of steps taken * degrees per step / each step is a fraction / our gear ratio
  return(ElMotor.currentPosition() * DEG_PER_STEP / MICROSTEP / ELGEAR);
}

float get_azimuth()
{
  //Number of steps taken * degrees per step / each step is a fraction / our gear ratio
  return(AzMotor.currentPosition() * DEG_PER_STEP / MICROSTEP / AZGEAR);
}

void set_elevation_target(float new_elevation)
{
  int total_steps;
  //Total number of steps possible is based on the motor step rate, the microstep fraction
  //setting, and the gearing on this axis.  It's a waste to calculate this on every new
  //position request, but it's cheap and makes it clear what we're doing.
  total_steps = MAX_EL / DEG_PER_STEP * MICROSTEP * ELGEAR;

  //Now that we know to total number of steps, move to whatever percentage of the total
  //step count we need to be at the same percentage of the elevation axis
  ElMotor.moveTo(total_steps * (new_elevation/MAX_EL));
}

void set_azimuth_target(float new_azimuth)
{
  int total_steps;
  //Total number of steps possible is based on the motor step rate, the microstep fraction
  //setting, and the gearing on this axis.  It's a waste to calculate this on every new
  //position request, but it's cheap and makes it clear what we're doing.
  total_steps = MAX_AZ / DEG_PER_STEP * MICROSTEP * AZGEAR;

  //Now that we know to total number of steps, move to whatever percentage of the total
  //step count we need to be at the same percentage of the elevation axis
  AzMotor.moveTo(total_steps * (new_azimuth/MAX_AZ));
}

void clear_buffers()
{
  //Reset the string for the next message
  rec_msg_index = 0;
  memset(rec_msg, 0 , MAXLENGTH);
  memset(ret_msg, 0 , MAXLENGTH);
}

void home()
{
  set_azimuth_target(0);
  set_elevation_target(0);
}

void tmc_init(TMC2208Stepper &st, const uint16_t mA, const uint16_t microsteps) 
{
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
  chopconf.intpol = true;
  chopconf.hend = chopper_timing.hend + 3;
  chopconf.hstrt = chopper_timing.hstrt - 1;
  st.CHOPCONF(chopconf.sr);

  st.rms_current(mA, HOLD_MULTIPLIER);
  st.microsteps(microsteps);
  st.ihold(0);
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

void accel_stepper_init(AccelStepper &stepper, int enable_pin)
{
  stepper.setEnablePin(enable_pin);
  stepper.setPinsInverted(false, false, true);
  stepper.setMaxSpeed(MAX_SPEED*MICROSTEP);
  stepper.setSpeed(MAX_SPEED*MICROSTEP);
  stepper.setAcceleration(MAX_SPEED * 3);
  stepper.enableOutputs();
}