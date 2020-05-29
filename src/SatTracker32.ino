#include <Arduino.h>
#include <TMCStepper.h>
#include <RemoteDebug.h>
#include "SatTrackerDefines.h"
#include "credentials.h"

RemoteDebug Debug;

char rec_msg[MAXLENGTH];  // array that will hold command string we get from rotctld
char ret_msg[MAXLENGTH];  //holds the message we will send back to rotctld
int rec_msg_index;
long lLastMsgTime;        //used for debugging - remove in final


//Globals to hold our position data
float fCurrentAz, fCurrentEl;
float fRequestedAz, fRequestedEl;
bool bAzDir, bElDir, bDisableAzOnZero, bDisableElOnZero;
unsigned int iAzSteps, iAltSteps;

//This just makes the code easier to read.  They're only pointers, so not much RAM burned.
HardwareSerial *rotctl = &Serial;
HardwareSerial *ElMotorSerial = &Serial1;
HardwareSerial *AzMotorSerial = &Serial2;

TMC2208Stepper ElMotorDriver(ElMotorSerial, R_SENSE);
TMC2208Stepper AzMotorDriver(AzMotorSerial, R_SENSE);

void setup() {

  //Setup WiFi and connect
  WiFi.setHostname(HOSTNAME);

  unsigned long lBeginTime = millis();
  while (!WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    delay(5000);
    ESP.restart();
  }

  //initialize the remote debugger object
  Debug.begin(HOSTNAME);

  //Fire up the 3 serial ports
  rotctl->begin(HOST_SERIAL_BAUD_RATE); 
  ElMotorSerial->begin(MOTOR_SERIAL_BAUD_RATE);  
  AzMotorSerial->begin(MOTOR_SERIAL_BAUD_RATE);

  //initialize the two motor drivers
  init_driver(&ElMotorDriver);
  init_driver(&AzMotorDriver);

}

void loop() {
  while ( rotctl->available() )
  {

    if(rec_msg_index < MAXLENGTH)
    {

      //Buffer is still OK, so get the next character from the serial port
      rec_msg[rec_msg_index++] = rotctl->read();

      //Check the character we just got.  A CR or LF indicates a complete message.
      if (rec_msg[rec_msg_index-1] == 10 || rec_msg[rec_msg_index-1] == 13)
      {

        if (rec_msg_index > 1)  //make sure this message isn't just a blank line
        {
          debugD("Got a message of %s", rec_msg);
          debugV("Time since last message is %d", (int) (millis() - lLastMsgTime));
          lLastMsgTime = millis();

          //Call the message parser
          ParseMessage();

          //Send the return message from the parser to rotctld via Serial
          debugD("Returning: %s", ret_msg);
          rotctl->print(ret_msg);
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
      rotctl->flush();

      break;
    }

  }

  //Run the stepper motors and update the current position
  step_motors();

  //Process the background tasks for remote dedugging and OTA updates
  Debug.handle();
}

void init_driver(TMC2208Stepper *motor)
{
  motor->begin();
  motor->toff(5);
  motor->pdn_disable(true);
  motor->rms_current(300);
  motor->ihold(0);
  motor->pwm_autoscale(true);
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
            sprintf(ret_msg,"AZ%3.1f EL%3.1f", fCurrentAz, fCurrentEl);
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
              fRequestedAz =  temp;                   //Set the global requested azimuth to the new value
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
                fRequestedEl = temp;                  //Set the global requested altitude to the new value
              }
              else
              {
                debugE("Got a bad altitude of %f", temp);  //Throw an error if the new altitude was out of range
              }
            }
            debugI("Requested AZ now: %3.1f - Requested Alt now: %3.1f", fRequestedAz, fRequestedEl);
            
            //Update the steps and directions
            update_motors();    

          }
          //we'll respond to the request for new with the current position, same as above
          sprintf(ret_msg,"AZ%3.1f EL%3.1f", fCurrentAz, fCurrentEl);
        }
    }
    else
    {
      //Stop scenario. Echo the message and home the tracker
      sprintf(ret_msg, rec_msg);

      home();
      update_motors();
    }
}

void clear_buffers()
{
  //Reset the string for the next message
  rec_msg_index = 0;
  memset(rec_msg, 0 , MAXLENGTH);
  memset(ret_msg, 0 , MAXLENGTH);
}

void step_motors()
{
    if (iAzSteps > 0)
    {
        /*  REPLACE THIS WITH THE UART COMMANDS */
        //Enable the motor
        //digitalWrite(AZMOTOR_ENABLE, ENABLE);
        
        
        //Set the direction pin
        digitalWrite(AZMOTOR_DIR, bAzDir);

        //start 1 step
        digitalWrite(AZMOTOR_STEP, HIGH);

        //subtract the step we just did from the total
        iAzSteps--;

        //update the current position
        if(bAzDir == CW)
        {
            fCurrentAz+=(DEG_PER_STEP/AZGEAR);
        }
        else
        {
            fCurrentAz-=(DEG_PER_STEP/AZGEAR);
            //The current azimuth can't go below zero
            if (fCurrentAz <= 0.0)
            {
                fCurrentAz = 0.0;
            }
        }
    }
    else
    {
        //We're not stepping, but we might need to disable the motors since we're done moving
        if(bDisableAzOnZero)
        {
            /*  REPLACE THIS WITH THE UART COMMANDS */
            //digitalWrite(AZMOTOR_ENABLE, DISABLE);
            bDisableAzOnZero = false;
        }
    }
    

    if (iAltSteps > 0)
    {
        /*  REPLACE THIS WITH THE UART COMMANDS */
        //Enable the motor
        //digitalWrite(ELMOTOR_ENABLE, ENABLE);

        //Set the direction pin
        digitalWrite(ELMOTOR_DIR, bElDir);
        
        //start 1 step
        digitalWrite(ELMOTOR_STEP, HIGH);

        //subtract the step we just did from the total
        iAltSteps--;

        //update the current position
        if(bElDir == UP)
        {
            fCurrentEl+=(DEG_PER_STEP/ELGEAR);
        }
        else
        {
            fCurrentEl-=(DEG_PER_STEP/ELGEAR);
            //The current altitude can't go below zero
            if (fCurrentEl <= 0.0)
            {
                fCurrentEl = 0.0;
            }
        }
    }
    else
    {
        //We're not stepping, but we might need to disable the motors since we're done moving
        if(bDisableElOnZero)
        {
            /*  REPLACE THIS WITH THE UART COMMANDS */
            //digitalWrite(ELMOTOR_ENABLE, DISABLE);
            bDisableElOnZero = false;
        }
    }

    if(stepping)   //We need the flag because we call this routine from loop() and we don't
                   //want to wait the STEP_DELAY time for no reason
    {
        //Wait the step time, end the steps, and wait again so the next step isn't too soon.
        delayMicroseconds(STEP_DELAY);
        digitalWrite(AZMOTOR_STEP, LOW);
        digitalWrite(ELMOTOR_STEP, LOW);
        delayMicroseconds(STEP_DELAY);
    }
}

void update_motors()
{
  //Steps are an int, but all the rest of the calculation arguments are float.  Need the extra parens
  //and the explicit cast to (int) to keep the math in floating point for as long as possible.

  //There are two possibilities.  If the delta is less than 180 than the "direct" path is the shortest.
  //Otherwise the other direction is the shorter path

  if(abs(fRequestedAz - fCurrentAz)<= 180) //This is the "direct" path
  {
    if(fRequestedAz > fCurrentAz)
    {
      //Need to move the Az CW
      bAzDir = CW;
      iAzSteps = (int) ((fRequestedAz-fCurrentAz)/DEG_PER_STEP*AZGEAR);
    }
    if(fRequestedAz < fCurrentAz)
    {
      //Need to move the Az CCW
      bAzDir = CCW;
      iAzSteps = (int) ((fCurrentAz-fRequestedAz)/DEG_PER_STEP*AZGEAR);
    }
  }
  else    // This is the "shortcut" path
  {
    if(fRequestedAz > fCurrentAz)
    {
      //Need to move the Az CCW
      bAzDir = CCW;
      iAzSteps = abs((int) ((fRequestedAz-fCurrentAz)/DEG_PER_STEP*AZGEAR));
    }
    if(fRequestedAz < fCurrentAz)
    {
      //Need to move the Az CW
      bAzDir = CW;
      iAzSteps = abs((int) ((fCurrentAz-fRequestedAz)/DEG_PER_STEP*AZGEAR));
    }
  }
  

  //altitude is simpler. Only one way to calculate it
  if(fRequestedEl > fCurrentEl)
  {
    //Need to move the El UP
    bElDir = UP;
    iAltSteps = (int) ((fRequestedEl-fCurrentEl)/DEG_PER_STEP*ELGEAR);
  }
  if(fRequestedEl < fCurrentEl)
  {
    //Need to move the El Down
    bElDir = DOWN;
    iAltSteps = (int) ((fCurrentEl-fRequestedEl)/DEG_PER_STEP*ELGEAR);
  }
  
  debugI("AZ steps: %d - EL steps: %d", iAzSteps, iAltSteps);

}

void home()
{
    //Set the motors to home
    fRequestedAz=0.0;
    fRequestedEl=0.0;

    //Set the motors to disable once homed
    bDisableAzOnZero = true;
    bDisableElOnZero = true;
}
