#include <Arduino.h>
#include <TMCStepper.h>
#include <RemoteDebug.h>
#include <WebOTA.h>
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
unsigned int iAzSteps, iElSteps;

//This just makes the code easier to read
HardwareSerial *rotctl = &Serial;
HardwareSerial *AltMotorSerial = &Serial1;
HardwareSerial *AzMotorSerial = &Serial2;

TMC2208Stepper AltMotor(AltMotorSerial, R_SENSE);
TMC2208Stepper AzMotor(AzMotorSerial, R_SENSE);

void setup() {

  //Fire up the 3 serial ports
  rotctl->begin(SERIAL_BAUD_RATE); 
  AltMotorSerial->begin(SERIAL_BAUD_RATE);  
  AzMotorSerial->begin(SERIAL_BAUD_RATE);

  //Setup WiFi and connect
  WiFi.setHostname(HOSTNAME);

  unsigned long lBeginTime = millis();
  while (!WiFi.isConnected())
  {
    WiFi.begin(SSID, PASSWORD);
    delay(2000);
    if (millis() - lBeginTime > WIFI_TIMEOUT) break;
  }

  if(WiFi.isConnected())
  {
      //initialize the remote debugger object
      Debug.begin(HOSTNAME);

      //webota is instantiated in the WebOTA.h file, hence not declared above
      webota.init();            
  }
  else
  {
    //Couldn't connect to WiFi so....
  }


}

void loop() {
  // put your main code here, to run repeatedly:

  Debug.handle();
  webota.handle();
}


void ParseMessage()
{
    char *token; //used in the strtok function
    float temp;

    //The first character will either be an "A" for an AX/EL query or command, or an "S" for a stop moving command
    if (rec_msg[0] == 'A')
    {
    //AL/EL scenario
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
            if (token != NULL)
            {
                token+=2;  //skip the AZ characters
                temp = atof(token);
                //Check to see that the request is in range before accepting it
                if (temp >= MIN_AZ && temp <= MAX_AZ)
                {
                    fRequestedAz =  temp;
                }
                else
                {
                    debugE("Got a bad azimuth of %f", temp);
                }

                //keep going for the elevation.  The NULL source means continue where you last left off
                token = strtok(NULL, " ");
                if (token != NULL)
                {
                    token+=2;  //Skip the EL characters
                    temp = atof(token);
                    //Check to see if the new value is in range before accespting it
                    if (temp >= MIN_EL && temp <= MAX_EL)
                    {
                        fRequestedEl = temp;
                    }
                    else
                    {
                        debugE("Got a bad elevation of %f", temp);
                    }
                }
                debugI("Requested AZ now: %3.1f - Requested EL now: %3.1f", fRequestedAz, fRequestedEl);
                
                //Update the steps and directions
                update_motors();    
                debugI("AZ steps: %d - EL steps: %d", iAzSteps, iElSteps);

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
    int step_del = STEP_DELAY_MIN;

    if(iAzSteps > 0 && iElSteps >0)
    {
        //We're running both motors, run at 2X the min.
        step_del = step_del *2;
    }
    else
    {
        //only one motor (or neither) is running so run at 4X the delay
        step_del = step_del * 4;
    }
    

    if (iAzSteps > 0)
    {
        //Enable the motor
        digitalWrite(AZMOTOR_ENABLE, ENABLE);
        //Set the direction pin
        digitalWrite(AZMOTOR_DIR, bAzDir);
        //do 1 step
        digitalWrite(AZMOTOR_STEP, HIGH);
        delayMicroseconds(step_del);
        digitalWrite(AZMOTOR_STEP, LOW);
        delayMicroseconds(step_del);

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
        if(bDisableAzOnZero)
        {
            digitalWrite(AZMOTOR_ENABLE, DISABLE);
            bDisableAzOnZero = false;
        }
    }
    

    if (iElSteps > 0)
    {
        //Enable the motor
        digitalWrite(ELMOTOR_ENABLE, ENABLE);
        //Set the direction pin
        digitalWrite(ELMOTOR_DIR, bElDir);
        //do 1 step
        digitalWrite(ELMOTOR_STEP, HIGH);
        delayMicroseconds(step_del);
        digitalWrite(ELMOTOR_STEP, LOW);
        delayMicroseconds(step_del);

        //subtract the step we just did from the total
        iElSteps--;

        //update the current position
        if(bElDir == UP)
        {
            fCurrentEl+=(DEG_PER_STEP/ELGEAR);
        }
        else
        {
            fCurrentEl-=(DEG_PER_STEP/ELGEAR);
            //The current elevation can't go below zero
            if (fCurrentEl <= 0.0)
            {
                fCurrentEl = 0.0;
            }
        }
    }
    else
    {
        if(bDisableElOnZero)
        {
            digitalWrite(ELMOTOR_ENABLE, DISABLE);
            bDisableElOnZero = false;
        }
    }

}

void update_motors()
{
    //These need to be in parens and explicitly cast, otherwise the math occurs as integers

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
    if(fRequestedEl > fCurrentEl)
    {
        //Need to move the El UP
        bElDir = UP;
        iElSteps = (int) ((fRequestedEl-fCurrentEl)/DEG_PER_STEP*ELGEAR);
    }
    if(fRequestedEl < fCurrentEl)
    {
        //Need to move the El Down
        bElDir = DOWN;
        iElSteps = (int) ((fCurrentEl-fRequestedEl)/DEG_PER_STEP*ELGEAR);
    }
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
