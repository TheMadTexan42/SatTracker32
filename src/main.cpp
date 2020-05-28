#include <Arduino.h>
#include <TMCStepper.h>
#include <RemoteDebug.h>
#include <WebOTA.h>
#include "SatTrackerDefines.h"
#include "credentials.h"

RemoteDebug debugger;

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
      debugger.begin(HOSTNAME);

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

  debugger.handle();
  webota.handle();
}