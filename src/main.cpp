#include <Arduino.h>
#include <TMCStepper.h>
#include <RemoteDebug.h>
#include <WebOTA.h>
#include "SatTrackerDefines.h"
#include "credentials.h"

RemoteDebug debugger;

TMC2208Stepper AltMotor(&Serial1, R_SENSE);
TMC2208Stepper AzMotor(&Serial2, R_SENSE);

//This just makes the code easier to read
HardwareSerial *rotctld = &Serial;

void setup() {

  //Fire up the 3 serial ports
  Serial.begin(SERIAL_BAUD_RATE); 
  Serial1.begin(SERIAL_BAUD_RATE);  
  Serial2.begin(SERIAL_BAUD_RATE);

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