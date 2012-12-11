#include <SoftwareSerial.h>

#define DOF_DATA_INTERVAL 35 // Interval (milliseconds) between data sending, between 1 and 255 (inclusive)
#define DOF_DATA_CONTINUOUS true // Set to true to enable a continuous data stream on startup, false otherwise
#define DOF_SERIAL_DEBUG false // Set to true to echo the 9DoF stream to Serial and do no processesing on data

SoftwareSerial dofSerial(2, 3); // RX, TX
#include "DofHandler.h"
DofHandler<SoftwareSerial> dofHandler(&dofSerial);


void setup() {
  Serial.begin(38400);
  
  // Initialize at 9600 baud, but change to 28800
  dofHandler.begin(9600, 28800);
  dofHandler.setContinuousStream(DOF_DATA_CONTINUOUS);
  dofHandler.setUpdateInterval(DOF_DATA_INTERVAL);
  
}

void loop() {
#if DOF_SERIAL_DEBUG == true
  dofHandler.debugRead(Serial);
#else
  if (dofHandler.checkStream()) {
    if (dofHandler.isPacketGood())
      dofHandler.printData(Serial);
    else {
      Serial.println("Bad");
    }
  }
#endif
}
