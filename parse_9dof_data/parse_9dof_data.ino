#include <SoftwareSerial.h>

#define DOF_DATA_INTERVAL 35 // Interval (milliseconds) between data sending, between 1 and 255 (inclusive)
#define DOF_DATA_CONTINUOUS false // Set to true to enable a continuous data stream on startup, false otherwise
#define DOF_SERIAL_DEBUG false // Set to true to echo the 9DoF stream to Serial and do no processesing on data

SoftwareSerial dofSerial(2, 3); // RX, TX
#include "DofHandler.h"
DofHandler<SoftwareSerial> dofHandler(&dofSerial);


void setup() {
  Serial.begin(38400);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // Initialize at 9600 baud, but change to 28800
  dofHandler.begin(9600, 28800);
  dofHandler.setContinuousStream(DOF_DATA_CONTINUOUS);
  dofHandler.setUpdateInterval(DOF_DATA_INTERVAL);
  dofHandler.setDataMode(DOF_DATA_MODE_EULER);

}

boolean sending = false;
unsigned long time = 0;

void loop() {
#if DOF_SERIAL_DEBUG == true
  dofHandler.debugRead(Serial);
#else
  /*if (dofHandler.checkStream()) {
    if (dofHandler.isPacketGood()) {
      dofHandler.printData(Serial);
    } else {
      Serial.println("Bad");
    }
  }*/
  if (!sending) {
    time = millis();
    dofHandler.requestData();
    sending = true;
  } 
  else {
    if (dofHandler.checkStream()) {
      sending = false;
      time = millis() - time;
      if (dofHandler.isPacketGood()) {
        dofHandler.printData(Serial);
        digitalWrite(13, LOW);
      } else {
        digitalWrite(13, HIGH);
      }
      delay(100);
    }
  }
  if (Serial.available()) {
    int c = Serial.read();
    if (c == 'A') {
      dofHandler.setDataMode(DOF_DATA_MODE_ALL);
      dofHandler.requestData();
      while (!dofHandler.checkStream()) {}
      if (dofHandler.isPacketGood()) {
        // Send all data
      }
    } else if (c == 'G') {
      dofHandler.setDataMode(DOF_DATA_MODE_GYRO);
      dofHandler.requestData();
      while (!dofHandler.checkStream()) {}
      if (dofHandler.isPacketGood()) {
        // TODO: Send gyro data
      }
    } else if (c == 'E') {
      dofHandler.setDataMode(DOF_DATA_MODE_EULER);
      dofHandler.requestData();
      while (!dofHandler.checkStream()) {}
      if (dofHandler.isPacketGood()) {
        // TODO: Send gyro data
        struct AMG_ANGLES {
          int yaw;
          int pitch;
          int roll;
          char checkSum;
        } gdata;
        EulerData data = dofHandler.getEulerData();
        gdata.yaw = data.yaw;
        gdata.pitch = data.pitch;
        gdata.roll = data.roll;
        
        serialStructPrint(Serial, &gdata, sizeof(gdata));
      }
    }
  }
#endif
}

void serialStructPrint(Stream &out, void* ptr, unsigned int objSize) {
  byte * b = (byte *) ptr;
  for(unsigned int i = 0; i<objSize; i++) {
    out.write(b[i]);
  }
}

