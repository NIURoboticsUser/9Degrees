// The next two lines are for using SoftwareSerial instead of HardwareSerial
#include <SoftwareSerial.h>
SoftwareSerial dofSerial(2, 3); // RX, TX

#include "DofData.h"
#include "DofHandler.h"
// If you are using SoftwareSerial, uncomment the next line
DofHandler<SoftwareSerial> dofHandler(&dofSerial);

// If you are using Hardware Serial (Serial, Serial1, etc.),
// uncomment the next line
//DofHandler<HardwareSerial> dofHandler(&Serial);



void setup() {
  // Uncomment if you are not using the main Serial line to
  // communicate with the 9DoF
  Serial.begin(38400);

  // Connect to the 9DoF at 9600 baud to tell it to
  // upgrade to 28800 baud.
  // (You should not call Serial.begin() in this case)
  dofHandler.begin(9600, 28800);
  
  // Set the data mode of the 9DoF. This tells the 9DoF
  // what kind of data it is sending.
  /* Current possible values are:
    - DOF_DATA_MODE_ALL = 0, Sends all of the sensor data (accel, magnetom, gyro).
    - DOF_DATA_MODE_GYRO = 1, Sends only the gyroscope data (X, Y, and Z).
    - DOF_DATA_MODE_EULER = 2, Sends the Euler angles (yaw, pitch, and roll).
    
    Defaults to DOF_DATA_MODE_ALL.
  */
  dofHandler.setDataMode(DOF_DATA_MODE_ALL);
  
  // This is the rate (in milliseconds) that the 9DoF will read
  // from its sensors. That is, this is how often requests from the 9Dof
  // will come back with new and different data; requests for data
  // more frequent than this will get stale data. It is best to set this
  // to be about the time it takes you process the data.
  dofHandler.setUpdateInterval(40);
  
  // Send out a request for data from the 9DoF.
  dofHandler.requestData();
}

void loop() {
  // Check the Serial buffer for new data, and if a bad
  // packet is received, ask for more data.
  dofHandler.checkStreamValid();
  
  // Check to see if there is new data available.
  // By passing in true, we clear the new data available flag,
  // so new calls to isNewDataAvailable return false
  // (until a packet is received).
  if (dofHandler.isNewDataAvailable(true)) {
    // We could also do a call to isPacketGood, which makes sure
    // that the new packet that was received is well formed, but
    // checkStreamValid does that for us, so we don't have to.
    
    // We will also request another frame of data, so it is ready
    // the next time we need it.
    dofHandler.requestData();
    
    // Now it is time to do stuff with this data.
    
    // We need to check to see what kind of data we are
    // receiving. (We already know because we set the data mode
    // in the setup() function, but this is for demonstration
    // purposes when the type of data you are requesting can change.
    // Note: the curly braces around the "case X:" are needed to
    // resolve sope issues with the variables declared in the switch.
    switch (dofHandler.getLastDataMode()) {
      case DOF_DATA_MODE_GYRO: {
        GyroData gyro = dofHandler.getGyroData();
        // Now, we can do stuff with our gyroscope data.
        } break;
      case DOF_DATA_MODE_ALL: {
        DofData sensorData = dofHandler.getData();
        // Now, we can do stuff with our sensor data.
        } break;
      case DOF_DATA_MODE_EULER: {
        EulerData eulerData = dofHandler.getEulerData();
        // Now, we can do stuff with our euler data.
        } break;
    }
    
    dofHandler.printData(Serial);
  }
}

void serialStructPrint(Stream &out, void* ptr, unsigned int objSize) {
  byte * b = (byte *) ptr;
  for(unsigned int i = 0; i<objSize; i++) {
    out.write(b[i]);
  }
}

