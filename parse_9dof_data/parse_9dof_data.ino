#include <cctype>

// -----------------------------------------------------------------------------------------
// ---------------------Begin 9 Degrees of Freedom Configuration----------------------------
// -----------------------------------------------------------------------------------------
#define DOF_SERIAL_STREAM dofSerial // Serial object name to use for the 9DoF
#define DOF_SERIAL_HARDWARE_BASE false // Set to true if you are using "Serial" for 9DoF communication
#define DOF_DATA_INTERVAL 20 // Interval (milliseconds) between data sending, between 1 and 255 (inclusive)
#define DOF_DATA_CONTINUOUS true // Set to true to enable a continuous data stream on startup, false otherwise
#define DOF_SERIAL_DEBUG false // Set to true to echo the 9DoF stream to Serial and do no processesing on data

// Baud rate for the 9 Degress of freedom. Use values from the table below.
// Does not have an effect when using USB serial (Serial)
#define DOF_SERIAL_BAUD 6 
/*
DOF_SERIAL_BAUD values:
  1 -> 2400 baud
  3 -> 9300 baud (default)
  4 -> 14400 baud
  5 -> 19200 baud
  6 -> 28800 baud (recommended)
  7 -> 38400 baud
  8 -> 57600 baud
  9 -> 115200 baud
  
Baud rates above 28800 do not seem to work with sofware serial.
*/

// -----------------------------------------------------------------------------------------
// -----------------------End 9 Degrees of Freedom Configuration----------------------------
// -----------------------------------------------------------------------------------------


unsigned long timer;

void timeStart() {
  timer = millis();
}

unsigned long timeEnd() {
  timer = millis() - timer;
  return timer;
}

struct DofData {
  double accelX;
  double accelY;
  double accelZ;
  double magX;
  double magY;
  double magZ;
  short gyroX;
  short gyroY;
  short gyroZ;
  
};

#include <SoftwareSerial.h>

SoftwareSerial dofSerial(2, 3); // RX, TX

DofData dofData;
unsigned long dofDataTime;

void setup() {
  
  Serial.begin(38400);
  dofDataTime = millis();
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  
  setupDof();
}

void loop() {
  loopDof();
}

void debugRead(const Stream &in, const Stream &out) {
  if (DOF_SERIAL_STREAM.available()) {
    byte in = (byte)DOF_SERIAL_STREAM.read();
    Serial.write(in);
  }
  return;
}

// -----------------------------------------------------------------------------------------
// -----------------------Begin 9 Degrees of Freedom Code-----------------------------------
// -----------------------------------------------------------------------------------------

#define DOF_BUFFER_SIZE 30 // Packet's data size is 30 bytes

byte dof_data_buffer[DOF_BUFFER_SIZE + 1] = {0};
byte dof_data_buffer_size = 0;
byte dof_packet_state = 0;
boolean dof_data_bad_line = false;
short dof_good_count = 0;
short dof_bad_count = 0;

void setupDof() {

#if DOF_SERIAL_HARDWARE_BASE != true
  #if DOF_SERIAL_BAUD == 1
    #define DOF_SERIAL_BAUD_RATE 2400
  #elif DOF_SERIAL_BAUD == 2
    #define DOF_SERIAL_BAUD_RATE 4800
  #elif DOF_SERIAL_BAUD == 3
    #define DOF_SERIAL_BAUD_RATE 9600
  #elif DOF_SERIAL_BAUD == 4
    #define DOF_SERIAL_BAUD_RATE 14400
  #elif DOF_SERIAL_BAUD == 5
    #define DOF_SERIAL_BAUD_RATE 19200
  #elif DOF_SERIAL_BAUD == 6
    #define DOF_SERIAL_BAUD_RATE 28800
  #elif DOF_SERIAL_BAUD == 7
    #define DOF_SERIAL_BAUD_RATE 38400
  #elif DOF_SERIAL_BAUD == 8
    #define DOF_SERIAL_BAUD_RATE 57600
  #elif DOF_SERIAL_BAUD == 9
    #define DOF_SERIAL_BAUD_RATE 115200
  #else
    #define DOF_SERIAL_BAUD_RATE 9600
    #undef DOF_SERIAL_BAUD
    #define DOF_SERIAL_BAUD 3
  #endif
  
  DOF_SERIAL_STREAM.begin(9600);
  
  #if DOF_SERIAL_BAUD != 3 // Only configure baud if not default
    delay(100);
    DOF_SERIAL_STREAM.print("#b"); // Configure new baud rate of 38400
    DOF_SERIAL_STREAM.print(DOF_SERIAL_BAUD);
    DOF_SERIAL_STREAM.flush();
    DOF_SERIAL_STREAM.end();
    delay(100);
    DOF_SERIAL_STREAM.begin(DOF_SERIAL_BAUD_RATE);
    delay(10);
  #endif
  
#else
  #define DOF_SERIAL_BAUD_RATE 9600
#endif

  dofContinuousStream(DOF_DATA_CONTINUOUS);
  dofSetInterval(DOF_DATA_INTERVAL);
}

void loopDof() {
#if DOF_SERIAL_DEBUG == true
  debugRead();
  return;
#endif
  if (DOF_SERIAL_STREAM.available()) {
    byte in = (byte)DOF_SERIAL_STREAM.read();
    switch (dof_packet_state) {
      case 0:
        if (in == '9') {
          dof_packet_state = 1;
        }
        break;
      case 1:
        if (in == 'D')
          dof_packet_state = 2;
        else
          dof_packet_state = 0;
        break;
      case 2:
        if (in == 'o')
          dof_packet_state = 3;
        else
          dof_packet_state = 0;
          
        break;
      case 3:
        if (in == 'F')
          dof_packet_state = 4;
        else
          dof_packet_state = 0;
        break;
      default:
        if (dof_data_buffer_size >= DOF_BUFFER_SIZE) {
          if (in == '\n') {
            // Good line
            readDofPacket();
            Serial.print("Avg "); // The average time between new data
            Serial.println(dofDataTime / dof_good_count);
            Serial.print((double)dof_bad_count / dof_good_count);
            Serial.print(" (");
            Serial.print(dof_bad_count);
            Serial.println(")");
            printDofData();
          } else {
            // Bad line
            Serial.println("Bad");
            dof_bad_count++;
          }
          clearDofBuffer();
        } else {
          dof_data_buffer[dof_data_buffer_size] = in;
          dof_data_buffer[++dof_data_buffer_size] = 0;
        }
        break;
    }
  }
}

void readDofPacket() {
  // Format:
  // MMMMAAAABBBBCCCCIIIIJJJJKKKKXXYYZZN (35 bytes long; 30 bytes of data)
  // Where MMMM is the magic number "9DoF" (no null terminator),
  // AAAA, BBBB, and CCCC are the X, Y and Z values (respectively) of the accelerometer
  // IIII, JJJJ, and KKKK are the X, Y and Z values (respectively) of the magnetometer
  // XX, YY, and ZZ are the X, Y and Z values (respectively) of the gyroscope
  // N is a new line character (\n)
  // MMMM and N are stripped as data is coming in.
  
  // Packet size is already 36 bytes long
  DofData data;
  
  read_double(0, data.accelX);
  read_double(4, data.accelY);
  read_double(8, data.accelZ);
  
  read_double(12, data.magX);
  read_double(16, data.magY);
  read_double(20, data.magZ);
  
  read_short(24, data.gyroX);
  read_short(26, data.gyroY);
  read_short(28, data.gyroZ);
  
  dofData = data;
  
  // Statistics collecting to check both average age of data,
  // And for seeing how old current data is
  dof_good_count++;
  dofDataTime = millis();
  
}

void read_double(byte startIndex, double &out) {
  // Dirty casting and shifting magic, undoing what was done on the 9Dof
  long val = 0;
  val |= dof_data_buffer[startIndex]; val <<= 8;
  val |= dof_data_buffer[startIndex + 1]; val <<= 8;
  val |= dof_data_buffer[startIndex + 2]; val <<= 8;
  val |= dof_data_buffer[startIndex + 3];
  out = *((double *)&val);
}

void read_short(byte startIndex, short &out) {
  out = 0;
  out |= dof_data_buffer[startIndex]; out <<= 8;
  out |= dof_data_buffer[startIndex + 1];
}



void clearDofBuffer() {
  dof_data_buffer[0] = 0;
  dof_data_buffer_size = 0;
  dof_data_bad_line = false;
  dof_packet_state = 0;
}

void printDofData() {
  Serial.println("\n9DoF Data:");
  Serial.print("{ { ");
  Serial.print(dofData.accelX);
  Serial.print(", ");
  Serial.print(dofData.accelY);
  Serial.print(", ");
  Serial.print(dofData.accelZ);
  Serial.print(" }, { ");
  Serial.print(dofData.magX);
  Serial.print(", ");
  Serial.print(dofData.magY);
  Serial.print(", ");
  Serial.print(dofData.magZ);
  Serial.print(" }, { ");
  Serial.print(dofData.gyroX);
  Serial.print(", ");
  Serial.print(dofData.gyroY);
  Serial.print(", ");
  Serial.print(dofData.gyroZ);
  Serial.println(" } }");
}

void dofContinuousStream(boolean enable) {
  if (enable) {
    DOF_SERIAL_STREAM.println("#o1");
  } else {
    DOF_SERIAL_STREAM.println("#o0");
  }
}

void dofRequestData() {
  DOF_SERIAL_STREAM.println("#f"); // Request _f_rame
}

void dofSetInterval(byte interval) {
  DOF_SERIAL_STREAM.print("#i"); // Set _i_nterval
  DOF_SERIAL_STREAM.write(interval);
}

// -----------------------------------------------------------------------------------------
// -------------------------End 9 Degrees of Freedom Code-----------------------------------
// -----------------------------------------------------------------------------------------
