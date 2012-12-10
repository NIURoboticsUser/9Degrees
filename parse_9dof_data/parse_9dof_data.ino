#include <cctype>

#define DOF_BUFFER_SIZE 30 // Packet's data size is 30 bytes
#define ATOF_SUBSTRING_BUFFER_SIZE 64
#define DOF_SERIAL_STREAM dofSerial

byte dof_data_buffer[DOF_BUFFER_SIZE + 1] = {0};
byte dof_data_buffer_size = 0;
byte dof_packet_state = 0;
boolean dof_data_bad_line = false;
char atof_substring_buffer[ATOF_SUBSTRING_BUFFER_SIZE + 1] = {0};
short goodCount = 0;

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
  
  // Setup 9DoF data stream
  DOF_SERIAL_STREAM.begin(9600);
  delay(100);
  DOF_SERIAL_STREAM.print("#b6"); // Configure new baud rate of 38400
  DOF_SERIAL_STREAM.flush();
  DOF_SERIAL_STREAM.end();
  delay(100);
  DOF_SERIAL_STREAM.begin(28800);
  delay(10);
  DOF_SERIAL_STREAM.println("#o1"); // Enable continuous stream
  DOF_SERIAL_STREAM.print("#i"); // Set interval
  DOF_SERIAL_STREAM.write(30);
}

void loop() {
  // Uncomment below line to do a debug read of the 9DOF
 // Serial.println(DOF_SERIAL_STREAM.available());
 // DOF_SERIAL_STREAM.read();return;
  debugRead();return;
  //analogWrite(6, map(DOF_SERIAL_STREAM.available(), 0, 64, 0, 128));
  //analogWrite(6, map(dof_packet_state, 0, 4, 0, 255));
  /*
  if (DOF_SERIAL_STREAM.available() > 60) {
    digitalWrite(6, HIGH);
  } else {
    digitalWrite(6, LOW);
  }*/
  
  readDof();
  
  
  /*if (DOF_SERIAL_STREAM.available() > 60) {
    Serial.print("Available: ");
    Serial.println(DOF_SERIAL_STREAM.available());
  }
  if (DOF_SERIAL_STREAM.available()) {
    byte in = (byte)DOF_SERIAL_STREAM.read();
    if (in == '\n') {
      // There was a new line
      // Only parse the line if we don't know if it's bad already
      if (!dof_data_bad_line) {
        unsigned long timer = millis();
        boolean goodLine = parseDofLine();
        timer = millis() - timer;
        if (timer > 20) {
          Serial.print("Long Run! ");
          Serial.println(timer);
        }
        if (goodLine) {
          Serial.println("Good");
        } else {
          Serial.println("Bad");
        }
      } else {
        Serial.println("Overflow");
        clearDofBuffer();
      }
    } else {
      if (dof_data_buffer_size >= DOF_BUFFER_SIZE) {
        // Overflow of buffer
        dof_data_bad_line = true;
      } else {
        if (in > 126) {
          Serial.println("Extended");
        } else if (in < 32) {
          Serial.println("Control");
        } else if (in == 0) {
          Serial.println("Null");
        }
        dof_data_buffer[dof_data_buffer_size] = in;
        dof_data_buffer[++dof_data_buffer_size] = 0;
      }
    }
  }*/
}


void readDof() {
  if (DOF_SERIAL_STREAM.available()) {
    byte in = (byte)DOF_SERIAL_STREAM.read();
    switch (dof_packet_state) {
      case 0:
        if (in == '9') {
          dof_packet_state = 1;
          timeStart();
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
        delay(50);
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
            Serial.print("T");
            Serial.println(timeEnd());
            //Serial.println("Good");
            //Serial.println(DOF_SERIAL_STREAM.available());
            //Serial.print("Avg ");
            //Serial.println(dofDataTime / goodCount);
            //delay(5);
            //timeStart();
            printDofData();
            //Serial.println(timeEnd());
          } else {
            Serial.println("Bad");
            //Serial.print("Line error (overflow)! Char: ");
            //Serial.write(in);
           // Serial.print(" Available: ");
            //Serial.print(DOF_SERIAL_STREAM.available());
            //Serial.println();
          }
          clearDofBuffer();
        } else {
          dof_data_buffer[dof_data_buffer_size] = in;
          dof_data_buffer[++dof_data_buffer_size] = 0;
        }
        break;
    }
    /*if (in == '\n') {
      if (!dof_data_bad_line) {
        unsigned long timer = millis();
        boolean goodLine = readDofPacket();
        timer = millis() - timer;
        if (timer > 20) {
          Serial.print("Long Run! ");
          Serial.println(timer);
        }
        if (goodLine) {
          Serial.println("Good");
        } else {
          Serial.println("Bad");
        }
      } else {
        Serial.println("Overflow");
        clearDofBuffer();
      }
    } else {
      if (dof_data_buffer_size >= DOF_BUFFER_SIZE) {
        // Overflow of buffer
        dof_data_bad_line = true;
      } else {
        switch (dof_packet_state) {
          case 0:
            if (in == '9')
              dof_packet_state = 1;
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
            dof_data_buffer[dof_data_buffer_size] = in;
            dof_data_buffer[++dof_data_buffer_size] = 0;
            break;
        }
      }
    }*/
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
  read_short(38, data.gyroZ);
  
  dofData = data;
  //Serial.print("Time: ");
  //Serial.println(millis() - dofDataTime);
  
  goodCount++;
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
  //short val = 0;
  out = 0;
  out |= dof_data_buffer[startIndex]; out <<= 8;
  out |= dof_data_buffer[startIndex + 1];
}

void debugRead() {
  if (DOF_SERIAL_STREAM.available()) {
    byte in = (byte)DOF_SERIAL_STREAM.read();
    Serial.write(in);
  }
  return;
}

// Parses the line stored in dof_data_buffer
// If parsing was successful, stores data into 
// Returns true if the parsing was successful, false otherwise
boolean parseDofLine() {
  // Example line:
  //#A-14,7.17,248.83,M4.67,18.17,13100,G26.00,123.00,-7.00.\n
  return false;
  // Macro time!
/*  
  // bad_line() clears the line buffer and returns out of parseDofLine
#define bad_line() clearDofBuffer();return false;
  
  // checks to see if the passed character is the next character in the line
  // if so, allows funtion to continue. If not, calls bad_line()
#define check_char(CHAR) if (dof_data_buffer[byteCursor++] != CHAR) { bad_line(); }
  
  // Reads the next section (set of characters to the comma delimiter)
  // and puts the value into the passed variable,
  // exiting the function if the section is invalid
#define read_section(TARGET) if (!parseSection(byteCursor, length, TARGET)) { bad_line(); };

  byte length = dof_data_buffer_size;
  Serial.print("Buffer data: ");
  //Serial.print(dof_data_buffer);
  Serial.println();
  
  // Check line length
  // Minimum possible line length is 13
  if (length <= 13) {
    Serial.println("byteCursorLength bad");
    bad_line();
  }
  
  // Verify beginning byte
  if (dof_data_buffer[0] != '#') {
    bad_line();
  }
  
  byte byteCursor = 1;
  
  DofData data;
  
  // Accelerometer
  
  check_char('A');
  read_section(data.accelX);
  check_char(',');
  read_section(data.accelY);
  check_char(',');
  read_section(data.accelZ);
  
  // Magnometer
  
  check_char(',');
  check_char('M');
  read_section(data.magX);
  check_char(',');
  read_section(data.magY);
  check_char(',');
  read_section(data.magZ);
  
  // Gyroscope
  
  check_char(',');
  check_char('G');
  read_section(data.gyroX);
  check_char(',');
  read_section(data.gyroY);
  check_char(',');
  read_section(data.gyroZ);
  
  // If we get this far, the line has been well formed, up to this point.
  // Now to check to see if there is more we haven't seen yet.
  
  // The byteCursor *should* be greater than the length of the line
  if (byteCursor >= length) {
    // Everything is looking good
    dofData = data;
    dofDataTime = millis();
    //storeDofData(data);
    clearDofBuffer();
    return true;
  }
  Serial.println("byteCursorLength bad");
  Serial.print("Byte cursor: ");
  Serial.print(byteCursor);
  Serial.print(" Length: ");
  Serial.println(length);
  
  bad_line();

#undef bad_line
#undef check_char
#undef read_section
*/
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

double atof_substring(byte str[], byte start_index, byte end_index) {
  byte j = 0;
  for (byte i = start_index; i <= end_index; i++) {
    atof_substring_buffer[j++] = str[i];
  }
  atof_substring_buffer[j] = 0;
  return atof(atof_substring_buffer);
}

boolean parseSection(byte &byteCursor, byte length, double &value) {
  // endByte is inclusive; that is, the substring's last index is endByte,
  // including endByte
  byte endByte = byteCursor;
  /* Hit mask values (flags)
    0x10 - '-'
    0x20 - '.'
    0x40 - ','
  */
  byte hitMask = 0;
  for (short i = byteCursor; i < length; i++) {
    byte b = dof_data_buffer[i];
    switch (b) {
      case '-':
        if (hitMask & 0x10) {
          // Uh, oh. we found a '-' more than we should have...
          return false;
        } else {
          hitMask |= 0x10;
        }
        break;
      case '.':
        if (hitMask & 0x20) {
          // Uh, oh. we found a '.' more than we should have...
          return false;
        } else {
          hitMask |= 0x20;
        }
        break;
      case ',':
        // Got to the end of the section
        endByte = i - 1;
        value = atof_substring(dof_data_buffer, byteCursor, endByte);
        byteCursor = endByte + 1;
        return true;
      default:
        if (!isdigit(b)) {
          // Not expecting anything else except a digit
          return false;
        }
        endByte = i;
        break;
    }
  }
  
  // We reached the end of the string, so the section is done.
  value = atof_substring(dof_data_buffer, byteCursor, endByte);
  byteCursor = endByte + 1;
  return true;
}
