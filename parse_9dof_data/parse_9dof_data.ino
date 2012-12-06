#include <cctype>

#define DOF_BUFFER_SIZE 64
#define ATOF_SUBSTRING_BUFFER_SIZE 64

char dof_data_buffer[DOF_BUFFER_SIZE + 1] = {0};
short dof_data_buffer_size = 0;
boolean dof_data_bad_line = false;
char atof_substring_buffer[ATOF_SUBSTRING_BUFFER_SIZE + 1] = {0};

struct DofData {
  double accelX;
  double accelY;
  double accelZ;
  double magX;
  double magY;
  double magZ;
  double gyroX;
  double gyroY;
  double gyroZ;
  
};

DofData dofData;
unsigned long dofDataTime;

void setup() {
  Serial.begin(9600);
  dofDataTime = millis();
}

void loop() {
  // Uncomment below line to do a debug read of the 9DOF
  //debugRead();return;
  
  
  while (Serial.available()) {
    byte in = (byte)Serial.read();
    if (in == '\n') {
      // There was a new line
      // Only parse the line if we don't know if it's bad already
      if (!dof_data_bad_line) {
        parseDofLine();
      } else {
        clearDofBuffer();
      }
    } else {
      if (dof_data_buffer_size >= DOF_BUFFER_SIZE) {
        // Overflow of buffer
        dof_data_bad_line = true;
      } else {
        dof_data_buffer[dof_data_buffer_size] = in;
        dof_data_buffer[++dof_data_buffer_size] = 0;
      }
    }
  }
}

void debugRead() {
  if (Serial.available()) {
    byte in = (byte)Serial.read();
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
  
  // Macro time!
  
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
  
  // Erase ending decimal
  if (dof_data_buffer[--length] == '.') {
    dof_data_buffer[length] = 0;
    length;
  } else {
    bad_line();
  }
  
  // Check line length
  // Minimum possible line length is 13
  if (length <= 13) {
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
  if (byteCursor > length) {
    // Everything is looking good
    dofData = data;
    dofDataTime = millis();
    //storeDofData(data);
    clearDofBuffer();
    return true;
  }
  
  bad_line();

#undef bad_line
#undef check_char
#undef read_section
}

void clearDofBuffer() {
  dof_data_buffer[0] = 0;
  dof_data_buffer_size = 0;
  dof_data_bad_line = false;
}

double atof_substring(char str[], byte start_index, byte end_index) {
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
