/* This file is part of the Razor AHRS Firmware */

// Outputs in binary, but in a packet format so packet starts and ends can be located
// Mid-stream
void output_sensors_binary_packet() {
  // Format:
  // MMMMAAAABBBBCCCCIIIIJJJJKKKKXXXXYYYYZZZZN (41 bytes long)
  // Where MMMM is the magic number "9DoF" (no null terminator),
  // AAAA, BBBB, and CCCC are the X, Y and Z values (respectively) of the accelerometer
  // IIII, JJJJ, and KKKK are the X, Y and Z values (respectively) of the magnetometer
  // XX, YY, and ZZ are the X, Y and Z values (respectively) of the gyroscope (as signed shorts)
  // N is a new line character (\n)

// Caution: Dirty casting magic below. The bitshift operator is not defined for floating point numbers,
// So, I dereference the double pointer that is casted to a long pointer.
#define write_double(DOUBLE) { long val = *(long *)&DOUBLE; Serial.write(val >> 24); Serial.write(val >> 16); Serial.write(val >> 8); Serial.write(val);}
#define write_short(SHORT) { short val = SHORT; Serial.write(val >> 8); Serial.write(val); }
  // Magic number
  Serial.write('9'); Serial.write('D');
  Serial.write('o'); Serial.write('F');
  double temp;
  switch (data_mode) {
    case DATA_MODE_ALL: // 30 Bytes
      // Accelerometer
      temp = (accel[0] - accel_offset[0]) / GRAVITY;
      write_double(temp);
      temp = (accel[1] - accel_offset[1]) / GRAVITY;
      write_double(temp);
      temp = accel[2] / GRAVITY;
      write_double(temp);
      
      // Magnetometer
      write_double(magnetom[0]);
      write_double(magnetom[1]);
      write_double(magnetom[2]);
      
      // Gyroscope
      write_short((short)(gyro[0] - gyro_offset[0]));
      write_short((short)(gyro[1] - gyro_offset[1]));
      write_short((short)(gyro[2] - gyro_offset[2]));
      break;
    case DATA_MODE_GYRO: // 6 Bytes
      write_short((short)(gyro[0] - gyro_offset[0]));
      write_short((short)(gyro[1] - gyro_offset[1]));
      write_short((short)(gyro[2] - gyro_offset[2]));
      break;
    case DATA_MODE_EULER: // 12 Bytes
      write_double(roll);
      write_double(pitch);
      write_double(yaw);
      break;
  }
  
  Serial.write('\n');
  
#undef write_double
}

void output_sensors_text()
{
  Serial.print("#A-"); Serial.print('=');
  Serial.print(accel[0]); Serial.print(",");
  Serial.print(accel[1]); Serial.print(",");
  Serial.print(accel[2]); Serial.println();

  Serial.print("#M-"); Serial.print('=');
  Serial.print(magnetom[0]); Serial.print(",");
  Serial.print(magnetom[1]); Serial.print(",");
  Serial.print(magnetom[2]); Serial.println();

  Serial.print("#G-"); Serial.print('=');
  Serial.print(gyro[0]); Serial.print(",");
  Serial.print(gyro[1]); Serial.print(",");
  Serial.print(gyro[2]); Serial.println();
}

void output_sensors_text_single() // Single line
{
  Serial.print("#A");
  Serial.print(accel[0]); Serial.print(",");
  Serial.print(accel[1]); Serial.print(",");
  Serial.print(accel[2]); Serial.print(",");

  Serial.print('M');
  Serial.print(magnetom[0]); Serial.print(",");
  Serial.print(magnetom[1]); Serial.print(",");
  Serial.print(magnetom[2]); Serial.print(",");

  Serial.print('G');
  Serial.print(gyro[0]); Serial.print(",");
  Serial.print(gyro[1]); Serial.print(",");
  Serial.print(gyro[2]); Serial.print('\n');
}

void output_calibration(int calibration_sensor)
{
  if (calibration_sensor == 0)  // Accelerometer
  {
    // Output MIN/MAX values
    Serial.print("accel x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
      if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
      Serial.print(accel_min[i]);
      Serial.print("/");
      Serial.print(accel_max[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
  else if (calibration_sensor == 1)  // Magnetometer
  {
    // Output MIN/MAX values
    Serial.print("magn x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
      if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
      Serial.print(magnetom_min[i]);
      Serial.print("/");
      Serial.print(magnetom_max[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
  else if (calibration_sensor == 2)  // Gyroscope
  {
    // Average gyro values
    for (int i = 0; i < 3; i++)
      gyro_average[i] += gyro[i];
    gyro_num_samples++;
      
    // Output current and averaged gyroscope values
    Serial.print("gyro x,y,z (current/average) = ");
    for (int i = 0; i < 3; i++) {
      Serial.print(gyro[i]);
      Serial.print("/");
      Serial.print(gyro_average[i] / (float) gyro_num_samples);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
}

void output_sensors()
{
  if (output_mode == OUTPUT__MODE_SENSORS_RAW)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('R');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_CALIB)
  {
    // Apply sensor calibration
    compensate_sensor_errors();
    
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('C');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_BOTH)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
    {
      output_sensors_binary();
      compensate_sensor_errors();
      output_sensors_binary();
    }
    else if (output_format == OUTPUT__FORMAT_TEXT)
    {
      output_sensors_text('R');
      compensate_sensor_errors();
      output_sensors_text('C');
    }
  }
}

void output_sensors_text(char raw_or_calibrated)
{
  Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(accel[0]); Serial.print(",");
  Serial.print(accel[1]); Serial.print(",");
  Serial.print(accel[2]); Serial.println();

  Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(magnetom[0]); Serial.print(",");
  Serial.print(magnetom[1]); Serial.print(",");
  Serial.print(magnetom[2]); Serial.println();

  Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(gyro[0]); Serial.print(",");
  Serial.print(gyro[1]); Serial.print(",");
  Serial.print(gyro[2]); Serial.println();
}

void output_sensors_binary()
{
  Serial.write((byte*) accel, 12);
  Serial.write((byte*) magnetom, 12);
  Serial.write((byte*) gyro, 12);
}

