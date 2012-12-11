
/*
// Output angles: yaw, pitch, roll
void output_angles()
{
  if (output_format == OUTPUT__FORMAT_BINARY)
  {
    float ypr[3];  
    ypr[0] = TO_DEG(yaw);
    ypr[1] = TO_DEG(pitch);
    ypr[2] = TO_DEG(roll);
    Serial.write((byte*) ypr, 12);  // No new-line
  }
  else if (output_format == OUTPUT__FORMAT_TEXT)
  {
    Serial.print("#YPR=");
    Serial.print(TO_DEG(yaw)); Serial.print(",");
    Serial.print(TO_DEG(pitch)); Serial.print(",");
    Serial.print(TO_DEG(roll)); Serial.println();
  }
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



void output_sensors_raw(char sep1, char sep2)
{
  Serial.print('#');
  //Print Acceleration Data
  Serial.print(accel[0]); Serial.print(sep1);
  Serial.print(accel[1]); Serial.print(sep1);
  Serial.print(accel[2]); Serial.print(sep2);

  //Print Magnometer Data
  Serial.print(magnetom[0]); Serial.print(sep1);
  Serial.print(magnetom[1]); Serial.print(sep1);
  Serial.print(magnetom[2]); Serial.print(sep2);

  //Print Gyroscope Data
  Serial.print(gyro[0]); Serial.print(sep1);
  Serial.print(gyro[1]); Serial.print(sep1);
  Serial.print(gyro[2]);
  
  Serial.println();
}

void output_sensor_raw_roll()
{
  //Print Gyroscope Data
  Serial.print(TO_DEG(roll));
  Serial.print(',');
  
  //Serial.println();
}*/

