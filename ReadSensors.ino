void ReadSensors(double& pitchInput, double& rollInput, double& yawInput, double& altitudeInput) {

  //IMU

  //accelometer

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  xAcc = (int16_t(Wire.read() << 8 | Wire.read())) / 16384.0;
  yAcc = (int16_t(Wire.read() << 8 | Wire.read())) / 16384.0;
  zAcc = (int16_t(Wire.read() << 8 | Wire.read())) / 16384.0;

  //gyro
  deltaGyroTime = (double)(micros() - gyroMicros) / 1000000;
  gyroMicros = micros();

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  xGyro = (int16_t(Wire.read() << 8 | Wire.read())) / 131.0;
  yGyro = (int16_t(Wire.read() << 8 | Wire.read())) / 131.0;
  zGyro = (int16_t(Wire.read() << 8 | Wire.read())) / 131.0;

  pitchInputGyro = (xGyro - pitchGyroOffset) * deltaGyroTime;
  rollInputGyro = (yGyro - rollGyroOffset) * deltaGyroTime;

  double rollInputAcc = atan(yAcc / sqrt(xAcc * xAcc + zAcc * zAcc)) * RAD_TO_DEG - rollAccOffset;
  double pitchInputAcc = atan2(-xAcc, zAcc) * RAD_TO_DEG - pitchAccOffset;

  rollInput = 0.9 * (rollInput + rollInputGyro) + 0.1 * (rollInputAcc);
  pitchInput = 0.9 * (pitchInput + pitchInputGyro) + 0.1 * (pitchInputAcc);

  //compass
  sensors_event_t event;
  mag.getEvent(&event);

  heading = atan2(event.magnetic.y, event.magnetic.x);
  heading -= declinationAngle;
  if (heading > 0) heading -= 2 * PI;
  if (heading < 0) heading += 2 * PI;
  heading *= 180 / PI;
  if (heading < headingOffset) {
    yawInput = 360 - (headingOffset - heading);
  } else {
    yawInput = heading - headingOffset;
  }
  yawInput = fmod(yawInput, 360);
  if (yawInput < 0) {
    yawInput += 360;
  }
  yawInput = map(yawInput, 0, 360, -180, 180);


  //altitudemeter

  //bmp280
  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  pres = pres / 100;
  counterAltitude += 1;

  //kalman's altitude filter
  float xPredTemp = xTemp;
  float pPredTemp = pTemp + QTemp;
  float xPredPres = xPres;
  float pPredPres = pPres + QPres;

  kTemp = pPredTemp / (pPredTemp + RTemp);
  kPres = pPredPres / (pPredPres + RPres);

  xTemp = xPredTemp + kTemp * (temp - xPredTemp);
  xPres = xPredPres + kPres * (pres - xPredPres);

  pTemp = (1 - kTemp) * pPredTemp;
  pPres = (1 - kPres) * pPredPres;
  
  //barometric equation
  altitudeInput = (((pow((1013.15 / xPres), 1 / 5.257) - 1) * (xTemp + 273.15)) / 0.0065);
  if (altitudeInputOffset == 0 && counterAltitude == 50) {
    altitudeInputOffset = altitudeInput;
    yawInput = 0;
  }
  altitudeInput -= altitudeInputOffset;
  altitudeInput = constrain(altitudeInput, 0, 1000);


  //   --- IMU INFO ---

  // - Plot format -

  // IMU
  /*Serial.print("pitchInput:");
  Serial.print(pitchInput);
  Serial.print(",");

  Serial.print("rollInput:");
  Serial.print(rollInput);
  Serial.print(",");

  Serial.print("yawInput:");
  Serial.println(yawInput);*/



  // - Serial format -

  // Heading calculation
  /*Serial.print("Offset | ");
  Serial.print(headingOffset);
  Serial.print(" | Heading | ");
  Serial.println(heading);
  Serial.print(" | Setpoint | ");
  Serial.print(yawSetpoint);
  Serial.print(" | Input ");
  Serial.println(yawInput);*/

  // IMU output
  /*Serial.print("Pitch | ");
  Serial.print(pitchInput);
  Serial.print(" | Roll | ");
  Serial.println(rollInput);
  Serial.print(" | Yaw | ");
  Serial.print(yawInput);
  Serial.print(" | Altitude ");
  Serial.println(altitudeInput);*/
}
