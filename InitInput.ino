void InitInput() {
  Serial.begin(115200);
  SerialBT.begin("DRON MK1");
  radio.begin();
  Wire.begin();
  bme.begin();
  mag.begin();
  EEPROM.begin(512);
  radio.setPALevel(RF24_PA_MAX);
  escRB.attach(esc1, 1000, 2000);
  escRF.attach(esc2, 1000, 2000);
  escLB.attach(esc3, 1000, 2000);
  escLF.attach(esc4, 1000, 2000);
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  pitchKp = double(EEPROM.read(0)) / 1000;
  pitchKi = double(EEPROM.read(4)) / 1000;
  pitchKd = double(EEPROM.read(8)) / 1000;

  rollAccOffset = double(EEPROM.read(12)) / 1000;
  pitchAccOffset = double(EEPROM.read(16)) / 1000;
  rollGyroOffset = double(EEPROM.read(20)) / 1000;
  pitchGyroOffset = double(EEPROM.read(24)) / 1000;

  if (rollAccOffset == 0) {
    for (int i = 0; i < 500; i++) {

      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr, 6, true);

      xAcc = (int16_t(Wire.read() << 8 | Wire.read())) / 16384.0;
      yAcc = (int16_t(Wire.read() << 8 | Wire.read())) / 16384.0;
      zAcc = (int16_t(Wire.read() << 8 | Wire.read())) / 16384.0;

      rollAccOffset += atan(yAcc / sqrt(xAcc * xAcc + zAcc * zAcc)) * RAD_TO_DEG;
      pitchAccOffset += atan2(-xAcc, zAcc) * RAD_TO_DEG;
      if (i == 499) {
        rollAccOffset = rollAccOffset / 500;
        pitchAccOffset = pitchAccOffset / 500;
        EEPROM.write(12, float(rollAccOffset * 1000));
        EEPROM.write(16, float(pitchAccOffset * 1000));
      }
    }
  }


  if (rollGyroOffset == 0) {
    for (int i = 0; i < 500; i++) {

      Wire.beginTransmission(MPU_addr);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr, 6, true);

      xGyro = (int16_t(Wire.read() << 8 | Wire.read())) / 131.0;
      yGyro = (int16_t(Wire.read() << 8 | Wire.read())) / 131.0;
      zGyro = (int16_t(Wire.read() << 8 | Wire.read())) / 131.0;

      rollGyroOffset += yGyro;
      pitchGyroOffset += xGyro;
      if (i == 499) {
        rollGyroOffset = rollGyroOffset / 500;
        pitchGyroOffset = pitchGyroOffset / 500;
        EEPROM.write(20, float(rollGyroOffset * 1000));
        EEPROM.write(24, float(pitchGyroOffset * 1000));
      }
    }
  }

  EEPROM.commit();

  xTaskCreatePinnedToCore(
    GetRadioTaskFunction,   /* Funkce úkolu. */
    "GetRadioTask",         /* Název úkolu. */
    10000,                  /* Velikost zásobníku úkolu. */
    NULL,                   /* Parametr úkolu. */
    1,                      /* Priorita úkolu. */
    &GetRadioTask,          /* Ukazatel na úkol pro sledování vytvořeného úkolu. */
    1);

  sensors_event_t event;
  mag.getEvent(&event);

  headingOffset = atan2(event.magnetic.y, event.magnetic.x);
  headingOffset -= declinationAngle;
  if (headingOffset < 0) {
    headingOffset += 2 * M_PI;
  }
  headingOffset = headingOffset * 180 / M_PI;
  headingOffset -= 180;

  Serial.println("");
  Serial.println("|---------------------------------------|");
  Serial.println("|                                       |");
  Serial.println("|       STG                             |");
  Serial.println("|             FLIGHT                    |");
  Serial.println("|                      CONTROLLER       |");
  Serial.println("|                                       |");
  Serial.println("|---------------------------------------|");
  Serial.println("");

  Serial.println("PID EEPROM VALUES");
  Serial.println("");
  Serial.println("-----------------");
  Serial.println("");

  Serial.print("pitchKp | ");
  Serial.print(pitchKp);
  Serial.print(" | pitchKi | ");
  Serial.print(pitchKi);
  Serial.print(" | pitchKd | ");
  Serial.println(pitchKd);
}
