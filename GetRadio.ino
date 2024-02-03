void GetRadio(double& yawSetpoint, double& rollSetpoint, double& pitchSetpoint, double& altitudeSetpoint, double& seaLevelPres, bool& FLMOD) {
  if (radio.available()) {
    radio.read(&data, sizeof(data));
    yawSetpoint = data[0];
    altitudeSetpoint = data[1];
    altitudeSetpoint = map(altitudeSetpoint, 0, 360, 0, 180);
    if (altitudeSetpoint <= 105) altitudeSetpoint = int(altitudeSetpoint * ((250 - altitudeSetpoint) / 100));
    else {
      altitudeSetpoint += 42;
      altitudeSetpoint = map(altitudeSetpoint, 153, 222, 153, 180);
    }
    rollSetpoint = data[2];
    rollSetpoint = map(rollSetpoint, -45, 45, -30, 30);
    pitchSetpoint = data[3];
    pitchSetpoint = map(pitchSetpoint, -45, 45, -30, 30);
    seaLevelPres = data[4];
    FLMOD = data[5];
    previousMillisRadioError = millis();
  } else {
    currentMillisRadioError = millis();
    if (currentMillisRadioError - previousMillisRadioError > 300) FLMOD = false;
  }
  radio.stopListening();
  radio.openWritingPipe(address[1]);
  if (!flightError) altitude = int(altitudeInput);
  else altitude = 999;
  radio.write(&altitude, sizeof(altitude));
  radio.openReadingPipe(1, address[0]);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  
  /*
  V "prefs" na ovladaci pridat moznost reset, ktera bude slouzit pro resetovani flightErroru pri pretoceni dronu
  */
}
