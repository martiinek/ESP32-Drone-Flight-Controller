static String LastConsPID;
int counterBluetooth = 0;
void Bluetooth(double& pitchKp, double& pitchKi, double& pitchKd) {
  if (counterBluetooth != 1) counterBluetooth += 1;
  if (SerialBT.available()) {
    switch (SerialBT.read()) {
      case 'A':
        pitchKp += 0.1;
        break;
      case 'B':
        pitchKp -= 0.1;
        break;
      case 'C':
        EEPROM.write(0, float(pitchKp * 1000));
        Serial.println("Saving");
        EEPROM.commit();
        break;
      case 'D':
        pitchKi += 0.001;
        break;
      case 'E':
        pitchKi -= 0.001;
        break;
      case 'F':
        EEPROM.write(4, float(pitchKi * 1000));
        Serial.println("Saving");
        EEPROM.commit();
        break;
      case 'G':
        pitchKd += 0.1;
        break;
      case 'H':
        pitchKd -= 0.1;
        break;
      case 'I':
        EEPROM.write(8, float(pitchKd * 1000));
        Serial.println("Saving");
        EEPROM.commit();
        break;
    }
    rollKp = pitchKp;
    rollKi = pitchKi;
    rollKd = pitchKd;
  }
  String ConsPID = "(" + String(pitchKp) + "|" + String(pitchKi) + "|" + String(pitchKd) + ")";
  currentMillisBluetooth = millis();
  if (((currentMillisBluetooth - previousMillisBluetooth) >= bluetoothInterval) || counterBluetooth == 1) {
    SerialBT.println(ConsPID);
  }
  LastConsPID = ConsPID;
}