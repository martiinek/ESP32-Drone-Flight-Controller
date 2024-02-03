void ControlMotors() {
  sp1 = -rollOutput - pitchOutput + altitudeSetpoint;
  sp2 = -rollOutput + pitchOutput + altitudeSetpoint;
  sp3 = rollOutput - pitchOutput + altitudeSetpoint;
  sp4 = rollOutput + pitchOutput + altitudeSetpoint;

  if (altitudeSetpoint > 100) {
    sp1 += yawOutput;
    sp2 -= yawOutput;
    sp3 -= yawOutput;
    sp4 += yawOutput;
  }

  sp1 = constrain(sp1, 0, 180);
  sp2 = constrain(sp2, 0, 180);
  sp3 = constrain(sp3, 0, 180);
  sp4 = constrain(sp4, 0, 180);

  //   --- MOTOR SPEEDS ---

  // - Serial format -

  /*Serial.print("M1 | ");
  Serial.print(sp1);
  Serial.print(" | M2 | ");
  Serial.print(sp2);
  Serial.print(" | M3 | ");
  Serial.print(sp3);
  Serial.print(" | M4 | ");
  Serial.println(sp4);*/

  if (altitudeSetpoint > 0 && FLMOD && !flightError) {
    escLB.write(sp1);
    escLF.write(sp2);
    escRB.write(sp3);
    escRF.write(sp4);
    //Serial.println("FINE"); //FLYMOD STATUS
  } else {
    escLB.write(0);
    escLF.write(0);
    escRB.write(0);
    escRF.write(0);
    //Serial.println("ERROR"); //FLYMOD STATUS
  }
}
