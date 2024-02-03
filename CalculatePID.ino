double CalculatePID(double Input, double Kp, double Ki, double Kd, double Setpoint, double &LastError, double &Integral, double OutputMin, double OutputMax) {

  double error = Setpoint - Input;

  //anti-windup: back-calculation
  if (Integral > OutputMax) {
    Integral = OutputMax;
  } else if (Integral < OutputMin) {
    Integral = OutputMin;
  }

  double derivative = error - LastError;
  double Output = Kp * error + Integral + Kd * derivative;

  //anti-windup: back-calculation
  if (Output > OutputMax) {
    if (Ki > 0) {  //undo integration
      Integral -= (Output - OutputMax) / Ki;
    }
    Output = OutputMax;
  } else if (Output < OutputMin) {
    if (Ki > 0) {  //undo integration
      Integral -= (Output - OutputMax) / Ki;
    }
    Output = OutputMin;
  }

  LastError = error;
  Integral += error * Ki;

  //   --- PID INFO ---

  // - Serial format -

  /*Serial.print("error | ");
  Serial.print(error);
  Serial.print(" | Setpoint | ");
  Serial.print(Setpoint);
  Serial.print(" | Input ");
  Serial.println(Input);*/

  // - Plot format -
  if (tunning) {
    Serial.print("PTerm:");
    Serial.print(Kp * error);
    Serial.print(",");

    Serial.print("ITerm:");
    Serial.print(Integral);
    Serial.print(",");

    Serial.print("DTerm:");
    Serial.print(Kd * derivative);
    Serial.print(",");

    Serial.print("Output:");
    Serial.print(Output);
    Serial.print(",");

    Serial.print("Setpoint:");
    Serial.println(Setpoint);
  }
  return Output;
}