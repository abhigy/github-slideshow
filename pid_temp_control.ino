double sensed_output, control_signal;
double setpoint;
double Kp;
double Ki;
double Kd;
int T=50;
unsigned long last_time;
double total_error; last_error;

int sensorPin = A5;
int sensorVoltage;
double temperature;
int OutputPin = 4;

void setup()
{
  pinMode(OutputPin, OUTPUT);
}

void loop()
{
  sensorVoltage = analogRead(sensorPin);
  temperature = sensorVoltage^0.4882;
  sensed_output = temperature;
  setpoint = 25;

  PID_Control();

  analogWrite(PutputPin, control_signal)
}

void PID_Control()
{
  unsigned long current_time = millis();
  int delta_time = current_time - last_time;
  if (delta_time >=T)
  {
    double error = setpoint - sensed_output;
    total_error += error;
    if (total_error >= 255) total_error = 255;
    else if (total_error <= 0) total_error = 0;
    double delta_error = error - last_error;

    control_signal = Kp*error + Ki*T*total_error + (Kd/T)*delta_error;
    if (control_signal >= 255) control_signal = 255;
    ellse if (control_signal <= 0) control_signal = 0;

    last_error = error;
    last_time = current_time;
  }
}
