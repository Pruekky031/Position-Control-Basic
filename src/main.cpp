#include <Arduino.h>
// #include <teleplot.h>
#define ENCA 2
#define ENCB 3
#define PPR 224.52

#define MOTOR_PIR1 8
#define MOTOR_PIR2 13
#define MOTOR_PWM 10

float velocity = 0.0f;
int palue = 0.0f;
long prevTime = 0.0f;
int dir;
float rpm;
float motor_angle;
long Time = 0;
float revCount = 0.0f;
unsigned long previousTime;

float SetPoint = -200.0f;

void readEncoder();
float PIDSpeedControl(float SetPoint, float MotorRPM);
void SetMotorPWM(float PWM);

void setup()
{
  Serial.begin(115200);
  Serial.println("Start//");
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  pinMode(MOTOR_PIR1, OUTPUT);
  pinMode(MOTOR_PIR2, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

// void loop()
// {

//   if (Serial.available() > 0)
//   {
//     SetPoint = Serial.readStringUntil('\n').toFloat();
//   }
//   unsigned long currentTime = millis();
//   float dt = (currentTime - previousTime) / 1000.0;

//   if (dt >= 0.01)
//   {

//     unsigned long currentTime = millis();
//     float dt = (currentTime - previousTime) / 1000.0;
//     if (dt >= 0.01)
//     {

//       rpm = (velocity / PPR) * 60.0;
//       revCount = (palue / PPR);
//       motor_angle = (palue / PPR) * 360.0;
//       if (motor_angle < 0)
//       {
//         motor_angle = 360 + fmod(motor_angle, 360.0);
//       }
//       else
//       {
//         motor_angle = fmod(motor_angle, 360.0);
//       }
//       SetMotorPWM(100);
//       Serial.print(">motor_angle:");
//       Serial.println(motor_angle);
//       Serial.print(">revCount:");
//       Serial.println(revCount);
//       Serial.print(">rpm  :");
//       Serial.println(rpm);
//       // if (SetPoint != 0)
//       // {
//       //   float Output = PIDSpeedControl(abs(SetPoint), abs(rpm));
//       //   SetMotorPWM(Output);
//       // }
//       velocity = 0;
//     }
//   }
// }

void loop()
{
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  if (dt >= 0.01)
  {

    rpm = (velocity / PPR) * 60.0;
    velocity = 0;
    revCount = (palue / PPR);
    motor_angle = (palue / PPR) * 360.0;
    if (motor_angle < 0)
    {
      motor_angle = 360 + fmod(motor_angle, 360.0);
    }
    else
    {
      motor_angle = fmod(motor_angle, 360.0);
    }

    if (SetPoint != 0)
    {
      float Output = PIDSpeedControl(SetPoint, abs(rpm));
      SetMotorPWM(Output);
    }
  }
}

void readEncoder()
{
  unsigned long currentTime = micros();
  if (digitalRead(ENCB) == 1)
  {
    dir = 1;
    palue += 1;
  }
  else
  {
    dir = -1;
    palue -= 1;
  }
  float deltaTime = (currentTime - prevTime) / 1000000.0;
  if (deltaTime > 0)
  {
    velocity = 1.0 / deltaTime;
  }
  prevTime = currentTime;
}

float PIDSpeedControl(float SetPoint, float MotorRPM)
{

  static float P_SpeedControl;
  static float I_SpeedControl;
  static float D_SpeedControl;

  float Kp;
  float Ki;
  float Kd;

  SetPoint = 75.0f;
  // Setpoint direction + or -
  if (SetPoint > 0)
  {
    if (SetPoint >= 150)
    {
      // high gain direction +

      Kp = 3.3f;
      Ki = 0.05f;
      Kd = 0.0f;
    }
    else
    {
      // low gain direction +

      Kp = 3.3f;
      Ki = 0.02f;
      Kd = 0.0f;
    }
  }
  else
  {
    if (SetPoint <= -150)
    {
      // high gain direction -

      Kp = 3.5f;
      Ki = 0.03f;
      Kd = 0.03f;
    }
    else
    {
      // low gain direction -

      Kp = 2.2f;
      Ki = 0.05f;
      Kd = 0.01f;
    }
  }

  float Error_SpeedControl;
  static float LastError_SpeedControl;
  static float SumError;
  float MaxSumError = 1000.0f;
  float Output_SpeedControl;
  static unsigned long MotorDoneTime;
  static unsigned long PIDLoopTime_SpeedControl;

  if (millis() - PIDLoopTime_SpeedControl >= 10)
  {

    Error_SpeedControl = abs(SetPoint) - MotorRPM;

    SumError = SumError + Error_SpeedControl;

    // sum P term
    P_SpeedControl = Kp * Error_SpeedControl;

    // limit SumError to prevent integral windup
    if (SumError > MaxSumError)
      SumError = MaxSumError;
    if (SumError < -MaxSumError)
      SumError = -MaxSumError;

    // delete I term if motor is Stopped for more than 70ms to prevent integral windup
    if (millis() - MotorDoneTime >= 70)
    {
      I_SpeedControl = 0;
      MotorDoneTime = millis();
    }

    // Sum I term
    I_SpeedControl = I_SpeedControl + (Ki * SumError);

    // Sum D term
    D_SpeedControl = Kd * (Error_SpeedControl - LastError_SpeedControl) / 10.0;

    // sum PID terms
    Output_SpeedControl = P_SpeedControl + I_SpeedControl + D_SpeedControl;

    // save Error to LastError for next loop
    LastError_SpeedControl = Error_SpeedControl;

    // save current time for next loop
    PIDLoopTime_SpeedControl = millis();

    if (Output_SpeedControl > 255)
      Output_SpeedControl = 255;
    if (Output_SpeedControl < 0)
      Output_SpeedControl = 0;

    Serial.print(">SetPoint: ");
    Serial.println(SetPoint);
    if (SetPoint > 0)
    {
      Serial.print(">MotorRPM: ");
      Serial.println(MotorRPM * 1);
    }
    else
    {
      Serial.print(">MotorRPM: ");
      Serial.println(MotorRPM * -1);
    }

    Serial.print(">Error: ");
    Serial.println(Error_SpeedControl);
    Serial.print(">Output: ");
    Serial.println(Output_SpeedControl);
    Serial.print(">Max: ");
    Serial.println(SetPoint + 50);
    Serial.print(">Min: ");
    Serial.println(SetPoint - 50);

    Serial.print(">Kp:");
    Serial.println(Kp);

    Serial.print(">Ki:");
    Serial.println(Ki);

    Serial.print(">Kd:");
    Serial.println(Kd);

    Serial.print(">P_SpeedControl:");
    Serial.println(P_SpeedControl);

    Serial.print(">I_SpeedControl:");
    Serial.println(I_SpeedControl);

    Serial.print(">D_SpeedControl:");
    Serial.println(D_SpeedControl);

    return Output_SpeedControl;
  }
}
void SetMotorPWM(float PWM)
{
  analogWrite(MOTOR_PWM, abs(PWM));
  if (SetPoint > 0)
  {
    digitalWrite(MOTOR_PIR1, HIGH);
    digitalWrite(MOTOR_PIR2, LOW);
  }
  else if (SetPoint < 0)
  {
    digitalWrite(MOTOR_PIR1, LOW);
    digitalWrite(MOTOR_PIR2, HIGH);
  }
  else
  {
    digitalWrite(MOTOR_PIR1, LOW);
    digitalWrite(MOTOR_PIR2, LOW);
  }
}
