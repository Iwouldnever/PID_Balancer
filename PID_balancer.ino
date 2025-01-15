#include "arduino_secrets.h"

#include <Servo.h>
#define sensor A1
Servo myservo; 
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(9);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float distance = distance_read();
  int reading = analogRead(A1);
  SetTunings(2.5, 0.002, 500);
  Compute();
  Serial.print(analogRead(A1));
  Serial.print("   ");
  double temp = map(Output, -2000, 2000, 180, 0);
  double print = constrain(temp, 0, 180);
  Serial.print(print);
  Serial.print("  ");
  Serial.println(Output);
  
  myservo.write(print); 

}

void Compute()
{

unsigned long now = millis();
double timeChange = (double)(now - lastTime);

double Setpoint = 305;
double Input = analogRead(A1); 
if(Input < 10){
  Input += 590;
}
double error = Setpoint - Input;
errSum += (error * timeChange);
double dErr = (error - lastErr) / timeChange;
 

Output = kp * error + ki * errSum + kd * dErr;
 

lastErr = error;
lastTime = now;
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
kp = Kp;
ki = Ki;
kd = Kd;
}