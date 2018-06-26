#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include <stdio.h>
#include <FaBo9Axis_MPU9250.h>

FaBo9Axis fabo_9axis;

// Derived from https://github.com/MackinnonBuck/ArduinoQuadopter/blob/master/ArduinoQuadcopter/ArduinoQuadcopter.ino, Mackinnon Buck

int t;

Servo frontLeftESC;
Servo frontRightESC;
Servo rearLeftESC;
Servo rearRightESC;

#define OFFSET_X -2.4
#define OFFSET_Y 2.2
#define OFFSET_Z 1.4

#define ESC_FRONT_LEFT_PIN 8
#define ESC_FRONT_RIGHT_PIN 6
#define ESC_REAR_LEFT_PIN 7
#define ESC_REAR_RIGHT_PIN 9

#define YAW_KP              0.1
#define YAW_KI              0.1
#define YAW_KD              0.0

#define PITCHROLL_KP            0.1
#define PITCHROLL_KI            0.1
#define PITCHROLL_KD            0.5

double yawInput;
double yawOutput;
double yawSetpoint = 0;


double rollInput;
double rollOutput;
double rollSetpoint = 0;

double pitchInput;
double pitchOutput;
double pitchSetpoint = 0;

PID yawPID(&yawInput, &yawOutput, &yawSetpoint, YAW_KP, YAW_KI, YAW_KD, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, PITCHROLL_KP, PITCHROLL_KI, PITCHROLL_KD, REVERSE);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, PITCHROLL_KP, PITCHROLL_KI, PITCHROLL_KD, REVERSE);

float gx,gy,gz;

double trim(double value, double min, double max)
{
  return fmax(fmin(value, max), min);
}

double deadZone(double value, double dz)
{
  return value > dz ? value : 0.0;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (fabo_9axis.begin()) {
    Serial.println("configured FaBo 9Axis I2C Brick");
  } 
  else {
    Serial.println("device error");
    while(1);
  }
  // Initailize ESCs
  frontLeftESC.attach(ESC_FRONT_LEFT_PIN);
  frontRightESC.attach(ESC_FRONT_RIGHT_PIN);
  rearLeftESC.attach(ESC_REAR_LEFT_PIN);
  rearRightESC.attach(ESC_REAR_RIGHT_PIN);

  // Start up sequence
  delay(500); 
  int time = millis();
  while(millis()<time+1200){ 
  frontLeftESC.writeMicroseconds(1300);
  frontRightESC.writeMicroseconds(1300);
  rearLeftESC.writeMicroseconds(1300);
  rearRightESC.writeMicroseconds(1300);
  }
  time = millis();
  while(millis()<time+1200){
  frontLeftESC.writeMicroseconds(1000);
  frontRightESC.writeMicroseconds(1000);
  rearLeftESC.writeMicroseconds(1000);
  rearRightESC.writeMicroseconds(1000);
  }


  // Initialize PID algorithms
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-1.0, 1.0);
  yawPID.SetSampleTime(50);

  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-1.0, 1.0);
  rollPID.SetSampleTime(50);

  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-1.0, 1.0);
  pitchPID.SetSampleTime(50);

  t = millis();
}

void loop() {
  if (millis()<t+7000){
  frontLeftESC.writeMicroseconds(1000);
  frontRightESC.writeMicroseconds(1000);
  rearLeftESC.writeMicroseconds(1000);
  rearRightESC.writeMicroseconds(1000);
  } else {
  fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  yawInput = ((gz-OFFSET_Z));
  pitchInput = ((gx-OFFSET_X));
  rollInput = ((gy-OFFSET_Y));

  Serial.print("\n");Serial.print("Yaw:");Serial.print(yawInput);Serial.print("\t");
  Serial.print("Roll:");Serial.print(rollInput);Serial.print("\t");
  Serial.print("Pitch:");Serial.print(pitchInput);Serial.print("\t");
  
//  have to xyz
  Serial.print("\n");Serial.print(gx);
  Serial.print("  ");
  Serial.print(gy);
  Serial.print("  ");
  Serial.print(gz);

  yawPID.Compute();
  rollPID.Compute();
  pitchPID.Compute();

  int frontLeftESC_value = map(trim(-rollOutput + pitchOutput + yawOutput, -2, 2), -2, 2, 1500, 1600);
  int frontRightESC_value = map(trim(rollOutput + pitchOutput - yawOutput, -2, 2), -2, 2, 1500, 1600);
  int rearLeftESC_value = map(trim(-rollOutput - pitchOutput - yawOutput, -2, 2), -2, 2, 1500, 1600);
  int rearRightESC_value = map(trim(rollOutput - pitchOutput + yawOutput, -2, 2), -2, 2, 1500, 1600);

  Serial.print("\n");Serial.print(-rollOutput + pitchOutput + yawOutput);Serial.print("\t");
  Serial.print(rollOutput + pitchOutput - yawOutput);Serial.print("\t");
  Serial.print(-rollOutput - pitchOutput - yawOutput);Serial.print("\t");
  Serial.print(rollOutput - pitchOutput + yawOutput);Serial.print("\t");

  frontLeftESC.writeMicroseconds(frontLeftESC_value);
  frontRightESC.writeMicroseconds(frontRightESC_value);
  rearLeftESC.writeMicroseconds(rearLeftESC_value);
  rearRightESC.writeMicroseconds(rearRightESC_value); 

  Serial.print("\n");Serial.print(frontLeftESC_value);
  Serial.print(frontRightESC_value);
  Serial.print(rearLeftESC_value);
  Serial.print(rearRightESC_value);
  }

  if (Serial.available())
  {
    exit(0);    
  }
}

