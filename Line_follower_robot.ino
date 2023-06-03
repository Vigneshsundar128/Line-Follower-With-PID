#include <DRV8833.h>

// Create an instance of the DRV8833:
DRV8833 driver = DRV8833();

// Ain1, Ain2, Bin1, and Bin2 DRV8833 pins.
const int inputA1 = 5, inputA2 = 6, inputB1 = 9, inputB2 = 10;

// The speed of the motors:
const int Lmotorspeed = 255; // Left speed control
const int Rmotorspeed = 255; //Right speed control

int P, D, I, previousError, PIDvalue, error;
int Lspeed, Rspeed;

float Kp = 100;
float Kd = 0;
float Ki = 0 ;


int minValues[6], maxValues[6], threshold[6];

void setup()
{
  Serial.begin(9600);

  // Attach the motors to the input pins:
  driver.attachMotorA(inputA1, inputA2);
  driver.attachMotorB(inputB1, inputB2);

  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
}

void loop()
{
  while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  while (digitalRead(12)) {}
  delay(1000);

  while (1)
  {
    if (analogRead(1) > threshold[1] && analogRead(5) < threshold[5] )
    {
      Lspeed = 0; Rspeed = Rmotorspeed;
      driver.motorAStop();
      driver.motorBForward(Rmotorspeed);
    }

    else if (analogRead(5) > threshold[5] && analogRead(1) < threshold[1])
    { Lspeed = Lmotorspeed; Rspeed = 0;
      driver.motorAForward(Lmotorspeed);
      driver.motorBStop();
    }
    else if (analogRead(3) > threshold[3])
    {
      Kp = 0.0006 * (1000 - analogRead(2));
      Kd = 10 * Kp;
      //Ki = 0.0001;
      Serial.print("kp");
      Serial.print(" ");
      Serial.print("kd");
      Serial.println(" ");
      linefollow();
    }
    else if (analogRead(3) > threshold[4])
    {
      Kp = 0.0006 * (1000 - analogRead(3));
      Kd = 10 * Kp;
      //Ki = 0.0001;
      Serial.print("kp");
      Serial.print(" ");
      Serial.print("kd");
      Serial.println(" ");
      linefollow();
    }
  }
}

void linefollow()
{
  int error = (analogRead(2) - analogRead(4));

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  Lspeed = Lmotorspeed - PIDvalue;
  Rspeed = Rmotorspeed + PIDvalue;

  if (Lspeed > 255) {
    Lspeed = 255;
  }
  if (Lspeed < 0) {
    Lspeed = 0;
  }
  if (Rspeed > 255) {
    Rspeed = 255;
  }
  if (Rspeed < 0) {
    Rspeed = 0;
  }
  // Put the motors in forward using the speed:
  driver.motorAForward(Lspeed);
  driver.motorBForward(Rspeed);

}

void calibrate()
{
  for ( int i = 0; i < 6; i++)
  {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  
  for (int i = 0; i < 3000; i++)
  {
    // rotate robot to calibrate
    driver.motorAForward(Lmotorspeed);
    driver.motorBForward(Rmotorspeed);

    for ( int i = 0; i < 6; i++)
    {;;
      if (analogRead(i) < minValues[i])
      {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i])
      {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for ( int i = 0; i < 6; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
  driver.motorAStop();
  driver.motorBStop();
}
