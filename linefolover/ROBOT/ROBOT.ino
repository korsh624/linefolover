#include <QTRSensors.h>
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading

#define KP 0.1
#define KD -0.05

#define MAX_SPEED 100
#define MIN_SPEED -10
#define DEF_SPEED 100

QTRSensorsAnalog qtra(
(unsigned char[]) {
  0, 1, 2, 3, 4, 5, 6, 7
},
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR
);

unsigned int sensors[NUM_SENSORS];
unsigned int sensorValues[NUM_SENSORS];
int lspeed = 0;
int rspeed = 0;
int lasterror = 0;
bool crossroud = false;
int crosscounter = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(3, OUTPUT); //Левое назад
  pinMode(5, OUTPUT); //Левое вперед
  pinMode(6, OUTPUT); //правое назад
  pinMode(10, OUTPUT); //правое вперед
  pinMode(12, INPUT_PULLUP);
  pinMode(11, OUTPUT);
  digitalWrite(11, 1);

  delay(500);
  Serial.begin(9600);
  Serial.println("Wait for button...");

  while (digitalRead(12) != 0)
    delay(50);

  pinMode(13, OUTPUT);
  analogWrite(3, 160);
  analogWrite(10, 160);
  digitalWrite(13, HIGH);
  Serial.println("Calibration");
  for (int i = 0; i < 100; i++)
  {
    qtra.calibrate();
  }
  digitalWrite(13, LOW);
  digitalWrite(3, LOW);
  digitalWrite(10, LOW);

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }

  Serial.println();
  Serial.println();
  Serial.println("Wait for button...");

  while (digitalRead(12) != 0)
    delay(50);
}

void printline() {
  int position = qtra.readLine(sensors);
  int error = position - 500 * (NUM_SENSORS - 1);
  double correction = (double)KP * (double)error + (double)KD * ((double)lasterror - (double)error);
  lasterror = error;
  //  Serial.print (correction);
}

void chek_crossroad() {
  /*
    for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensors[i]);
    Serial.print(" ");
    }
    Serial.println();
  */
  if (!crossroud) {
    if ((sensors[0] > 850) && (sensors[7] > 850)) {
      Serial.println("CrossRoad");
      crosscounter++;
      crossroud = true;
    }
  }
  else {
    if ((sensors[0] < 850) && (sensors[7] < 850)) {
      crossroud = false;
    }
  }
  /*
    Serial.print("crosscounter");
    Serial.println(crosscounter);
  */
}

void linefolov() {
  int position = qtra.readLine(sensors);
  int error = position - 500 * (NUM_SENSORS - 1);
  double correction = (double)KP * (double)error + (double)KD * ((double)lasterror - (double)error);
  lasterror = error;
  rspeed = speedclien(DEF_SPEED - correction);
  lspeed = speedclien(DEF_SPEED + correction);
  /*
    Serial.print(position);
    Serial.print(" -- ");
    Serial.print(error);
    Serial.print(" -- ");
    Serial.print(correction);
    Serial.print(" -- ");
    Serial.print(lspeed);
    Serial.print("  ");
    Serial.print(rspeed);
    Serial.print("  ");
    Serial.println();
  */
  //  return;

  if (rspeed > 0) {
    digitalWrite(6, 0);
    analogWrite(10, rspeed);
  } else {
    digitalWrite(10, 0);
    analogWrite(6, -rspeed);
  }
  if (lspeed > 0) {
    digitalWrite(3, 0);
    analogWrite(5, lspeed);
  } else {
    digitalWrite(5, 0);
    analogWrite(3, -lspeed);
  }
}

int speedclien(int sp) {
  if (sp > MAX_SPEED) {
    sp = MAX_SPEED;
  }
  if (sp < MIN_SPEED) {
    sp = MIN_SPEED;
  }
  return sp;
}

void stopm() {
  digitalWrite(3, 0);
  digitalWrite(10, 0);
  digitalWrite(5, 0);
  digitalWrite(6, 0);
}

void moveFwd(int cnt) {
  crosscounter = 0;
  while (crosscounter < cnt) {
    linefolov();
    chek_crossroad();
  }
  stopm();
}
//void moveLeft() {
//  analogWrite(10, 170);
//  delay(400);
//  analogWrite(10, 0);
//}
void loop()
{
//  moveFwd(3);
//  moveLeft();
//  moveFwd(1);
//  moveLeft();
//  moveFwd(3);
//  while (1);
}
