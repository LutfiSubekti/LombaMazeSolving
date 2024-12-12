#include <Servo.h>
#include "HardwareConfig.h"

//ENCODER
#include <ESP32Encoder.h>
ESP32Encoder encoderL;
ESP32Encoder encoderR;


//STRATEGI
unsigned long startHold, endHold;
const long waitTime = 2000;

//SERVO
Servo servoA;
Servo servoC;
int defServoA = 90; //naikmax 180
int defServoC = 90; //tutupmax 180

int servoAMIN = 0, servoAMAX = 80;
int servoCMIN = 90, servoCMAX = 140;
bool stateNaik = true;
bool stateTutup = false;


//--------OLED
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#define OLED_ADDR   0x3C
Adafruit_SSD1306 display(-1);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

//----------color sensor
#include "Adafruit_TCS34725.h"

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();
/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);


//----------save data
#include <EEPROM.h>

struct EEPROMDATA {
  int f_SENSOR_REFF[8];
  int b_SENSOR_REFF[8];
};
EEPROMDATA EE;


#define front 0
#define rear 1
#define fast 0
#define slow 1
#define maksi 2
#define f1 0
#define f2 1
#define f3 2
#define l1 3
#define l2 4
#define l3 5
#define l4 6
#define r1 7
#define r2 8
#define r3 9
#define r4 10

int f_sensor[8], b_sensor[8];
bool f_valSensor[8], b_valSensor[8];
byte bitSensor = 0;
//int b_bitSensor = 0;

bool hitam = 1;
bool putih = 0;

int sensiSensor = 0;

bool awal = true;
int jalan = 1;
//int color;


//--------var timer------------//
int count = 0;
unsigned long previousMillis = 0;

//----------PID---------------//
int error = 0;
int lastError = 0;
byte kp = 30;
byte kd = 150;
byte SPEED = 200;
int MIN_SPEED = -160;
byte MAX_SPEED = 200;
//----------------------------//

bool pos_sensor = 0;

// global variables needed for driveStraight() function
int leftPower, rightPower = leftPower;
long prevLeftCount = 0, prevRightCount = 0;

//------BUTTON----//
#define BTN_START digitalRead(pin_BTN_START)
//#define BTN_KALIB digitalRead(pin_BTN_KALIB)

void setup() {
  Serial.begin(115200);
  EEPROM.begin(1024);  
  
  //motor
  ledcSetup(M1A_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M1B_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M2A_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M2B_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(pin_MOTOR_DIRL, M1A_PWM_CHANNEL);
  ledcAttachPin(pin_MOTOR_PWML, M1B_PWM_CHANNEL);
  ledcAttachPin(pin_MOTOR_DIRR, M2A_PWM_CHANNEL);
  ledcAttachPin(pin_MOTOR_PWMR, M2B_PWM_CHANNEL);

  //servo
  servoA.attach(servo_angkat);
  servoC.attach(servo_capit);
  servoA.write(defServoA);  //Default
  servoC.write(defServoC);  //Default

  //sensor garis
  pinMode(f_sel1, OUTPUT);
  pinMode(f_sel2, OUTPUT);
  pinMode(f_sel3, OUTPUT);
  pinMode(f_pin_sensor, INPUT);

  pinMode(b_sel1, OUTPUT);
  pinMode(b_sel2, OUTPUT);
  pinMode(b_sel3, OUTPUT);
  pinMode(b_pin_sensor, INPUT);

  //buzzer
  pinMode(buzz, OUTPUT);

  //button
  pinMode(pin_BTN_START, INPUT_PULLUP);
  //pinMode(pin_BTN_KALIB, INPUT_PULLUP);


  turun(120,1,0);
  buka(80,1,0);
  
  //encoder
  // Enable the weak pull down resistors
  //ESP32Encoder::useInternalWeakPullResistors=DOWN;
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = UP;

  encoderL.attachHalfQuad(36, 39);
  encoderR.attachHalfQuad(4, 5);

  encoderL.clearCount();
  encoderR.clearCount();

  //display oled
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(27, 30);
  display.print("Ruang Robot");
  display.display();

  delay(500);

  display.clearDisplay();
  display.display();

  EEPROM.get(0, EE);
}

void loop() {
  sensitivitas();
  displaySensor();
  displayEncoder();
  display.clearDisplay();

  if (encoderR.getCount() <= 200 || encoderR.getCount() > 500) {
    display.clearDisplay();
    display.setCursor(27, 10);
    display.print("Ready Start");
    display.display();
    if (!BTN_START) {
      strategi0();
    }
  }

  else if (encoderR.getCount() >= 200 && encoderR.getCount() <= 500) {
    display.clearDisplay();
    display.setCursor(20, 10);
    display.print("Kalibrasi Garis");
    display.display();
    if (!BTN_START) {
      kalibrasiSensor();
    }
  }

}

void jalanLurus(int Power) {
  leftPower = Power;

  // use wheel encoders to drive straight continuously

  // amount to offset motor powers to drive straight
  int offset = 5;

  // get current wheel encoder counts
  int leftCount = encoderL.getCount();
  int rightCount = encoderR.getCount();

  // calculate increase in count from previous reading
  long leftDiff = leftCount - prevLeftCount;
  long rightDiff = rightCount - prevRightCount;

  // store current counts as "previous" counts for next reading
  prevLeftCount = leftCount;
  prevRightCount = rightCount;

  // adjust left & right motor powers to keep counts similar (drive straight)
  // if left rotated more than right, slow down left & speed up right
  if (leftDiff > rightDiff) {
    leftPower = leftPower - offset;
    rightPower = rightPower + offset;
  }
  // if right rotated more than left, speed up left & slow down right
  else if (leftDiff < rightDiff) {
    leftPower = leftPower + offset;
    rightPower = rightPower - offset;
  }

  // apply adjusted motor powers
  setMotor(leftPower, rightPower);
  //delay(10);  // short delay before next reading
}

void followJarak(float distance, int xspeed, int lama) {
  pilihSpeed(xspeed);
  float correction = -1.0; // need decimal point for float value
  if (distance > 0) distance += correction;
  else if (distance < 0) distance -= correction;

  // variables for tracking wheel encoder counts
  long leftCount = 0;
  long rightCount = 0;

  // RedBot values based on encoders, motors & wheels
  float countsPerRev = 422.0; // 192 encoder ticks per wheel revolution
  float wheelDiam = 4.5;  // wheel diameter = 65 mm = 2.56 in
  float wheelCirc = PI * wheelDiam; // wheel circumference = 3.14 x 2.56 in = 8.04 in

  // based on distance, calculate number of wheel revolutions
  float numRev = distance / wheelCirc;

  // calculate target encoder count
  float targetCount = numRev * countsPerRev;

  // reset encoder counters and start driving
  encoderL.clearCount();
  encoderR.clearCount();
  //delay(100);

  //setMotor(leftPower, rightPower);
  followLine();

  // keeps looping while right encoder count less than target count
  while (abs(rightCount) < abs(targetCount)) {

    // get current wheel encoder counts
    //leftCount = encoderL.getCount();
    rightCount = encoderR.getCount();

    followLine();

  }

  // target count reached
  rem(lama);
  encoderL.clearCount();
  encoderR.clearCount();
}

//void percabanganJarak(float distance, float offset_jarak, int xspeed, byte mode, int lama) {
//  pilihSpeed(xspeed/2);
//  
//  float correction = -1.0; // need decimal point for float value
//  if (distance > 0) distance += correction;
//  else if (distance < 0) distance -= correction;
//
//  // variables for tracking wheel encoder counts
//  long leftCount = 0;
//  long rightCount = 0;
//
//  // RedBot values based on encoders, motors & wheels
//  float countsPerRev = 422.0; // 192 encoder ticks per wheel revolution
//  float wheelDiam = 4.5;  // wheel diameter = 65 mm = 2.56 in
//  float wheelCirc = PI * wheelDiam; // wheel circumference = 3.14 x 2.56 in = 8.04 in
//
//  // based on distance, calculate number of wheel revolutions
//  float numRev = distance / wheelCirc;
//  float numRev_start = offset_jarak / wheelCirc;
//  float numRev_finish = (distance-offset_jarak) / wheelCirc;
//
//  // calculate target encoder count
//  float targetCount = numRev * countsPerRev;
//  float targetCount_start = numRev_start * countsPerRev;
//  float targetCount_finish = numRev_finish * countsPerRev;
//
//  // reset encoder counters and start driving
//  encoderL.clearCount();
//  encoderR.clearCount();
//  //delay(100);
//
//  //setMotor(leftPower, rightPower);
//  followLine();
//
//  // keeps looping while right encoder count less than target count
//  while (abs(rightCount) < abs(targetCount)) {
//
//    // get current wheel encoder counts
//    //leftCount = encoderL.getCount();
//    rightCount = encoderR.getCount();
//
//    if((abs(rightCount) < abs(targetCount_start)) || (abs(rightCount) > abs(targetCount_finish))){
//      pilihSpeed(xspeed/2); 
//    }else{
//      pilihSpeed(xspeed);
//    }
//    
//    followLine();
//  }
//
//  // target count reached
////  rem(lama);
//  encoderL.clearCount();
//  encoderR.clearCount();
//
//  percabangan(xspeed/2,mode,lama);
//}

void percabanganAccel(float jarak, int speedx, byte modex, unsigned int lama){
  int offset = 0.1 * jarak;
  followJarak(offset, speedx/2, 0);
  followJarak(jarak-(offset*2), speedx, 0);
  followJarak(offset, speedx/2, 0);
  percabangan(speedx/2, modex, lama);
}

void followJarakAccel(float jarak, int speedx, unsigned int lama){
  int offset = 0.1 * jarak;
  followJarak(offset, speedx/2, 0);
  followJarak(jarak-(offset*2), speedx, 0);
  followJarak(offset, speedx/2, lama);
}

void moveJarak(float distance, int xspeed, int lama) {

  // use wheel encoders to drive straight for specified distance at specified power

  // set initial power for left and right motors
  int leftPower = xspeed;
  int rightPower = leftPower;

  // amount to offset motor powers to drive straight
  int offset = 5;

  // if negative distance, make motor powers & offset also negative
  if (distance < 0) {
    leftPower *= -1;
    rightPower *= -1;
    offset *= -1;
  }

  // use correction to improve distance accuracy
  // adjust correction value based on test results
  float correction = -1.0; // need decimal point for float value
  if (distance > 0) distance += correction;
  else if (distance < 0) distance -= correction;

  // variables for tracking wheel encoder counts
  long leftCount = 0;
  long rightCount = 0;
  long prevLeftCount = 0;
  long prevRightCount = 0;
  long leftDiff, rightDiff;

  // RedBot values based on encoders, motors & wheels
  float countsPerRev = 422.0; // 192 encoder ticks per wheel revolution
  float wheelDiam = 4.5;  // wheel diameter = 65 mm = 2.56 in
  float wheelCirc = PI * wheelDiam; // wheel circumference = 3.14 x 2.56 in = 8.04 in

  // based on distance, calculate number of wheel revolutions
  float numRev = distance / wheelCirc;

  // calculate target encoder count
  float targetCount = numRev * countsPerRev;

  // reset encoder counters and start driving
  encoderL.clearCount();
  encoderR.clearCount();
  //delay(100);

  setMotor(leftPower, rightPower);

  // keeps looping while right encoder count less than target count
  while (abs(rightCount) < abs(targetCount)) {

    // get current wheel encoder counts
    leftCount = encoderL.getCount();
    rightCount = encoderR.getCount();

    // calculate increase in count from previous reading
    leftDiff = abs(leftCount - prevLeftCount);
    rightDiff = abs(rightCount - prevRightCount);

    // store current counts as "previous" counts for next reading
    prevLeftCount = leftCount;
    prevRightCount = rightCount;

    // adjust left & right motor powers to keep counts similar (drive straight)

    // if left rotated more than right, slow down left & speed up right
    if (leftDiff > rightDiff) {
      leftPower = leftPower - offset;
      rightPower = rightPower + offset;
    }
    // else if right rotated more than left, speed up left & slow down right
    else if (leftDiff < rightDiff) {
      leftPower = leftPower + offset;
      rightPower = rightPower - offset;
    }

    // apply adjusted motor powers
    setMotor(leftPower, rightPower);
    //delay(10);  // short delay before next reading
  }

  // target count reached
  rem(lama);
  encoderL.clearCount();
  encoderR.clearCount();
}

void belokSudut(float angle, int xspeed, int lama) {

  // use wheel encoders to pivot (turn) by specified angle

  // set motor power for pivoting
  int power = xspeed; // clockwise

  // use correction to improve angle accuracy
  // adjust correction value based on test results
  float correction = -5.0; // need decimal point for float value
  if (angle > 0) angle += correction;
  else if (angle < 0) angle -= correction;

  // variable for tracking wheel encoder counts
  long rightCount = 0;
  long leftCount = 0;

  // values based on RedBot's encoders, motors & wheels
  float countsPerRev = 422.0; // 192 encoder ticks per wheel revolution
  float wheelDiam = 4.5; // wheel diameter = 65 mm = 2.56 in
  float wheelCirc = PI * wheelDiam; // wheel circumference = 3.14 x 2.56 in = 8.04 in
  float pivotDiam = 14.25; // pivot diameter = distance between centers of wheel treads = 6.125 in
  float pivotCirc = PI * pivotDiam; // pivot circumference = 3.14 x 6.125 in = 19.23 in

  // based on angle, calculate distance (arc length) for pivot
  float distance = abs(angle) / 360.0 * pivotCirc;

  // based on distance, calculate number of wheel revolutions
  float numRev = distance / wheelCirc;

  // based on number of revolutions, calculate target encoder count
  float targetCount = numRev * countsPerRev;

  // reset encoder counters and start pivoting
  //encoder.clearEnc(BOTH);
  encoderL.clearCount();
  encoderR.clearCount();

  if (pos_sensor == 0) {
    if (angle < 0) {
      setMotor(-power, power);
    } else {
      setMotor(power, -power);
    }
  }
  else if (pos_sensor == 1) {
    if (angle < 0) {
      setMotor(-power, power);
    } else {
      setMotor(power, -power);
    }
  }

  // keeps looping while right encoder count less than target count
  if (pos_sensor == 0) {

    if (angle < 0) {
      while (abs(rightCount) < abs(targetCount)) {
        // get current wheel encoder count
        rightCount = encoderR.getCount();
      }
    } else {
      while (abs(leftCount) < abs(targetCount)) {
        // get current wheel encoder count
        leftCount = encoderL.getCount();
      }
    }
  }
  else if (pos_sensor == 1) {

    if (angle < 0) {
      while (abs(rightCount) < abs(targetCount)) {
        // get current wheel encoder count
        rightCount = encoderL.getCount();
      }
    } else {
      while (abs(leftCount) < abs(targetCount)) {
        // get current wheel encoder count
        leftCount = encoderR.getCount();
      }
    }
  }
  // target count reached

  rem(lama);

  encoderL.clearCount();
  encoderR.clearCount();
}

void belokSudutPro(float angle, int xspeed, int lama){
  int offset1 = 0.7 * angle;
  int offset2 = 0.3 * angle;
  belokSudut(offset1,xspeed, 0);
  belokSudut(offset2,xspeed/2, lama);
}

void displayEncoder() {

  int djarakL = encoderL.getCount() * PI * 4.5 / 422;
  display.setCursor(10, 55);
  display.print(String((int32_t)djarakL));

  int djarakR = encoderR.getCount() * PI * 4.5 / 422;
  display.setCursor(100, 55);
  display.print(String((int32_t)djarakR));
  display.display();
}

void ambilAngkat(){
  naik(6,1,300);
  tutup(170,1,500);
  turun(120,1,300);
}


void ambilTurun(){
  naik(6,1,300);
  tutup(170,1,500);
//  turun(120,4,300);
}

void taruhAngkat(){
  naik(6,1,300);
  buka(80,1,500);
  turun(120,1,300);
}

void taruhTurun(){
  naik(6,1,300);
  buka(80,1,500);
//  turun(120,4,300);
}
  
void naik(int value, int speedx, int mili) {
  if (!stateNaik) {
    //180 -0
    for (int pos = servoAMIN; pos >= value; pos -= speedx) {
      servoA.write(pos);
      delay(10);
    }
    servoAMAX = constrain(value, 0, 180);
    stateNaik = true;
  }
  delay(mili);

}
void turun(int value, int speedx, int mili) {
  if (stateNaik) {
    //80 - 180
    for (int pos = servoAMAX; pos <= value; pos += speedx) {
      servoA.write(pos);
      delay(10);
    }
    servoAMIN = constrain(value, 0, 180);
    stateNaik = false;
  }
  delay(mili);
}
void tutup(int value, int speedx, int mili) {
  if (!stateTutup) {
    for (int pos = servoCMIN; pos <= value; pos += speedx) {
      servoC.write(pos);
      delay(10);
    }
    servoCMAX = constrain(value, 0, 180);
    stateTutup = true;
  }
  delay(mili);
}
void buka(int value, int speedx, int mili) {
  if (stateTutup) {
    for (int pos = servoCMAX; pos >= value; pos -= speedx) {
      servoC.write(pos);
      delay(10);
    }
    servoCMIN = constrain(value, 0, 180);
    stateTutup = false;
  }
  delay(mili);
}

void pilihSensor(bool pilih_sensor) {
  if (pilih_sensor == front) {
    pos_sensor = 0;
  }
  else if (pilih_sensor == rear) {
    pos_sensor = 1;
  }
}

//---------------------fungsi counter waktu--------------//
void counter_waktu(int interval) {
  unsigned long currentMillis = millis();
  if ((unsigned long)(currentMillis - previousMillis) >= interval) {
    count += 1;
    previousMillis = millis();
  }
}
//-------------------------------------------------------//

//-----------------------follow line timer----------------//
void followTimer(unsigned int countGoal, int speedx,  unsigned int lama_henti) {
  count = 0;
  pilihSpeed(speedx);

  while (count < countGoal) {
    counter_waktu(50);
    followLine();
  }
  //digitalWrite(buzz, HIGH);
  if (lama_henti > 0) {
    rem(lama_henti);
  }
  //digitalWrite(buzz, LOW);
  count = 0;
}
//-------------------------------------------------------//


//-----------------------move timer----------------//
void moveTimer(unsigned int countGoal, int kiri, int kanan, unsigned int lama_henti) {
  count = 0;
  while (count < countGoal) {
    counter_waktu(50);
    setMotor(kiri, kanan);
  }

  digitalWrite(buzz, HIGH);
  if (lama_henti > 0) {
    rem(lama_henti);
  }
  digitalWrite(buzz, LOW);

  count = 0;
}
//-------------------------------------------------------//


//---------------------Percabangan--------------------------//
void percabangan(int speedx, byte mode, unsigned int lama_henti)
{
  byte x = 0;
  pilihSpeed(speedx);
  while (x < 1)
  {
    if (mode == f1) {
      if (pos_sensor == 0) {
        followLine();
        if (f_valSensor[0] && f_valSensor[7]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (f_valSensor[0] && f_valSensor[7]) {
          followLine();
        }
      }

      else if (pos_sensor == 1) {
        followLine();
        if (b_valSensor[0] && b_valSensor[7]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (b_valSensor[0] && b_valSensor[7]) {
          followLine();
        }
      }
    }
    else if (mode == f2) {
      if (pos_sensor == 0) {
        followLine();
        if (f_valSensor[1] && f_valSensor[6]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (f_valSensor[1] && f_valSensor[6]) {
          followLine();
        }
      }

      else if (pos_sensor == 1) {
        followLine();
        if (b_valSensor[1] && b_valSensor[6]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (b_valSensor[1] && b_valSensor[6]) {
          followLine();
        }
      }
    }
    else if (mode == f3) {
      if (pos_sensor == 0) {
        followLine();
        if (f_valSensor[0] && f_valSensor[1] && f_valSensor[6] && f_valSensor[7]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (f_valSensor[0] && f_valSensor[1] && f_valSensor[6] && f_valSensor[7]) {
          followLine();
        }
      }

      else if (pos_sensor == 1) {
        followLine();
        if (b_valSensor[0] && b_valSensor[1] && b_valSensor[6] && b_valSensor[7]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (b_valSensor[0] && b_valSensor[1] && b_valSensor[6] && b_valSensor[7]) {
          followLine();
        }
      }
    }
    else if (mode == l1) {
      if (pos_sensor == 0) {
        followLine();
        if (f_valSensor[0]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (f_valSensor[0]) {
          followLine();
        }
      }

      else if (pos_sensor == 1) {
        followLine();
        if (b_valSensor[0]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (b_valSensor[0]) {
          followLine();
        }
      }
    }
    else if (mode == l2) {
      if (pos_sensor == 0) {
        followLine();
        if (f_valSensor[0] && f_valSensor[1]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (f_valSensor[0] && f_valSensor[1]) {
          followLine();
        }
      }

      else if (pos_sensor == 1) {
        followLine();
        if (b_valSensor[0] && b_valSensor[1]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (b_valSensor[0] && b_valSensor[1]) {
          followLine();
        }
      }
    }
    else if (mode == l3) {
      if (pos_sensor == 0) {
        followLine();
        if (f_valSensor[0] && f_valSensor[1] && f_valSensor[2]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (f_valSensor[0] && f_valSensor[1] && f_valSensor[2]) {
          followLine();
        }
      }

      else if (pos_sensor == 1) {
        followLine();
        if (b_valSensor[0] && b_valSensor[1] && b_valSensor[2]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (b_valSensor[0] && b_valSensor[1] && b_valSensor[2]) {
          followLine();
        }
      }
    }

    else if (mode == l4) {
      if (pos_sensor == 0) {
        followLine();
        if (f_valSensor[0] || f_valSensor[1]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (f_valSensor[0] || f_valSensor[1]) {
          followLine();
        }
      }

      else if (pos_sensor == 1) {
        followLine();
        if (b_valSensor[0] || b_valSensor[1]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (b_valSensor[0] || b_valSensor[1]) {
          followLine();
        }
      }
    }
    else if (mode == r1) {
      if (pos_sensor == 0) {
        followLine();
        if (f_valSensor[7]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (f_valSensor[7]) {
          followLine();
        }
      }

      else if (pos_sensor == 1) {
        followLine();
        if (b_valSensor[7]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (b_valSensor[7]) {
          followLine();
        }
      }
    }
    else if (mode == r2) {
      if (pos_sensor == 0) {
        followLine();
        if (f_valSensor[6] && f_valSensor[7]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (f_valSensor[6] && f_valSensor[7]) {
          followLine();
        }
      }

      else if (pos_sensor == 1) {
        followLine();
        if (b_valSensor[6] && b_valSensor[7]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (b_valSensor[6] && b_valSensor[7]) {
          followLine();
        }
      }
    }
    else if (mode == r3) {
      if (pos_sensor == 0) {
        followLine();
        if (f_valSensor[5] && f_valSensor[6] && f_valSensor[7]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (f_valSensor[5] && f_valSensor[6] && f_valSensor[7]) {
          followLine();
        }
      }

      else if (pos_sensor == 1) {
        followLine();
        if (b_valSensor[5] && b_valSensor[6] && b_valSensor[7]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (b_valSensor[5] && b_valSensor[6] && b_valSensor[7]) {
          followLine();
        }
      }
    }

    else if (mode == r4) {
      if (pos_sensor == 0) {
        followLine();
        if (f_valSensor[6] || f_valSensor[7]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (f_valSensor[6] || f_valSensor[7]) {
          followLine();
        }
      }

      else if (pos_sensor == 1) {
        followLine();
        if (b_valSensor[6] || b_valSensor[7]) {
          digitalWrite(buzz, HIGH); x++;
        }
        while (b_valSensor[6] || b_valSensor[7]) {
          followLine();
        }
      }
    }

    digitalWrite(buzz, LOW);
  }

  if (lama_henti > 0) {
    rem(lama_henti);
  }
}
//--------------------------------------------------------------------//

//------------------------------UBAH WARNA GARIS HITAM-----------------------//
void gantihitam() {
  byte x = 0;
  while (x < 1) {
    followLine();
    if (pos_sensor == 0) {
      followLine();
      if (f_valSensor[1] && f_valSensor[6]) {
        hitam = 1; putih = 0; x++;
      }
    }

    else if (pos_sensor == 1) {
      followLine();
      if (b_valSensor[1] && b_valSensor[6]) {
        hitam = 1; putih = 0; x++;
      }
    }
  }
  followJarak(5, 100, 0);
}

//------------------------------UBAH WARNA GARIS PUTIH-----------------------//
void gantiputih() {
  byte x = 0;
  while (x < 1) {
    followLine();
    if (pos_sensor == 0) {
      followLine();
      if (f_valSensor[1] && f_valSensor[6]) {
        hitam = 0; putih = 1; x++;
      }
    }

    else if (pos_sensor == 1) {
      followLine();
      if (b_valSensor[1] && b_valSensor[6]) {
        hitam = 0; putih = 1; x++;
      }
    }
  }
  followJarak(5, 100, 0);
}

//--------------------------belok kanan--------------------------//
void kanan(int kec, unsigned int lama_henti) {
  int sensor = readSensor(pos_sensor);
  if (pos_sensor == 0) {
    moveTimer(3, kec, -(kec * 0.6), 0);
    while (f_valSensor[3] || f_valSensor[4]) {
      setMotor(kec, -(kec * 0.6));
      sensor = readSensor(pos_sensor);
    }
    while (!f_valSensor[3] && !f_valSensor[4]) {
      setMotor(kec, -(kec * 0.6));
      sensor = readSensor(pos_sensor);
    }
  }
  else if (pos_sensor == 1) {
    moveTimer(3, (kec * 0.6), -kec, 0);
    while (b_valSensor[3] || b_valSensor[4]) {
      setMotor((kec * 0.6), -kec);
      sensor = readSensor(pos_sensor);
    }
    while (!b_valSensor[3] && !b_valSensor[4]) {
      setMotor((kec * 0.6), -kec);
      sensor = readSensor(pos_sensor);
    }
  }
  if (lama_henti > 0) {
    rem(lama_henti);
  }
}
//-----------------------------------------------------------//

//--------------------------belok kiri--------------------------//
void kiri(int kec, unsigned int lama_henti) {
  int sensor = readSensor(pos_sensor);
  if (pos_sensor == 0) {
    moveTimer(3, -(kec * 0.6), kec, 0);
    while (f_valSensor[3] || f_valSensor[4]) {
      setMotor(-(kec * 0.6), kec);
      sensor = readSensor(pos_sensor);
    }
    while (!f_valSensor[3] && !f_valSensor[4]) {
      setMotor(-(kec * 0.6), kec);
      sensor = readSensor(pos_sensor);
    }
  }
  else if (pos_sensor == 1) {
    moveTimer(3, -kec, (kec * 0.6), 0);
    while (b_valSensor[3] || b_valSensor[4]) {
      setMotor(-kec, (kec * 0.6));
      sensor = readSensor(pos_sensor);
    }
    while (!b_valSensor[3] && !b_valSensor[4]) {
      setMotor(-kec, (kec * 0.6));
      sensor = readSensor(pos_sensor);
    }
  }
  if (lama_henti > 0) {
    rem(lama_henti);
  }
}
//-----------------------------------------------------------//

//--------------------------belok kiri timer--------------------------//
void kiriTimer(unsigned int countbelok, int kec, unsigned int lama_henti) {
  moveTimer(countbelok, -kec, kec, lama_henti);

  if (lama_henti > 0) {
    rem(lama_henti);
  }
}
//-----------------------------------------------------------//

void pilihSpeed(int kecepatan) {

  kp = kecepatan * 0.2;
  kd = kecepatan * 0.60;
  SPEED = kecepatan;
  MIN_SPEED = -(kecepatan * 0.60);
  MAX_SPEED = kecepatan;

}

//--------------------FUNGSI SCAN GARIS-----------------//
void followLine() {
  int sensor = readSensor(pos_sensor);

  switch (sensor) {
    case 0b00000001: error = -7;  break;
    case 0b00000011: error = -6;  break;
    case 0b00000010: error = -5;  break;
    case 0b00000110: error = -4;  break;
    case 0b00000100: error = -3;  break;
    case 0b00001100: error = -2;  break;
    case 0b00001000: error = -1;  break;
    case 0b00011000: error = 0;  break; // lurus
    case 0b00010000: error = 1;  break;
    case 0b00110000: error = 2;  break;
    case 0b00100000: error = 3;  break;
    case 0b01100000: error = 4;  break;
    case 0b01000000: error = 5;  break;
    case 0b11000000: error = 6;  break;
    case 0b10000000: error = 7;  break;
  }

  int rateError = error - lastError;
  lastError = error;

  int moveVal = (int)(error * kp) + (rateError * kd);

  int moveLeft = SPEED - moveVal;
  int moveRight = SPEED + moveVal;


  moveLeft = constrain(moveLeft, MIN_SPEED, MAX_SPEED);
  moveRight = constrain(moveRight, MIN_SPEED, MAX_SPEED);

  if (pos_sensor == 0) {
    setMotor(moveLeft, moveRight);
  }
  else if (pos_sensor == 1) {
    setMotor(-moveRight, -moveLeft);
  }
}

//-------------------kalibrasi sensor
void kalibrasiSensor() {
  int i, f_nilaiSensor, b_nilaiSensor;
  int f_Sensor[8], f_besar[8], f_kecil[8];
  int b_Sensor[8], b_besar[8], b_kecil[8];

  for (i = 0; i < 8; i++) {
    f_besar[i] = 0;
    f_kecil[i] = 4095;
    b_besar[i] = 0;
    b_kecil[i] = 4095;
  }

  display.clearDisplay();
  display.display();
  delay(50);

  while (BTN_START) {
    display.setCursor(20, 30);
    display.print("Proses Kalibrasi");
    display.display();
    adcSensor();
    for (i = 0; i < 8; i++) {
      f_nilaiSensor = f_sensor[i];
      b_nilaiSensor = b_sensor[i];

      if (f_nilaiSensor < f_kecil[i]) {
        f_kecil[i] = f_nilaiSensor;
      }

      if (f_nilaiSensor > f_besar[i]) {
        f_besar[i] = f_nilaiSensor;
      }

      if (b_nilaiSensor < b_kecil[i]) {
        b_kecil[i] = b_nilaiSensor;
      }

      if (b_nilaiSensor > b_besar[i]) {
        b_besar[i] = b_nilaiSensor;
      }
    }
  }

  delay(10);

  //Perhitungan nilai pembanding
  for (i = 0; i < 8; i++) {
    EE.f_SENSOR_REFF[i] = (f_besar[i] - f_kecil[i]) / 2;
    EE.b_SENSOR_REFF[i] = (b_besar[i] - b_kecil[i]) / 2;
  }

  display.clearDisplay();
  display.display();

  display.setCursor(30, 30);
  display.print("Simpan...");
  display.display();
  EEPROM.put(0, EE);
  EEPROM.commit();
  delay(500);
  display.clearDisplay();
  display.display();
}


//-------------sensor garis multiplekser
void f_selector(bool a, bool b, bool c) {
  digitalWrite(f_sel1, a);
  digitalWrite(f_sel2, b);
  digitalWrite(f_sel3, c);
}

void b_selector(bool a, bool b, bool c) {
  digitalWrite(b_sel1, a);
  digitalWrite(b_sel2, b);
  digitalWrite(b_sel3, c);
}

void adcSensor() {
  //depan
  f_selector(0, 1, 0);
  f_sensor[0] = analogRead(f_pin_sensor);
  f_selector(0, 0, 1);
  f_sensor[1] = analogRead(f_pin_sensor);
  f_selector(0, 0, 0);
  f_sensor[2] = analogRead(f_pin_sensor);
  f_selector(0, 1, 1);
  f_sensor[3] = analogRead(f_pin_sensor);
  f_selector(1, 0, 0);
  f_sensor[4] = analogRead(f_pin_sensor);
  f_selector(1, 1, 0);
  f_sensor[5] = analogRead(f_pin_sensor);
  f_selector(1, 1, 1);
  f_sensor[6] = analogRead(f_pin_sensor);
  f_selector(1, 0, 1);
  f_sensor[7] = analogRead(f_pin_sensor);

  //belakang
  b_selector(0, 1, 0);
  b_sensor[0] = analogRead(b_pin_sensor);
  b_selector(0, 0, 1);
  b_sensor[1] = analogRead(b_pin_sensor);
  b_selector(0, 0, 0);
  b_sensor[2] = analogRead(b_pin_sensor);
  b_selector(0, 1, 1);
  b_sensor[3] = analogRead(b_pin_sensor);
  b_selector(1, 0, 0);
  b_sensor[4] = analogRead(b_pin_sensor);
  b_selector(1, 1, 0);
  b_sensor[5] = analogRead(b_pin_sensor);
  b_selector(1, 1, 1);
  b_sensor[6] = analogRead(b_pin_sensor);
  b_selector(1, 0, 1);
  b_sensor[7] = analogRead(b_pin_sensor);

}

int readSensor(bool posisi) {
  int i;
  adcSensor();
  for (i = 0; i < 8; i++) {
    if (f_sensor[i] > (EE.f_SENSOR_REFF[i] + (sensiSensor))) {
      f_valSensor[i] = hitam;
    } else {
      f_valSensor[i] = putih;
    }

    if (b_sensor[i] > (EE.b_SENSOR_REFF[i] + (sensiSensor))) {
      b_valSensor[i] = hitam;
    } else {
      b_valSensor[i] = putih;
    }
  }

  if (posisi == 0) {
    bitSensor = ((f_valSensor[7] * 1) + (f_valSensor[6] * 2) + (f_valSensor[5] * 4) + (f_valSensor[4] * 8)
                 + (f_valSensor[3] * 16) + (f_valSensor[2] * 32) + (f_valSensor[1] * 64) + (f_valSensor[0] * 128));
  }
  else if (posisi == 1) {
    bitSensor = ((b_valSensor[7] * 1) + (b_valSensor[6] * 2) + (b_valSensor[5] * 4) + (b_valSensor[4] * 8)
                 + (b_valSensor[3] * 16) + (b_valSensor[2] * 32) + (b_valSensor[1] * 64) + (b_valSensor[0] * 128));
  }

  return bitSensor;
}

void displaySensor() {
  byte i;
  int f_Sensor = readSensor(0);
  display.setCursor(40, 25);
  for (i = 0; i < 8; i++) {
    if (f_Sensor & (0b10000000 >> i)) {
      display.write('1');
    } else {
      display.write('_');
    }
  }

  int b_Sensor = readSensor(1);
  display.setCursor(40, 50);
  for (i = 0; i < 8; i++) {
    if (b_Sensor & (0b00000001 << i)) {
      display.write('1');
    } else {
      display.write('_');
    }
  }
  // display.setCursor(90, 10);
  // display.print(strgy[selectStrategy]);
  display.display();
}

//------------------FUNGSI MOTOR Penggerak---------------------//
void setMotor(int speedLeft, int speedRight)
{
  speedLeft = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);

  if (speedLeft > 0) {
    int speedL = map(speedLeft, 0, 255, 255, 0);
    ledcWrite(M1A_PWM_CHANNEL, speedL);
    ledcWrite(M1B_PWM_CHANNEL, 255);
  }
  else {
    int speedL = map(speedLeft, -255, 0, 0, 255);
    ledcWrite(M1A_PWM_CHANNEL, 255);
    ledcWrite(M1B_PWM_CHANNEL, speedL);
  }

  if (speedRight > 0) {
    int speedR = map(speedRight, 0, 255, 255, 0);
    ledcWrite(M2A_PWM_CHANNEL, speedR);
    ledcWrite(M2B_PWM_CHANNEL, 255);
  }
  else {
    int speedR = map(speedRight, -255, 0, 0, 255);
    ledcWrite(M2A_PWM_CHANNEL, 255);
    ledcWrite(M2B_PWM_CHANNEL, speedR);
  }
}
void rem(unsigned int lama) {
  ledcWrite(M2A_PWM_CHANNEL, 0);
  ledcWrite(M2B_PWM_CHANNEL, 0);
  ledcWrite(M1A_PWM_CHANNEL, 0);
  ledcWrite(M1B_PWM_CHANNEL, 0);
  delay(lama);
}

void beep(int nyala){
  if(nyala == 1){
    digitalWrite(buzz,HIGH);
  }else;
    digitalWrite(buzz,LOW);
}

#include "strategi.h"

void strategi0() {
  jalan_strategi();
}
void sensitivitas(){
  sensitivity();
}
