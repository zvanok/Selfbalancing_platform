/*
  ===Contact & Support===
  Website: http://eeenthusiast.com/
  Youtube: https://www.youtube.com/EEEnthusiast
  Facebook: https://www.facebook.com/EEEnthusiast/
  Patreon: https://www.patreon.com/EE_Enthusiast
  Revision: 1.0 (July 13th, 2016)
  ===Hardware===
  - Arduino Uno R3
  - MPU-6050 (Available from: http://eeenthusiast.com/product/6dof-mpu-6050-accelerometer-gyroscope-temperature/)
  ===Software===
  - Latest Software: https://github.com/VRomanov89/EEEnthusiast/tree/master/MPU-6050%20Implementation/MPU6050_Implementation
  - Arduino IDE v1.6.9
  - Arduino Wire library
  ===Terms of use===
  The software is provided by EEEnthusiast without warranty of any kind. In no event shall the authors or
  copyright holders be liable for any claim, damages or other liability, whether in an action of contract,
  tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in
  the software.
*/

#include <Wire.h>

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;


// motor 1 pins definition
int EnA  = 10;  int In1 =  12;  int In2 = 8;
// motor 2 pins definition
int In3 = 7; int In4 = 6; int EnB = 5;

// motot 3 pins definition
int EnA_  = 9; int In1_ =  4; int In2_ = 13;

// motor 4 pins definition
int In3_ = 11; int In4_ = 2;  int EnB_ = 3;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();

  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);

  pinMode(EnA_, OUTPUT);
  pinMode(EnB_, OUTPUT);
  pinMode(In1_, OUTPUT);
  pinMode(In2_, OUTPUT);
  pinMode(In3_, OUTPUT);
  pinMode(In4_, OUTPUT);

  pinMode(A1,INPUT);
  pinMode(A0,INPUT);
  pinMode(A2,INPUT);
}


void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  printData();
  if(digitalRead(A0)== HIGH){

  } else if(digitalRead(A1)== HIGH){
    down_all();
  }else if(digitalRead(A2)== HIGH){
    balanceSelfDown();
    Serial.print("balancing Down");
  }else{
    balanceSelfUpward();
  }
  delay(100);
}

void balanceSelfUpward(){
  if (gForceZ > 0.03) {
    motor1_go_up();
    motor3_go_up();
    delay(100);
    motor1_stop();
    motor3_stop();
  } 
  else if (gForceZ < - 0.03) {
    motor2_go_up();
    motor4_go_up();
    delay(100);
    motor2_stop();
    motor4_stop();
  }
  else  if (gForceY >  0.03) {
    motor3_go_up();
    motor4_go_up();
    delay(100);
    motor3_stop();
    motor4_stop();
  }
  else if (gForceY < -0.03) {
    motor1_go_up();
    motor2_go_up();
    delay(100);
    motor1_stop();
    motor2_stop();
  }
}

// balances platform with making legs length smaller
void balanceSelfDown(){
  if (gForceZ < - 0.03) {
    motor1_go_down();
    motor3_go_down();
    delay(50);
    motor1_stop();
    motor3_stop();
  } 
  else if (gForceZ >  0.03) {
    motor2_go_down();
    motor4_go_down();
    delay(50);
    motor2_stop();
    motor4_stop();
  }
  else  if (gForceY < - 0.03) {
    motor3_go_down();
    motor4_go_down();
    delay(50);
    motor3_stop();
    motor4_stop();
  }
  else if (gForceY >  0.03) {
    motor1_go_down();
    motor2_go_down();
    delay(50);
    motor1_stop();
    motor2_stop();
  }
}


void setupMPU() {
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Accel Registers (3B - 40)
  while (Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData() {
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}



void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
}







// motor 1
void motor1_stop() {
  digitalWrite(In1, LOW);     digitalWrite(In2, LOW);
}
void motor1_go_down() {
  digitalWrite(In1, LOW);    digitalWrite(In2, HIGH);   analogWrite(EnA, 200);
}
void motor1_go_up() {
  digitalWrite(In1, HIGH);   digitalWrite(In2, LOW);   analogWrite(EnA, 200);
}

//motor 2
void motor2_stop() {
  digitalWrite(In3, LOW);  digitalWrite(In4, LOW);
}
void motor2_go_down() {
  digitalWrite(In3, LOW);  digitalWrite(In4, HIGH);  analogWrite(EnB, 200);
}
void motor2_go_up() {
  digitalWrite(In3, HIGH); digitalWrite(In4, LOW);   analogWrite(EnB, 200);
}

// motor 3
void motor3_stop() {
  digitalWrite(In1_, LOW);     digitalWrite(In2_, LOW);
}
void motor3_go_down() {
  digitalWrite(In1_, LOW);    digitalWrite(In2_, HIGH);   analogWrite(EnA_, 200);
}
void motor3_go_up() {
  digitalWrite(In1_, HIGH);   digitalWrite(In2_, LOW);   analogWrite(EnA_, 200);
}

// motor 4
void motor4_stop() {
  digitalWrite(In3_, LOW);  digitalWrite(In4_, LOW);
}
void motor4_go_down() {
  digitalWrite(In3_, HIGH);  digitalWrite(In4_, LOW);  analogWrite(EnB_, 200);
}
void motor4_go_up() {
  digitalWrite(In3_, LOW); digitalWrite(In4_, HIGH);   analogWrite(EnB_, 200);
}



void up_pair1()   //run both motors in the same direction
{
  motor1_go_up();
  motor2_go_up();
  delay(2000);
  motor1_stop();
  motor2_stop();
}

void down_pair1() {
  motor1_go_down();
  motor2_go_down();
  delay(2000);
  motor1_stop();
  motor2_stop();
}

void up_pair2()   //run both motors in the same direction
{
  motor3_go_up();
  motor4_go_up();
  delay(2000);
  motor3_stop();
  motor4_stop();
}

void down_pair2() {
  motor3_go_down();
  motor4_go_down();
  delay(2000);
  motor3_stop();
  motor4_stop();
}

void up_all() {
  motor1_go_up();
  motor2_go_up();
  motor3_go_up();
  motor4_go_up();
  delay(2000);
  motor1_stop();
  motor2_stop();
  motor3_stop();
  motor4_stop();
}


void down_all() {
  motor1_go_down();
  motor2_go_down();
  motor3_go_down();
  motor4_go_down();
  delay(1000);
  motor1_stop();
  motor2_stop();
  motor3_stop();
  motor4_stop();
}
