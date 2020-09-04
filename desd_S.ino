
#include <Wire.h> 
// accelerometer parameters
long acc_x; long acc_y; long acc_z; // place holders for raw register data
float g_acc_x; float g_acc_y; float g_acc_z;  // place holders for parameters in g earth acceleration
float g_acc_norm = 16384.0;  // normalizer for raw data, for transfering into "g"s, as datasheet says for maximum lsb sesitivity
float angle_acc_x; float angle_acc_y; // place holders for angles derived from only acclerometer 

// gyrosope data 
long gyro_x; long gyro_y; long gyro_z; // place holders for raw register data 
float a_gyro_x; float a_gyro_y; float a_gyro_z; // place holders for  angle per sec for gyro
float a_gyro_norm = 131.0; // normalizer for raw data, for tranfering into "deg/s"s, as datasheet says for maximum lsb sesitivity
float angle_gyro_x; float angle_gyro_y; // place holders for angles derived from only gyro

// time variables
float elapse; float curr; float prev; // place holders for gyro angle calculation helpers



// real angles on which blancing algorithm is based on
float angle_x; float angle_y;

float angle_x_init; float angle_y_init;

// convertion from raw accelerometer data to "g"s
void acc_raw_converter(){
  g_acc_x = acc_x / g_acc_norm;
  g_acc_y = acc_y / g_acc_norm;
  g_acc_z = acc_z / g_acc_norm;
 // Serial.print(g_acc_x);
 // Serial.print("   -           -   ");
  
 // Serial.print(g_acc_y);

 // Serial.print("   -           -   ");
  
//  Serial.println(g_acc_z); 
}

// convertion from raw gyro data to "deg/s"
void gyro_raw_converter(){
  a_gyro_x = gyro_x / a_gyro_norm;
  a_gyro_y = gyro_y / a_gyro_norm;
  a_gyro_z = gyro_z / a_gyro_norm;
}

// this code was copied after realizing that i had to write the samme code for getting to the registes needed to do stuff
void mpu_get_ready() {
  Wire.beginTransmission(0b1101000); //as master wants to talk to mpu one has to begin transmition to the address of mpu
  Wire.write(0x6B); //now comes the phase of selecting the register needed to imitialize mpu 
  Wire.write(0b00000000); // as mpu is sleeping one has to wake it up so as datasheet says one has to write zero in abouve selected register
  Wire.endTransmission();  //as humans say goodby to each other protocol demands saying goodby from the master, ending the process
  Wire.beginTransmission(0b1101000); //again one mpu adress is needed to set up sensors
  Wire.write(0x1B); //one has to set up the gyro, so the proper register is accessed
  Wire.write(0x00000000); //we are selecting mosr sensitive mode of the gyro, thats what we intend to +/- 250 deg/s
  Wire.endTransmission(); // agian formalities
  Wire.beginTransmission(0b1101000); //visiting mpu for another sensor set up
  Wire.write(0x1C); //as one wishes to set up and configure accelerometer, its register is accessed
  Wire.write(0b00000000); //we are selecting most sensitive mode of accelerometer, thats what we intend to do +/- 2g
  Wire.endTransmission();// we like being polite
}

// deals with the mpu accelerometer
void deal_with_acc() {
  Wire.beginTransmission(0b1101000); //accessing mpu adress protocol defined
  Wire.write(0x3B); //accelerometer adress
  Wire.endTransmission(); // polite
  Wire.requestFrom(0b1101000, 6); //specified in datasheet registers to get the raw data
  while (Wire.available() < 6); // get the bytes needed
  acc_x = Wire.read() << 8 | Wire.read(); //swipe two bytes of x dir
  acc_y = Wire.read() << 8 | Wire.read(); //swipe two bytes of y dir
  acc_z = Wire.read() << 8 | Wire.read(); //swipe 16 bits fo z dir
  acc_raw_converter();
  tilt_angles_accel();
}
// deals with the mpu gyroscope
void deal_with_gyro() {
  Wire.beginTransmission(0b1101000); // addressing mpu adress d;
  Wire.write(0x43);   // adressing gyro adress d;
  Wire.endTransmission(); // polite
  Wire.requestFrom(0b1101000, 6); // one has to request regiteres proper specified in datasheet
  while (Wire.available() < 6); // get raw data
  gyro_x = Wire.read() << 8 | Wire.read(); //swipe 2 bytes of x dir
  gyro_y = Wire.read() << 8 | Wire.read(); //swipte 2 bytes of y dir
  gyro_z = Wire.read() << 8 | Wire.read(); //swipe 16 bits of z dir
  gyro_raw_converter();
  tilt_angles_gyro();
}

// angle calculation using only accelerometer
void tilt_angles_accel(){
  
  angle_acc_x = (atan(g_acc_y / sqrt(g_acc_x*g_acc_x + g_acc_z*g_acc_z)) * 180 / PI) + 1.83;
  angle_acc_y = (atan(-1 * g_acc_x / sqrt(g_acc_y*g_acc_y + g_acc_z*g_acc_z)) * 180 / PI) - 1.97;
 
}

// angle calculation using only gyroscope
void tilt_angles_gyro(){
  time_management();
  a_gyro_x = a_gyro_x + 2.89;
  a_gyro_y = a_gyro_y + 0.36;
  angle_gyro_x = angle_gyro_x + a_gyro_x * elapse;
  angle_gyro_y = angle_gyro_y + a_gyro_y * elapse;
}

// time management for gyro angle calcuation
void time_management(){
  prev = curr;  
  curr = millis();  
  elapse = curr - prev; 
  elapse /= 1000; // from milisec to sec
}

// calculate combination of two angles  // as we found out gyro is highly reliable in our situation
void calc_real_angle(){ 
  angle_x = 0.96 * angle_gyro_x + 0.04 * angle_acc_x + angle_x_init   ; 
  angle_y = 0.96 * angle_gyro_y + 0.04 * angle_acc_y  + angle_y_init;
}

// motor 1 pins definition
int EnA  = 10;  int In1 =  12;  int In2 = 8;
// motor 2 pins definition
int In3 = 7; int In4 = 6; int EnB = 5;
// motor 3 pins definition
int EnA_  = 9; int In1_ =  4; int In2_ = 13;
// motor 4 pins definition
int In3_ = 11; int In4_ = 2;  int EnB_ = 3;

// sets output pins as outputs 
void set_up_pins(){
  pinMode(EnA, OUTPUT);  pinMode(In1, OUTPUT); pinMode(In2, OUTPUT); // bridge 1
  pinMode(In3, OUTPUT);  pinMode(In4, OUTPUT); pinMode(EnB, OUTPUT);

  pinMode(EnA_, OUTPUT); pinMode(In1_, OUTPUT); pinMode(In2_, OUTPUT); // bridge 2
  pinMode(In3_, OUTPUT); pinMode(In4_, OUTPUT); pinMode(EnB_, OUTPUT);

  pinMode(A1,INPUT);
  pinMode(A0,INPUT);
  pinMode(A2,INPUT);
}


// yup this is setting things up for work
void setup() {
  Serial.begin(9600); // being master is cool
  Wire.begin();
  mpu_get_ready(); // seting up mpu , thus slaves
  set_up_pins(); // pins pins
  angle_x_init = 0;
  angle_y_init = 0;
}

// kinda looping thing
void loop() {
  deal_with_acc();
  deal_with_gyro();
  calc_real_angle();
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
 

// balances in upwards direction
void balanceSelfUpward(){
  
  if (acc_z > 0.03) {
    motor1_go_up();
    motor3_go_up();
    delay(100);
    motor1_stop();
    motor3_stop();
  } 
  else 
  if (acc_z < - 0.03) {
    motor2_go_up();
    motor4_go_up();
    delay(100);
    motor2_stop();
    motor4_stop();
  }
  else if  (acc_y >  0.03) {
    motor3_go_up();
    motor4_go_up();
    delay(100);
    motor3_stop();
    motor4_stop();
  }
  else if (acc_y < -0.03) {
    motor1_go_up();
    motor2_go_up();
    delay(100);
    motor1_stop();
    motor2_stop();
  }
}

// balances platform with making legs length smaller
void balanceSelfDown(){
  if (acc_z < - 0.03) {
    motor1_go_down();
    motor3_go_down();
    delay(70);
    motor1_stop();
    motor3_stop();
  } 
  else if (acc_z >  0.03) {
    motor2_go_down();
    motor4_go_down();
    delay(70);
    motor2_stop();
    motor4_stop();
  }
  else  if (acc_y < - 0.03) {
    motor3_go_down();
    motor4_go_down();
    delay(70);
    motor3_stop();
    motor4_stop();
  }
  else if (acc_y >  0.03) {
    motor1_go_down();
    motor2_go_down();
    delay(70);
    motor1_stop();
    motor2_stop();
  }
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
