#include <Wire.h> //The I2C library
#include <Servo.h>

//Position Error from CenterVal
int Error = 0; //Error 

//IMU------------------------------------------------------------
int CenterVal = 90; //maintained value for Servo


#define BMA180              0x40
#define ee_w_MASK           0x10
#define mode_config_MASK    0x03
#define bw_MASK             0xF0
#define range_MASK          0x0E
#define lat_int_MASK        0x01
#define lat_int             0x01

Servo pitch;
Servo roll;

int gyroResult[3], AccelX, AccelY, AccelZ, temp;
float timeStep = 0.02;          //20ms. Need a time step value for integration of gyro angle from angle/sec
float biasGyroX, biasGyroY, biasGyroZ, biasAccelX, biasAccelY, biasAccelZ;
float pitchGyro = 0;
float pitchAccel = 0;
float pitchPrediction = 0; //Output of Kalman filter
float rollGyro = 0;
float rollAccel = 0;
float rollPrediction = 0;  //Output of Kalman filter
float yawGyro = 0;
float giroVar = 0.1; //0.1
float deltaGiroVar = 0.1; //0.1
float accelVar = 10; //5
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; //0.1 angle and angle change rate covariance
float kx, kv;

float pitchServo;
float rollServo; 
float rollmap;

unsigned long timer;

  
//writeTo Function
void writeTo(byte device, byte toAddress, byte val) {
  Wire.beginTransmission(device);  
  Wire.write(toAddress);        
  Wire.write(val);        
  Wire.endTransmission();
}

//readFrom function
void readFrom(byte device, byte fromAddress, int num, byte result[]) {
  Wire.beginTransmission(device);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.read();
    i++;
  }
}

//Read Gyro
void getGyroscopeReadings(int gyroResult[]) {
  byte buffer[6];
  readFrom(0x68,0x1D,6,buffer);
  gyroResult[0] = (((int)buffer[0]) << 8 ) | buffer[1]; //Combine two bytes into one int
  gyroResult[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  gyroResult[2] = (((int)buffer[4]) << 8 ) | buffer[5];
} 

//Accelerometer Initialization
byte initializeBMA180()
{
  /*Set EEPROM image to write mode so we can change configuration*/
  delay(20);
  Wire.beginTransmission(BMA180);
  Wire.write(0x0D);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte ee_w = Wire.read();
  ee_w |= ee_w_MASK;
  Wire.beginTransmission(BMA180);
  Wire.write(0x0D);
  Wire.write(ee_w);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set mode configuration register to Mode 00*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x30);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte mode_config = Wire.read();
  mode_config &= ~(mode_config_MASK);
  Wire.beginTransmission(BMA180);
  Wire.write(0x30);
  Wire.write(mode_config);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set bandwidth to 10Hz*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x20);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte bw = Wire.read();
  bw &= ~(bw_MASK);
  bw |= 0x00 << 4;
  Wire.beginTransmission(BMA180);
  Wire.write(0x20);
  Wire.write(bw);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set acceleration range to 2g*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x35);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte range = Wire.read();
  range &= ~(range_MASK);
  range |= 0x00 << 1 ;
  /*    case B000: // 1g
        case B001: // 1.5g
        case B010/0x02: // 2g
        case B011: // 3g
        case B100: // 4g
        case B101: // 8g
        case B110: // 16g
        */
  
  Wire.beginTransmission(BMA180);
  Wire.write(0x35);
  Wire.write(range);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set interrupt latch state to non latching*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte latch_int = Wire.read();
  latch_int &= ~(0x01);
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  Wire.write(latch_int);
  if(Wire.endTransmission()){return(1);}
  delay(20); 
  /*Set interrupt type to new data*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte int_type = Wire.read();
  int_type |= 0x02;
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  Wire.write(int_type);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  return(0);
}

void servoPin() {
  pitch.attach(A0);
  roll.attach(A1);
}


void gyroInit() {  //gyro init
  writeTo(0x68,0x16,0x1A); //Set gyro to +/-2000deg/sec and 98Hz low pass filter
  writeTo(0x68,0x15,0x09); //Set gyro to 100Hz sample rate
}

void accelerometerGyroBias() {    //accelerometer and gyro bias to calibrate
  
  int totalGyroXValues = 0;
  int totalGyroYValues = 0;
  int totalGyroZValues = 0;
  int totalAccelXValues = 0;
  int totalAccelYValues = 0;
  int totalAccelZValues = 0;
  int i;
  
  // Determine zero bias for all axes of both sensors by averaging 50 measurements
  for (i = 0; i < 50; i += 1) {
    getGyroscopeReadings(gyroResult);
    readAccel();
    totalGyroXValues += gyroResult[0];
    totalGyroYValues += gyroResult[1];
    totalGyroZValues += gyroResult[2];
    totalAccelXValues += AccelX;
    totalAccelYValues += AccelY;
    totalAccelZValues += AccelZ;
    delay(50);
  }
  biasGyroX = totalGyroXValues / 50;
  biasGyroY = totalGyroYValues / 50;
  biasGyroZ = totalGyroZValues / 50;
  biasAccelX = totalAccelXValues / 50;
  biasAccelY = totalAccelYValues / 50;
  biasAccelZ = (totalAccelZValues / 50) - 256; //Don't compensate gravity away! We would all (float)!
}


void setup() {

  servoPin();
  
  Wire.begin();            //Open I2C communications as master
  Serial.begin(115200);    //Open serial communications to the PC to see what's happening
  
  gyroInit();
  initializeBMA180();

  delay(100); //wait for gyro to "spin" up
  
  accelerometerGyroBias();

}

void loop() {
   
  timer = millis(); //get a start value to determine the time the loop takes
  getGyroscopeReadings(gyroResult);
  readAccel();

  calculation();
  kalmanFilter();
  ServoMove();
  printData();

   //delay(1);

  timer = millis() - timer;          //how long did the loop take?
  timer = (timeStep * 1000) - timer; //how much time to add to the loop to make it last time step msec
  delay(timer);                                    //make one loop last time step msec
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
}



//*********************************************Function on the Loop Section*********************************************************************************//
byte readAccel()
{
  Wire.beginTransmission(BMA180);
  Wire.write(0x02);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,7) != 7){return(2);}
  AccelX = Wire.read();
  AccelX |= Wire.read() << 8;
  AccelX >>= 2;

  
  AccelY = Wire.read();
  AccelY |= Wire.read() << 8;
  AccelY >>= 2;

  
  AccelZ = Wire.read();
  AccelZ |= Wire.read() << 8;
  AccelZ >>= 2;
  temp = Wire.read();
  
        if (AccelZ < 1500){
    AccelZ = 1500;}
}


void calculation() {
  pitchAccel = atan2((AccelY - biasAccelY) / 8192, (AccelZ - biasAccelZ) / 8192) * 360.0 / (2*PI);
  pitchGyro = pitchGyro + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;
  pitchPrediction = pitchPrediction + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;
  
  rollAccel = atan2((AccelX - biasAccelX) / 8192, (AccelZ - biasAccelZ) / 8192) * 360.0 / (2*PI);
  rollGyro = rollGyro - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep; 
  rollPrediction = rollPrediction - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep;
  
  yawGyro = yawGyro - ((gyroResult[2] - biasGyroZ) / 14.375) * timeStep;
}

void kalmanFilter() {
  //-------------Filter Time-----------------KALMAN Filter//
  Pxx += timeStep * (2 * Pxv + timeStep * Pvv);
  Pxv += timeStep * Pvv;
  Pxx += timeStep * giroVar;
  Pvv += timeStep * deltaGiroVar;
  kx = Pxx * (1 / (Pxx + accelVar));
  kv = Pxv * (1 / (Pxx + accelVar));
  
  pitchPrediction += (pitchAccel - pitchPrediction) * kx;
  rollPrediction += (rollAccel - rollPrediction) * kx;
  
  Pxx *= (1 - kx);
  Pxv *= (1 - kx);
  Pvv -= kv * Pxv;
}

void ServoMove() {
   
  rollmap = rollPrediction;  
  //pitchServo = constrain(pitchServo, -90, 90);
  rollmap = map(rollmap, -90, 90, 0, 180);
  
      //Close Loop-------------------------------------------------------------
  Error = CenterVal - rollmap; //finds error between the wanted value and what it currently is.
  rollServo += Error; //“if you aren’t quite getting to target, keep increasing over a period of time until you get there” 
    
  roll.writeMicroseconds(rollServo);  
}

void printData() {
  //Serial.print(AccelX);
  //Serial.print("X \t");
  //Serial.print(AccelY);
  //Serial.print("Y \t");
  //Serial.print(AccelZ);
  //Serial.print("Z \t");
  Serial.print(rollmap);
  Serial.print("roll \t"); 
  Serial.print(Error);
  Serial.print("out \t");

  Serial.print(rollServo);
  Serial.print("servo \t");
  
  /*
   Serial.print(scaled.XAxis);
   Serial.print("   ");   
   Serial.print(scaled.YAxis);
   Serial.print("   ");   
   Serial.print(scaled.ZAxis);
   
   Serial.print("Heading:\t");
   Serial.print(heading);
   Serial.print(" Radians   \t");
   */
   

   Serial.println(" \n");
}
