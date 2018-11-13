#include<Wire.h>
#include<Math.h>

//Declearing complex types
typedef struct{
  float x;
  float y;
  float z;
}Vector3D;

enum AFSSEL {_2G, _4G, _8G, _16G};

//Declaring constants
const int MPU_addr=0x68;  // I2C address of the MPU-6050
const AFSSEL scale = _2G;

const int ledPin = 13;

Vector3D accel = {0,0,0};
float accelLast = 0;
float accelCurrent = 0;

float jerk;

const float jerkThreshold = 10;

unsigned long sampleMpuLast = 0;
unsigned long ledLastOn = 0;
const unsigned int sampleMpuRate = 10; //Sampeling every ... ms
const unsigned int ledOffDelay = 2000;

//Declearing functions
void beginMpu(void);
void configMpu(const int lsbScaling);
Vector3D readMpu(const AFSSEL scale);
float absVector3D(Vector3D accel);
int AFS_SEL_SET(AFSSEL a);

void setup()
{
  //Setting pinmodes
  pinMode(ledPin, OUTPUT);

  //Begining wire and MPU
  
  Wire.begin();
  beginMpu();
  configMpu(AFS_SEL_SET(scale));

  //Serial begin
  Serial.begin(9600);
}

void loop()
{
  //Sampeling and taking the time derivative at a known sample rate
  if(millis() - sampleMpuLast >= sampleMpuRate)
  {
    sampleMpuLast = millis();

    //Reading MPU
    accel = readMpu(scale);

    //Shifting current to last val
    accelLast = accelCurrent;
    accelCurrent = absVector3D(accel);

    //Time derivative to get g/s
    jerk = (accelCurrent - accelLast)/(0.001f*sampleMpuRate);
  }

  //Setting led on if jerk threshold is reached, and keeping the led on for ledOffDelay
  if(jerk >= jerkThreshold)
  {
    ledLastOn = millis();
    digitalWrite(ledPin, HIGH);
  }
  else if(digitalRead(ledPin) == true && millis() - ledLastOn >= ledOffDelay)
  {
    digitalWrite(ledPin, LOW);
  }
}

//Implementing functions
void beginMpu(void)
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}
void configMpu(const int lsbScaling)
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);         //Sets adress to ACCEL_CONFIG adress 0x1C
  Wire.write(lsbScaling);   //Sets to LSB scaling factor
  Wire.endTransmission(true);
}
Vector3D readMpu(const AFSSEL scale)
{
  Vector3D accel;
  int16_t AcX,AcY,AcZ;
  int16_t ssf = 16384/(scale+1);
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,6,true);  // request a total of 6 registers
  
  AcX=Wire.read()<<8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  accel.x = (float)AcX/(float)ssf;
  accel.y = (float)AcY/(float)ssf;
  accel.z = (float)AcZ/(float)ssf;

  return accel;
}
float absVector3D(const Vector3D accel)
{
  return sqrt(pow(accel.x, 2) + pow(accel.y, 2) + pow(accel.z, 2));
}
int AFS_SEL_SET(AFSSEL a)
{
  int b = 0;
  
  switch(a)
  {
    case _2G   : b = 0b00000000;  break;
    case _4G   : b = 0b00001000;  break;
    case _8G   : b = 0b00010000;  break;
    case _16G  : b = 0b00011000;  break;
  }
  
  return b;
}










