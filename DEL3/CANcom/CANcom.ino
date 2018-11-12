#include <FlexCAN.h>
#include <Wire.h>
#include <Metro.h>

#ifndef __MK66FX1M0__
  #error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif

typedef struct {                               // Struct containing MPU-data 
   
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
  
}MPUData;


Metro MPUtime = Metro(1000);                   // Set time delay in ms for MPU data transfer
Metro toggleLed = Metro(10000);                // Time delay in ms of which to toggle the onboard LED on/off

const int MPU_addr=0x68;                       // I2C address of the MPU-6050

void tglLedCan();                              // Function that sends 0x21 LED toggle message
void ledToggle();                              // Function that toggles the LED
void ledFromCan(bool CANLedState);             // Function that writes state to LED (implemented to read LSB in 0x22)
void readCAN();                                // Function that designates what to do when reading messages.
void AccCompToCan(MPUData mpud);               // Function that compares Acc in X and Y direction and turns LED off if AccY > AccX, sends message only when state changes.

MPUData getMPUdata();                          // Function that gets IMU data via I2C

CAN_message_t MPU2CAN(MPUData dataRec);        // Function that packs IMU-data into CAN-message

void setup(){
  delay(1000);
  Serial.println(F("Hello Teensy 3.6 dual CAN Test."));
  
  // I2C Initialization
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  // 

  // CAN initialization  
  Can0.begin(1000000);  
  Can1.begin(1000000);
  //
  
  // Enable pins initialized and set low
  pinMode(2, OUTPUT);
  pinMode(35, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(35, LOW);
  //

  // Pin for onboard LED, set high at program start
  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);
  //
 
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void loop(){
  while(Can0.available()) {
    readCAN();                              // Reads CAN data
  }

  AccCompToCan(getMPUdata());               // Function sends signal to turn LED on/ off when AcX < AcY. Does this only when state changes.
  
  if(MPUtime.check() == 1){
    Can1.write(MPU2CAN(getMPUdata()));      // sends the MPU data over CAN with freq 1 Hz
  }

  if(toggleLed.check() == 1){               // Sends toggleLed message with freq 1/10 Hz
  
    tglLedCan();
  
  }
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void AccCompToCan(MPUData mpud){
  
  static CAN_message_t msgLed;

  static bool lastStateComp;
  
  msgLed.ext = 0;
  msgLed.id = 0x22;
  msgLed.len = 1;
  msgLed.buf[0] = 1; 
  bool currStateComp = mpud.AcX > mpud.AcY;

  if(currStateComp != lastStateComp && lastStateComp == 1){

    msgLed.buf[0] = 0;
    Can1.write(msgLed);
    
  }

  if(currStateComp != lastStateComp && lastStateComp == 0){

    msgLed.buf[0] = 1;
    Can1.write(msgLed);
    
  }

  lastStateComp = currStateComp;
}

CAN_message_t MPU2CAN(MPUData dataRec){   // function for packing MPU data into a CAN message
  
  static CAN_message_t msgMPU;
  
  msgMPU.ext = 0;
  msgMPU.id = 0x20;
  msgMPU.len = 6;
  
  msgMPU.buf[0] = lowByte(dataRec.AcX);
  msgMPU.buf[1] = highByte(dataRec.AcX);
  msgMPU.buf[2] = lowByte(dataRec.AcY);
  msgMPU.buf[3] = highByte(dataRec.AcY);
  msgMPU.buf[4] = lowByte(dataRec.AcZ);
  msgMPU.buf[5] = highByte(dataRec.AcZ);

  return msgMPU;
}

MPUData getMPUdata(){   // Function for getting MPU data from I2C and returning MPU data in struct. 
  MPUData dataRec;
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  
  dataRec.AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  dataRec.AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  dataRec.AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  dataRec.Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  dataRec.GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  dataRec.GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  dataRec.GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  return dataRec;
}

void readCAN(){     // Function for reading incoming CAN data and executing commands based on what message arrives.

    CAN_message_t inMsg;
    
    Can0.read(inMsg);

switch(inMsg.id){
  case(0x21):
    {
      ledToggle();
      break;
    }

    case(0x22):
    {
      ledFromCan(bitRead(inMsg.buf[0],0));
      break;
    }

    case(0x20):
    {
      int16_t AccX = inMsg.buf[1]<<8|inMsg.buf[0];
      int16_t AccY = inMsg.buf[3]<<8|inMsg.buf[2];
      int16_t AccZ = inMsg.buf[5]<<8|inMsg.buf[4];

      Serial.print(AccX);
      Serial.print(" ");
      Serial.print(AccY);
      Serial.print(" ");
      Serial.println(AccZ); 
      break;
    }
  
  }
    
  
}

void ledToggle(){   // Function for toggling LED on / off

  digitalWrite(13,!digitalRead(13));
  
}

void ledFromCan(bool CANLedState){    // Function used for LED control by CANbus msg 0x22

  digitalWrite(13,CANLedState);
  
}

void tglLedCan(){     // Function for initializing led toggle and led control CAN messages

  static CAN_message_t msgLedTgl;
  
  msgLedTgl.ext = 0;
  msgLedTgl.id = 0x21;
  msgLedTgl.len = 1;

  Can1.write(msgLedTgl);

}

