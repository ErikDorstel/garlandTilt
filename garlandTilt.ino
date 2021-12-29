#include <Adafruit_NeoPixel.h>
#define gpio 15 // serial in led ring
Adafruit_NeoPixel ring=Adafruit_NeoPixel(60,gpio,NEO_GRB+NEO_KHZ800);

//get library from https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev and
//get library from https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "MPU6050_6Axis_MotionApps612.h"

TwoWire I2Ctwo=TwoWire(1);
MPU6050 mpu(0x68,&I2Ctwo);

struct tiltStruct { float x; float y; float z; float d; float xy; };
struct tiltStruct tilt;
unsigned long tiltTimer=millis()+100;

uint8_t fifoBuffer[64]; Quaternion q; VectorFloat gravity; float ypr[3];
int red=ring.Color(5,0,0); int green=ring.Color(0,5,0); int blue=ring.Color(0,0,5);

void addPixelColor(int index,int rA,int gA,int bA) {
  int color=ring.getPixelColor(index);
  int b=color & 255; int g=(color >> 8) & 255; int r=(color >> 16) & 255;
  r+=rA; b+=bA; g+=gA;
  if (r<0) { r=0; } if (r>255) { r=255; }
  if (g<0) { g=0; } if (g>255) { g=255; }
  if (b<0) { b=0; } if (b>255) { b=255; }
  ring.setPixelColor(index,r,g,b); }

void span(int index,int count,bool mode,int colorA,int colorB) {
  float rD=0; float gD=0; float bD=0; int dir=1;
  if (count<0) { dir=-1; count*=-1; }
  int bA=colorA & 255; int gA=(colorA >> 8) & 255; int rA=(colorA >> 16) & 255;
  int bB=colorB & 255; int gB=(colorB >> 8) & 255; int rB=(colorB >> 16) & 255;
  if (count>=2) { rD=(float)(rB-rA)/(count-1); gD=(float)(gB-gA)/(count-1); bD=(float)(bB-bA)/(count-1); }
  for (int x=0;x<count;x++) {
    int y=index+(x*dir); if (y>59) { y-=60; } if (y<0) { y+=60; }
    int r=rA+(x*rD); int g=gA+(x*gD); int b=bA+(x*bD);
    if (mode==false) { r*=-1; g*=-1; b*=-1; }
    addPixelColor(y,r,g,b); } }

void calibrateTilt() { mpu.CalibrateAccel(6); mpu.CalibrateGyro(6); mpu.PrintActiveOffsets(); }

void tiltWorker() {
  if (millis()>=tiltTimer) { tiltTimer=millis()+100;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q,fifoBuffer);
      mpu.dmpGetGravity(&gravity,&q);
      mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);
      if (ypr[2]<-M_PI/2 || ypr[2]>M_PI/2) { if (ypr[1]<0) { ypr[1]=-M_PI-ypr[1]; } else { ypr[1]=M_PI-ypr[1]; } }
      if (ypr[1]<-M_PI/2 || ypr[1]>M_PI/2) { if (ypr[2]<0) { ypr[2]=-M_PI-ypr[2]; } else { ypr[2]=M_PI-ypr[2]; } }
      tilt.z=ypr[0]*180/M_PI; tilt.y=ypr[1]*180/M_PI; tilt.x=ypr[2]*180/M_PI;
      tilt.d=(atan2(ypr[1],ypr[2])*180/M_PI)+180;
      tilt.xy=sqrt(ypr[1]*ypr[1]+ypr[2]*ypr[2])*180/M_PI; } } }

void setup() {
  I2Ctwo.begin(17,16,400000); //SDA 17,SCL 16, 400000 Hz
  mpu.initialize();
  mpu.dmpInitialize();
  //mpu.setXGyroOffset(27); mpu.setYGyroOffset(-10); mpu.setZGyroOffset(53);
  //mpu.setXAccelOffset(2400); mpu.setYAccelOffset(169); mpu.setZAccelOffset(2270);
  calibrateTilt();
  mpu.setDMPEnabled(true); }

void loop() {
  tiltWorker();
  ring.clear();
  span(tilt.d/6-tilt.xy/10,tilt.xy/5,true,red,blue);
  ring.show(); }
