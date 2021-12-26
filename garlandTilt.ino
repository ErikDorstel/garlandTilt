#include <Adafruit_NeoPixel.h>
#define gpio 15 // serial in led ring
Adafruit_NeoPixel ring=Adafruit_NeoPixel(60,gpio,NEO_GRB+NEO_KHZ800);

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

TwoWire I2Ctwo=TwoWire(1);
Adafruit_MPU6050 mpu;
const float RADtoDEG=57.29577951308232;

struct tiltStruct { float x; float y; float d; float xy; float ax; float ay; float az; float cax; float cay; float caz; };
struct tiltStruct tilt;
unsigned long tiltTimer; int tiltCalibrateCount;
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

float wrap180(float value) {
  while (value>+180) { value-=360; }
  while (value<-180) { value+=360; }
  return value; }

void calibrateTilt() { tilt.cax=tilt.ax; tilt.cay=tilt.ay; tilt.caz=tilt.az-9.81; }

void tiltWorker() {
  if (millis()>=tiltTimer) { tiltTimer=millis()+100; tiltCalibrateCount++;
    if (tiltCalibrateCount==50) { calibrateTilt(); }
    sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t);
    tilt.ax=(tilt.ax+a.acceleration.x)/2; tilt.ay=(tilt.ay+a.acceleration.y)/2; tilt.az=(tilt.az+a.acceleration.z)/2;
    float ax=tilt.ax-tilt.cax; float ay=tilt.ay-tilt.cay; float az=tilt.az-tilt.caz;
    tilt.x=wrap180((+atan2(ay,sqrt(az*az+ax*ax)))*RADtoDEG);
    tilt.y=wrap180((-atan2(ax,sqrt(az*az+ay*ay)))*RADtoDEG);
    tilt.d=wrap180(atan2(ax,ay)*RADtoDEG)+180;
    tilt.xy=sqrt(ax*ax+ay*ay)/9.81*90; } }

void setup() {
  I2Ctwo.begin(17,16,400000); // SDA pin 17, SCL pin 16, 400kHz frequency
  mpu.begin(0x68,&I2Ctwo);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); //2,4,8,16
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);      //250,500,1000,2000
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);    //5,10,21,44,94,184,260
  tilt.ax=0; tilt.ay=0; tilt.az=0; tilt.cax=0; tilt.cay=0; tilt.caz=0;
  tiltTimer=millis()+100; tiltCalibrateCount=0; }

void loop() {
  tiltWorker();
  ring.clear();
  span(tilt.d/6-tilt.xy/10,tilt.xy/5,true,red,blue);
  ring.show(); }
