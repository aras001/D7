#include "PID_v1.h"
#include <Wire.h>
#include "MS5837.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

MS5837 sensor;

const int limit1 = 22; 
const int limit2 = 28; 

double referans; 
double sistem_konum;
double kontrol_sinyali;

const int stepPin = 39; 
const int dirPin = 41; 

const int stepPin3 = 45; 
const int dirPin3 = 47; 

PID Denge(&sistem_konum,&kontrol_sinyali,&referans,3,2,0,DIRECT);

void setup() {

  Serial.begin(9600);
  
  Wire.begin();
  
  Denge.SetOutputLimits(-1,1); 
  Denge.SetSampleTime(10);  // PID çalışma periyodu 5 ms olarak belirlendi.
  Denge.SetMode(AUTOMATIC); 

  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);

  pinMode(limit1,INPUT); 
  pinMode(limit2,INPUT);

  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

}

void loop() {
  
imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

sistem_konum = euler.x();  

sensor.read();

Serial.print("X: ");
Serial.print(euler.x());
Serial.print("  Pressure: "); 
Serial.print(sensor.pressure()); 
Serial.print(" mbar   ");
Serial.print("Temperature: "); 
Serial.print(sensor.temperature()); 
Serial.println(" deg C    ");

//4053.0004 milibar ---- 30m
//2026.5002 milibar ---- 10m
//1013.2501 milibar ---- 0m

Denge.Compute();

if(sensor.pressure()< 2026){    // Su altı planörünün bulunduğu derinlik seviyesi 10 metreden az ise referans değeri -30 derece olan PID çalışmaya başlar.    
  
  referans = -30 ;
  
  while ( euler.x() == -30 && referans == -30 ){    // Su altı planörünün açı degeri (pitch) -30 derece ve referans değeri -30 derece ise planör yükselmeye başlar.
    digitalWrite(dirPin3,HIGH); 
    for(int x = 0; x < 10; x++) {   // Su altı planörünün su deposunu direk su ile doldurmak yerine step motorun her 50 adımında bir açı değerini konrol ederek doldurur. Bu sayede açı değeri bozulduğunda PID açıyı düzeltirken yükselmek yerine açı değeri istenilen değerde iken yükselmeye devam edilir. 
      digitalWrite(stepPin3,HIGH); 
      delayMicroseconds(500); 
      digitalWrite(stepPin3,LOW); 
      delayMicroseconds(500); 
    }
  }
}

if(sensor.pressure()> 4053){    // Su altı planörünün bulunduğu derinlik seviyesi 30 metreden fazla ise referans değeri 30 derece olan PID çalışmaya başlar.
  
  referans = 30 ;
  
  while ( euler.x() == 30 && referans == 30 ){    
    digitalWrite(dirPin3,LOW);
    for(int x = 0; x < 10; x++) {  
      digitalWrite(stepPin3,HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin3,LOW);
      delayMicroseconds(500);
    }
  }
}
  
  if(kontrol_sinyali > 0){
    if(limit2==1){                       // limit2 sınır anahtarın durumu kontrol edilir. limit1 sınır anahtarı basılı ise LOW değil ise HIGH dır.
      digitalWrite(dirPin,HIGH);
      for(int x = 0; x < 5; x++) { 
        digitalWrite(stepPin,HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin,LOW);
        delayMicroseconds(500);
      }
    }else {                             //  limit2 sınır anahtarı basılı ise step motor ters yönde 5000 adım çalışarak yük lineer sistemin orta noktasına taşınır.
    
      digitalWrite(dirPin,LOW); 
      for(int x = 0; x < 5000; x++) { 
        digitalWrite(stepPin,HIGH); 
        delayMicroseconds(500); 
        digitalWrite(stepPin,LOW); 
        delayMicroseconds(500); 
      }
    }
    }
  
  if(kontrol_sinyali < 0){
    if(limit1==1){                       // limit1 sınır anahtarın durumu kontrol edilir. limit1 sınır anahtarı basılı ise LOW değil ise HIGH dır.
      digitalWrite(dirPin,LOW);
      for(int x = 0; x < 5; x++) { 
        digitalWrite(stepPin,HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin,LOW);
        delayMicroseconds(500);
      }
    }else {                             //  limit1 sınır anahtarı basılı ise step motor ters yönde 5000 adım çalışarak yük lineer sistemin orta noktasına taşınır.
    
      digitalWrite(dirPin,HIGH); 
      for(int x = 0; x < 5000; x++) { 
        digitalWrite(stepPin,HIGH); 
        delayMicroseconds(500); 
        digitalWrite(stepPin,LOW); 
        delayMicroseconds(500); 
      }
    }
    }
  
}

