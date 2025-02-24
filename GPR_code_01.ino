/*
GPR_code_01 by Megan Cowan

For USIP Guided Parachute Recovery Team 
Code is still a work in progress and has not been tested 
*/



#include <Wire.h>
#include <SPI.h> //spi = other hardware https://www.arduino.cc/reference/en/language/functions/communication/spi/
#include <Adafruit_BNO08x.h> // Arduino Library for the BNO080 https://github.com/adafruit/Adafruit_BNO08x
#include <NMEAGPS.h> //This fully-configurable Arduino library uses minimal RAM, PROGMEM and CPU time
#include <GPSport.h> // https://github.com/SlashDevin/NeoGPS
#include <Adafruit_BMP280.h> // https://github.com/adafruit/Adafruit_BMP280_Library
#include <SD.h> //Enables reading and writing on SD cards https://www.arduino.cc/reference/en/libraries/sd/
#include <Servo.h> //temp until we learn more about servo
#include <math.h> 

class Node{
private:
  float data;
  unsigned long time;
  Node* next;
public: 
  Node(const float& d, const unsigned long& t){
    data = d;
    time = t;
    next = NULL;
  }
  float getData(){ return data;}
  unsigned long getTime(){ return time;}
  Node* getNext(){ return next;}
  void setData(const float& d){ data = d;}
  void setTime(const unsigned long& t){ time = t;}
  void setNext(Node* n){ next = n;}
};

class Queue{
private:
  Node* head;
  Node* back;
  int size;
public:
  Queue(){
    head = NULL;
    back = NULL;
    size = 0;
  }
  void addBack(const float& d, const unsigned long& t){
    Node* n = new Node(d, t);
    if(size == 0){
      head = n;
      back = n;
    }
    else{
      back->setNext(n); 
      back = n;
    }
    size++;
  }
  void removeFront(){
    Node<T>* curr = head;
    head = head->getNext();
    delete curr;
    s--;
  }
  Node* getHead(){ return head;}
  unsigned long timeDiff(){
    if (size > 1){
      return (back->getTime() - head->getTime);
    }
    return 0;
  }
  int getSize(){return size;}
};


// BNO08X (please insert connection info here)
#define BNO08X_CS A5 //For SPI mode, we need a CS pin
#define BNO08X_INT 5 
#define BNO08X_RESET A4 // For SPI mode, we also need a RESET but not for I2C or UART
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Here is where you define the sensor outputs you want to receive from BNO08X
void setBNO08xReports(void) {
  Serial.println("Setting desired reports");
  delay(25);
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 100000)) {
    Serial.println("Could not enable rotation vector");
  }// ROTATION_VECTOR is in Quaternion
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 500000)) {
    Serial.println("Could not enable linear acceleration");
  }// LINEAR_ACCELERATION is in form x, y, z, in m/s^2"
}

// Rotation Vector 
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

//GPS
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

float lat = 0.0;
float lon = 0.0;
float gpsAlt = 0.0;

//Baro (please insert connection info here)
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK); // software SPI/
float baroAlt = 0.0;

// SD card (please insert connection info here)
#define cardSelect 4
File flightData;
char fileName[] = "FlightData.txt"; // format FlightData_Date.txt

//Target (please set target location here here)
const float targ_lat = 29.7230244; 
const float targ_lon = -95.3401489;

//time logs
unsigned long resetTime = 0;
unsigned long rvTime = 0;
unsigned long last_rvTime = 0;
unsigned long gpslTime = 0;
unsigned long gpsaTime = 0;
unsigned long baroTime = 0;

//Servo 
Servo sRelease;
Servo sDeploy;
Servo sLeft;
Servo sRight;

float currAlt = 0.0;
float distN = 0.0;
float distE = 0.0;
float dist = 0.0; //Distance in meters
float targHead = 0.0; //heading to target in Degrees currHeading is yaw
float err = 0.0;
float last_err = 0.0;
float errSum = 0.0;
float p = 0.0;
float i = 0.0;
float d = 0.0;
int pos = 0;
int turn = 0;
int prevTurn = 0;
unsigned long errTimeSimple = 3500;
unsigned long fallStartTimeSimple = 5000;
unsigned long fallStopTimeSimple = 60000;
bool wasReleased = false;
bool isDeployed = false;
bool left = false;
bool prevLeft = false;
bool right = false;
bool prevRight = false;
Queue errQueue = Queue();
Queue altQueue = Queue();
Queue distQueue = Queue();

bool isFalling(/*Node* altStart, Node* distStart,*/ bool release){ // Inputs: Altitude list  
  //Node* currAlt = altStart;
  if(release){
    return true;
  }
  /*while(curr->getNext() != NULL){

  }*/
  return false;
}

//Please set these values
const float releaseAlt = 0.0;
const float releaseDist = 0.0;
const float deployAlt = 0.0;
const float pK = 1.0;
const float iK = 0.0;
const float dK = 0.0;
const int turnDelay = 15;
const int dereDelay = 1;

void setup() {
  Serial.begin(115200);
  gpsPort.begin(9600); //Initializing GPS
  delay(500);

  //Initializing BNO
  //if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x sensor Found!");

  setBNO08xReports();

  //Initializing BMP
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP280 sensor");
    while (1) { delay(10); }
  }
  Serial.println("BME280 sensor Found!");

  //Initializing SD What data do we record
  if (!SD.begin(cardSelect)) {
    Serial.println("Failed to find SD Card");
    while (1) { delay(10); }
  }
  Serial.println("SD Card Found!");
  flightData = SD.open(fileName, FILE_WRITE);
  if(!flightData) {
    Serial.print("Couldnt create "); 
    Serial.println(fileName);
  }

  // Initializing servos (please insert connection info here)
  sRelease.attach(9);
  sDeploy.attach(9);
  sLeft.attach(9);
  sRight.attach(9);

  resetTime = millis();
}

void loop() {
  //Resetting BNO every minute
  if (resetTime + 60000 <= millis()) {
    bno08x.hardwareReset();
    resetTime = millis()
    delay(100);
  }

  if (bno08x.wasReset()) {
    Serial.println("sensor was reset");
    setBNO08xReports();
    delay(100);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) { 
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      quaternionToEuler(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, &ypr, true);
      last_rvTime = rvTime;
      rvTime = millis();
    }
  }
  
  //Get data from BMP (put pressure at sea level in hpa here)
  baroAlt = bmp.readAltitude();
  baroTime = millis();

  //Get data from GPS
  if (gps.available( gpsPort )) {
    fix = gps.read();
    if (fix.valid.location) {
      lat = fix.latitude();
      lon = fix.longitude();
      gpslTime = millis();
    }
    if (fix.valid.altitude) {
      gpsAlt = fix.altitude();
      gpsaTime = millis();
    }
  }
  
  //Altitude in meters  (code for error later)
  if(gpsaTime < baroTime) {
    currAlt = baroAlt;
    altQueue.addBack(currAlt, baroTime);
  }
  else{
    currAlt = gpsAlt;
    altQueue.addBack(currAlt, gpsaTime);
  }
  
  if(wasReleased){
    if(altQueue.timeDiff() > fallStopTimeSimple){
      altQueue.removeFront();
    }
  }
  else{
    if(altQueue.timeDiff() > fallStartTimeSimple){
      altQueue.removeFront();
    }
  }

  distN = radians(targ_lat - lat);
  distE = radians(targ_lon - lon);
  dist = sqrt( distN*distN + distE*distE)) * 6371008.8; //Distance in meters
  targHead = degrees(atan2(distE,distN)); //heading to target in Degrees currHeading is yaw 
  distQueue.addBack(dist, gpslTime);
  if(distQueue.timeDiff() > fallStopTimeSimple){
    distQueue.removeFront();
  }
  
  //Flight Guidance 
  if(currAlt >= 30){
    //Stage I - Are we falling?
    if(!wasReleased){
      if((currAlt >= releaseAlt) || (dist <= releaseDist)|| isFalling(false)){
        //Activate release servo
        for (pos = 0; pos <= 180; pos += 1) {
          sRelease.write(pos);
          delay(dereDelay);
        }
        wasReleased = true;
      }
    }
    else{
      if(!isDeployed){
        if(currAlt <= deployAlt){
          //Activate deployment servo
          for (pos = 0; pos <= 180; pos += 1) {
            sDeploy.write(pos);
            delay(dereDelay);
          }
          isDeployed = true;
        }
      }
      else{
        if(!isFalling(true)){
          break;
        }
        else{
          prevLeft = left;
          prevRight = right;
          prevTurn = turn;
          if(dist > 50){
            //Stage II - Go to Target
            last_err = err;
            err = targHead - ypr.yaw;
            errQueue.addBack(err, rvTime);
            p = pK * err;
            if (errQueue.getSize() > 1){
              d = dK * ((err - last_err) / (rvTime - last_rvTime));
              if (errQueue.timeDiff() > errTimeSimple){
                errQueue.removeFront();
                Node* curr = errQueue.getHead();
                while (curr->getNext() != NULL){
                  errSum += 0.5 *((curr->getData() + curr->getNext()->getData()) * (curr->getTime() + curr->getNext()->getTime()));
                  curr = curr->getNext();
                }
                i = iK * errSum;
                turn =  round(p + i + d);
              } else{turn =  round(p + d);}
            } else{turn =  round(p);}
            if (turn == 0){
              left = false;
              right = false;
              if (prevLeft){
                for (pos = abs(prevTurn); pos >= 0; pos -= 1) {
                  sLeft.write(pos);
                  delay(turnDelay);
                }
              }
              if (prevRight){
                for (pos = abs(prevTurn); pos >= 0; pos -= 1) {
                  sRight.write(pos);
                  delay(turnDelay);
                }
              }
            }
            if (turn > 0){
              left = true;
              right = false;
              if (prevRight){
                for (pos = abs(prevTurn); pos >= 0; pos -= 1) {
                  sRight.write(pos);
                  delay(turnDelay);
                }
                if (prevLeft){
                  for (pos = abs(prevTurn); pos >= turn; pos -= 1) {
                    sLeft.write(pos);
                    delay(turnDelay);
                  }
                }
                else{
                  for (pos = 0; pos <= turn; pos += 1) {
                    sLeft.write(pos);
                    delay(turnDelay);
                  }
                }
              }
              else{
                if(turn > prevTurn){
                  for (pos = abs(prevTurn); pos <= abs(turn); pos += 1) {
                    sLeft.write(pos);
                    delay(turnDelay);
                  }
                }
                if(turn < prevTurn){
                  for (pos = abs(prevTurn); pos >= abs(turn); pos -= 1) {
                    sLeft.write(pos);
                    delay(turnDelay);
                  }
                }
              }
            }
            if (turn < 0){
              left = false;
              right = true;
              if (prevLeft){
                for (pos = abs(prevTurn); pos >= 0; pos -= 1) {
                  sLeft.write(pos);
                  delay(turnDelay);
                }
                if (prevRight){
                  for (pos = abs(prevTurn); pos >= turn; pos -= 1) {
                    sRight.write(pos);
                    delay(turnDelay);
                  }
                }
                else{
                  for (pos = 0; pos <= turn; pos += 1) {
                    sRight.write(pos);
                    delay(turnDelay);
                  }
                }
              }
              else{
                if(turn > prevTurn){
                  for (pos = abs(prevTurn); pos >= abs(turn); pos -= 1) {
                    sRight.write(pos);
                    delay(turnDelay);
                  }
                }
                if(turn < prevTurn){
                  for (pos = abs(prevTurn); pos <= abs(turn); pos += 1) {
                    sRight.write(pos);
                    delay(turnDelay);
                  }
                }
              }
            }
          }
          else{
            //Stage III - Loiter There
            left = true;
            right = true;
            turn = 180;
            if (prevLeft && !prevRight){
              for (pos = abs(prevTurn); pos <= turn; pos += 1) {
                sLeft.write(pos);
                delay(turnDelay);
              }
              for (pos = 0; pos <= turn; pos += 1) {
                sRight.write(pos);
                delay(turnDelay);
              }
            } 
            else if (!prevLeft && prevRight){
              for (pos = abs(prevTurn); pos <= turn; pos += 1) {
                sRight.write(pos);
                delay(turnDelay);
              }
              for (pos = 0; pos <= turn; pos += 1) {
                sLeft.write(pos);
                delay(turnDelay);
              }
            }
            else if (!prevLeft && !prevRight){
              for (pos = 0; pos <= turn; pos += 1) {
                sLeft.write(pos);
                delay(turnDelay);
              }
              for (pos = 0; pos <= turn; pos += 1) {
                sRight.write(pos);
                delay(turnDelay);
              }
            }
          }
        }
      }
    }
  }

  delay(50);
}