// Secrets file
#include "secrets.h"

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <WiFiUdp.h>
#include <OSCMessage.h>  /// https://github.com/CNMAT/OSC
#include <OSCBundle.h>  /// https://github.com/CNMAT/OSC
#include <OSCData.h>

// OSC Messaging
int sendCount;
int count;
int lastEntry;
const IPAddress outIp(SECRET_IP);
WiFiUDP Udp;
const unsigned int localPort = 8881;        // local port to listen for UDP packets (here's where we send the packets)
OSCErrorCode error;
#define SERIAL_SPEED 115200
#define VERSION 0.1

// Core Handling
TaskHandle_t Task1, Task2;
SemaphoreHandle_t Semaphore;

int counter = 0;

//LED strip WS2812B

#include <FastLED.h>

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 20;
int x;
int tail;

#define LED_PIN     18
//#define CLOCK_PIN 4
#define NUM_LEDS    111
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS  64

CRGB leds[NUM_LEDS];

//the Built-in-LED
static volatile unsigned int ledState = LOW;              // LOW means led is *on*

#ifndef BUILTIN_LED
#ifdef LED_BUILTIN
#define BUILTIN_LED LED_BUILTIN
#else
//#define BUILTIN_LED 13 
#endif
#endif


// BNO055 / Adafruit Sensor library
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define BNO055_ADDRESS 0x28
//BNO055 is by standard for the fusion board on 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);



// MOTORS
float speedValue;
float directionMotor1 = LOW; // touchOSC only takes on floats
float speed2Value;
float directionMotor2 = LOW; // touchOSC only takes on floats
float speed3Value;
float directionMotor3 = LOW; // touchOSC only takes on floats
float speed4Value;
float directionMotor4 = LOW; // touchOSC only takes on floats

//Motor1
#define in1 17
#define in2 16
#define enA 4// this pin must be PWM enabled pin if Arduino board is used
#define chA 0// this is used for the ledc function to separate the different motors

//Motor2
#define in3 21 // do not use pin 36 for this
#define in4 5
#define enB 19 //this pin must be PWM enabled pin if Arduino board is used
#define chB 1

//Motor3
#define in5 14
#define in6 15
#define enC 32// this pin must be PWM enabled pin if Arduino board is used
#define chC 2// this is used for the ledc function to separate the different motors

//Motor4
#define in7 33
#define in8 12
#define enD 27// this pin must be PWM enabled pin if Arduino board is used
#define chD 3// this is used for the ledc function to separate the different motors

//Motor5 // this has not been tested yet
#define in9 26
#define in10 25
#define enE 13// this pin is also the LED pin so it needs to be redfined must be PWM enabled pin if Arduino board is used 
#define chE 4// this is used for the ledc function to separate the different motors

void codeForTask1( void * parameter )
{
  for (;;) {

    //    xSemaphoreTake(Semaphore, portMAX_DELAY);



    currentMillis = millis();
    if (currentMillis - startMillis >= period && ledState == 1.0) { // the global variable "ledState" is controlled by the touchOSC button /led. It needs a semaphore.
      startMillis = currentMillis;
      if (x > tail) leds[x - tail] = CRGB::Black;
      x++;
      leds[x].setRGB(0, 255, 255);

      FastLED.show();
      if (x >= NUM_LEDS) x = 0;
    }
    //    else {
    //      fill_solid(leds, NUM_LEDS, CRGB::Black);
    //
    //    }
    //    if (ledState == 0.0) {
    //      xSemaphoreGive(Semaphore);
    //  }
    Serial.println(ledState);
  }
}

void codeForTask2( void * parameter )
{
  for (;;) {

    OSCMsgReceive(); //this is not affecting the LED strip, because it's in the other core

//    xSemaphoreTake(Semaphore, portMAX_DELAY);
//    if (ledState == 1.0) {
//      xSemaphoreGive(Semaphore);
//    }else{

    if (directionMotor1 == 1.0) {
      motor1Forward();
      ledcWrite(chA, speedValue); // this uses the ledc function as though it is analogue write
    }
    else {
      motor1Backward();
      ledcWrite(chA, speedValue); // this uses the ledc function as though it is analogue write
    }
    if (directionMotor2 == 1.0) {
      motor2Forward();
      ledcWrite(chB, speed2Value); // this uses the ledc function as though it is analogue write
    }
    else {
      motor2Backward();
      ledcWrite(chB, speed2Value); // this uses the ledc function as though it is analogue write
    }
    if (directionMotor3 == 1.0) {
      motor3Forward();
      ledcWrite(chC, speed3Value); // this uses the ledc function as though it is analogue write
    }
    else {
      motor3Backward();
      ledcWrite(chC, speed3Value); // this uses the ledc function as though it is analogue write
    }
    if (directionMotor4 == 1.0) {
      motor4Forward();
      ledcWrite(chD, speed4Value); // this uses the ledc function as though it is analogue write
    }
    else {
      motor4Backward();
      ledcWrite(chD, speed4Value); // this uses the ledc function as though it is analogue write
    }

    sendCount++;
    sensors_event_t event;

    if (sendCount > 1000) {

      imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      imu::Vector<3> linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      bno.getEvent(&event);
      sendViaOSC(event.orientation.x, event.orientation.y, event.orientation.z, linearAccel.x(), linearAccel.y(), linearAccel.z(), magnetometer.x(), magnetometer.y(), magnetometer.z());
      sendCount = 0;

      //      Serial.println("Task 2: ");
    }
  }
}

// the setup function runs once when you press reset or power the board
void setup() {

  builtInLedSetup();

  ledSetup();

  sensorSetup();

  motorSetup();

//  Semaphore = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    codeForTask1,
    "led1Task",
    10000,
    NULL,
    1, // priority value
    &Task1,
    0);
  delay(500);  // needed to start-up task1

  xTaskCreatePinnedToCore(
    codeForTask2,
    "OSCIMUTask",
    10000,
    NULL,
    1,
    &Task2,
    1);

}

// This function is in charge of assigning values to all the functions according to the osc address from where they come.
void OSCMsgReceive() {
  OSCMessage msgIN;
  int size;
  if ((size = Udp.parsePacket()) > 0) {
    while (size--)
      msgIN.fill(Udp.read());
    if (!msgIN.hasError()) {
      msgIN.dispatch("/led", builtInLed); //msgIN.dispatch is a full match, msIN.route is a partial match
      msgIN.dispatch("/motor1", motor1Speed);// the first argument is the osc address of the incoming message an
      msgIN.dispatch("/motor2", motor2Speed);
      msgIN.dispatch("/motor3", motor3Speed);
      msgIN.dispatch("/motor4", motor4Speed);
      msgIN.dispatch("/direction1", motor1Direction);
      msgIN.dispatch("/direction2", motor2Direction);
      msgIN.dispatch("/direction3", motor3Direction);
      msgIN.dispatch("/direction4", motor4Direction);
    }
  }
}
void builtInLed(OSCMessage &msg) {
  ledState = msg.getFloat(0);
  digitalWrite(BUILTIN_LED, ledState);
  Serial.print("/led: ");
  Serial.println(ledState);
}
void sendViaOSC(float gyroX, float gyroY, float gyroZ, float acX, float acY, float acZ, float oX, float oY, float oZ) {

  OSCMessage msg("/oscimu");
  msg.add((int32_t)gyroX);
  msg.add((int32_t)gyroY);
  msg.add((int32_t)gyroZ);
  msg.add((int32_t)acX);
  msg.add((int32_t)acY);
  msg.add((int32_t)acZ);
  msg.add((int32_t)oX);
  msg.add((int32_t)oY);
  msg.add((int32_t)oZ);
  //send out
  Udp.beginPacket(outIp, SECRET_OUTPORT);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  //  sendCount = 0;
}

void builtInLedSetup() {
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, ledState);    // turn *on* led
}

void ledSetup() {

  startMillis = millis();

  pinMode(LED_PIN, OUTPUT);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(BRIGHTNESS);
  //  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  //  FastLED.show();
  x = 0;
  tail = 4; //this is the number of LEDs that light up at the same time
}
void motorSetup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);

  ledcAttachPin(enA, chA);
  ledcSetup(chA, 5000, 8);
  ledcAttachPin(enB, chB); //the second integer is the Channel number
  ledcSetup(chB, 5000, 8); // the 1st integer is the channel number
  ledcAttachPin(enC, chC);
  ledcSetup(chC, 5000, 8);
  ledcAttachPin(enD, chD);
  ledcSetup(chD, 5000, 8);

}
//DIRECTION OUTPUT
void motor1Forward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void motor1Backward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
void motor2Forward() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
void motor2Backward() {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void motor3Forward() {
  digitalWrite(in5, LOW);
  digitalWrite(in6, HIGH);
}
void motor3Backward() {
  digitalWrite(in5, HIGH);
  digitalWrite(in6, LOW);
}
void motor4Forward() {
  digitalWrite(in7, LOW);
  digitalWrite(in8, HIGH);
}
void motor4Backward() {
  digitalWrite(in7, HIGH);
  digitalWrite(in8, LOW);
}
//SPEED
void motor1Speed(OSCMessage &msg) {
  speedValue = msg.getFloat(0);
}
void motor2Speed(OSCMessage &msg) {
  speed2Value = msg.getFloat(0);
}
void motor3Speed(OSCMessage &msg) {
  speed3Value = msg.getFloat(0);
}
void motor4Speed(OSCMessage &msg) {
  speed4Value = msg.getFloat(0);
}
//DIRECTION INPUT
void motor1Direction(OSCMessage &msg) {
  directionMotor1 = (boolean) msg.getFloat(0);
}
void motor2Direction(OSCMessage &msg) {
  directionMotor2 = (boolean) msg.getFloat(0);
}
void motor3Direction(OSCMessage &msg) {
  directionMotor3 = (boolean) msg.getFloat(0);
}
void motor4Direction(OSCMessage &msg) {
  directionMotor4 = (boolean) msg.getFloat(0);

}



void sensorSetup() {

  Serial.begin(115200);
  // initialize digital pin LED_BUILTIN as an output.

  Serial.begin(SERIAL_SPEED);
  while (!Serial) {
    //wait for Serial to start up
  }

  //Try to connect to the Wifi netowrk provided in secrets
  Serial.print("OSC IMU ");
  Serial.println(VERSION);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(SECRET_SSID);

  WiFi.begin(SECRET_SSID, SECRET_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }

  Serial.println();
  Serial.print("Success! Connected, IP Address is ");
  Serial.println(WiFi.localIP());
  //
  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);


  //Now try to find the BNO055 sensor
  if (bno.begin()) {
    Serial.print("Found BNO055 on I2C address 0x");
    Serial.println(BNO055_ADDRESS, HEX);
  } else {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);
}


void loop() {
      delay(1000);
//  vTaskDelete(NULL);
}
