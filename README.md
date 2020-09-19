# ESP32-Chandelier
ESP32 Chandelier

// This project uses an ESP32 Huzzah board to control a drone balloon with an LED strip and a BNO055 sensor to monitor rotation. It connects to Unity via OSC. 

September 2020

// OSC IMU
// Nathan Adams / Atau Tanaka - Goldsmiths, University of London
// 2020

// OSCIMU.ino
// Main code for sensing orientation and sending an OSC message

// Code is based on OSC_propShield, which was designed to work on a Teensy 3.1, Teensy PropShield and Adafruit Airlift Wifi (which uses the ESP32 Wifi coprocessor)
// for this project we are using the Adafruit HUZZAH32 Feather board, with the Adafruit 9DoF Fusion Breakout based on the Bosch BNO055. So this code is
// slightly reworked to use the appropriate libaries (e.g. Wifi instead of the Adafruit WifiNINA port / Adafruits Sensor library)

// Build History
// Jan 2020 -> 1-prototype-build - get basics working across wifi

// touchOSC code https://trippylighting.com/teensy-arduino-ect/touchosc-and-arduino-oscuino/

//Open Sound Control (OSC) library for the ESP8266/ESP32
//
//  Example for receiving open sound control (OSC) messages on the ESP8266/ESP32
//  Send integers '0' or '1' to the address "/led" to turn on/off the built-in LED of the esp8266.
//
//  This example code is in the public domain.
//
//  To substitute the analogWrite function I followed this tutorial:
//  https://www.youtube.com/watch?v=ZIhKmUGSpIo&list=PLTqVscZhxJE4oK2BxJwBQEVHRpRW1Wevg&index=54&t=198s
//
//  One way to improve this is to include the analogWrite fucntion using this:
//  https://github.com/ERROPiX/ESP32_AnalogWrite
//
