#define SECRET_SSID "your internet"
#define SECRET_PASSWORD "your password"
//#define SECRET_IP  192, 168, 1, 85 // this is the IPaddress for the MacPro
#define SECRET_IP  192, 168, 1, 95 // this is the IP address for the laptop
//#define SECRET_IP  192, 168, 1, 121 // this is the IPaddress for the Iphone
#define SECRET_OUTPORT 9992

//#define SECRET_PASSWORD "Fb)M2}8ht"
//#define SECRET_IP  10, 100, 114, 226

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
