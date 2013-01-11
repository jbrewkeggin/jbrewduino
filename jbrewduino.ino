/* jbrewduino */
/* Developed by Jordan Kagan */ 

// This Arduino sketch reads serves 3 purposes:
// 1. Toggle On/Off a HLT recirculation pump 
// 2. Toggle On/Off a Mash recirculation + sparge pump
// 3. Read input from DS18B20 "1-Wire" digital temperature sensors 
// and control attached heating element based on temperature feed back.
// This will be accomplished via a PID control loop.

// Valid messages are as follows from the Python program:
// "1" means toggle pump 1 off/on
// "2" means toggle pump 2 off/on
// "3" means toggle heating element off/on
// Codes 80 - 212 are reserved for fahrenheit temperatures

// All inputs are provided by a Python brewing GUI that transmits serial messages.
#include <Time.h>
#include <Wire.h>
#include <PID_v1.h>
#include <OneWire.h> /* for reading 1-wire bytes */
#include <DallasTemperature.h> /* For parsing and printing temp from DS18B20 bytes */
#include <SD.h>
// Data wire is plugged into pin 10 on the Arduino
#define ONE_WIRE_BUS 3

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

File dataFile; 
char filename[40];
// Assign the addresses of your 1-Wire temp sensors.

DeviceAddress mashTemp = { 0x28, 0x9E, 0xAC, 0x08, 0x04, 0x00, 0x00, 0x2C };
DeviceAddress hltTemp = { 0x28, 0xFB, 0x1E, 0x15, 0x04, 0x00, 0x00, 0x6D };
String inputString = ""; /* input with temperature from Python GUI */
int newTarget = 0;
int first_write = 0;
const int heatingPin = 6; /* pin in which the heating element is attached - PID output pin */
int tempLoop = 0; /* This is a global run flag for the temperature feedback loop */
/* We need a valid temperature string because we can also receive messages for toggling 
 * the heating elements and pumps */
int validTemp = 0; 
// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = 4;
//PID Constants
int k_prop = 500;
int k_int = 0;
int k_dif = 0;

double Setpoint, Input, Output; /* PID variables */
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,k_prop,k_int,k_dif, DIRECT);
int WindowSize = 1000; /* PID adjusts output between 0 and this window size*/

unsigned long windowStartTime;
unsigned long file_start;

void setup(void)
{
  
  
  pinMode(heatingPin, OUTPUT); 
  // start serial port
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  // Start up the library
  sensors.begin();
  // set the resolution to 10 bit (good enough?)
  sensors.setResolution(mashTemp, 10);
  sensors.setResolution(hltTemp, 10);
  inputString.reserve(30);
  
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  windowStartTime = millis();
 
}

/* function that prints temperatures in human readable form */
void printTemperature(DeviceAddress deviceAddress)
{
  if(first_write == 0) {
    file_start = millis();
    first_write = 1;
  }
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) {
    Serial.print("Error getting temperature");
  } else {
    //Serial.print("C: ");
    //Serial.print(tempC);
    Serial.print(" F: ");
    if (deviceAddress == mashTemp){
      dataFile.print("MASH");
      Input = DallasTemperature::toFahrenheit(tempC);
    }
    if (deviceAddress == hltTemp) {
      dataFile.print("HLT");
    }
    dataFile.print(",");
    dataFile.print(DallasTemperature::toFahrenheit(tempC));
    dataFile.print(",");
    dataFile.print(millis() - file_start);
    dataFile.print(",");
    dataFile.println(Output);
    Serial.println(DallasTemperature::toFahrenheit(tempC));
    
    
  }
}

void temperatureControl(){
  
  sensors.requestTemperatures();     
  Serial.print("Mash temperature is: ");
  printTemperature(mashTemp);
  myPID.Compute();/* This must be called once per loop to compute the new output*/
  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if(millis() - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output > millis() - windowStartTime) {
    Serial.println("turning heating element ON");
    digitalWrite(heatingPin,HIGH); 
  }
  else {
    digitalWrite(heatingPin,LOW); 
    Serial.println("turning heating element off");
  }
  Serial.print("HLT temperature is: ");
  printTemperature(hltTemp);   
}

/*Loop that receives input temperature from Python program */
void receiveInput() {
  
  while (Serial.available() > 0) {
    /* once we have input we can start temp control loop */
    tempLoop = 1;
    // get the new byte
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      Serial.println(inputString); 
      newTarget = inputString.toInt();
      if (newTarget > 60 && newTarget < 215) {
        if(dataFile) {
          dataFile.close();
        }
        sprintf(filename, "LOG-%02d%02d.csv", hour(), minute(), second());
        Serial.println(filename);
        dataFile = SD.open(filename, FILE_WRITE);
        if(dataFile) {
          Serial.println("New file opened successfully!");
          dataFile.print("kp = ");
          dataFile.print(k_prop);
          dataFile.print(", ki = ");
          dataFile.print(k_int);
          dataFile.print(", kd = ");
          dataFile.print(k_dif);
          
        } else {
          Serial.println("Issue opening file..."); 
        }
        validTemp = 1;
        Setpoint = newTarget;
        dataFile.print(", SP = ");
        dataFile.println(Setpoint);
        //tell the PID to range between 0 and the full window size
        myPID.SetOutputLimits(0, WindowSize); /* vary its output within a given range(1 secs here) */  
        //turn the PID on
        myPID.SetMode(AUTOMATIC); /*PID is on, MANUAL for off*/ 
        
      }
      else if (newTarget == 3) {
        Serial.println("Received Start/Stop Message!");
        if (validTemp == 1) {
          dataFile.close();
          digitalWrite(heatingPin, LOW);
          validTemp = 0;
        }
        else {
          dataFile = SD.open(filename, FILE_WRITE);
          digitalWrite(heatingPin, HIGH);
          validTemp = 1;
        }
      } 
      inputString = "";
    } /* if (inChar == '\n') */  
  
  } /* while (Serial.available() > 0) */    
}

void loop(void)
{ 
  receiveInput();  
  if(tempLoop == 1 && validTemp == 1)
   temperatureControl();
  
}/* void loop */
