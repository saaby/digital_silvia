#include <SoftwareSerial.h>
#include "PID.h"

#define lcdTxPin        2   // LCD Tx
#define consoleRxPin    0   // Console Rx
#define consoleTxPin    1   // Console Tx
#define ssrSigPin       9   // SSR relay signal
#define ssrGndPin       8   // SSR relay ground
#define tempSigPin      0   // Temperature sensor singal
#define tempGndPin      6   // Temperature sensor ground 
#define tempVccPin      7   // Temperature sensor supply
#define shotGndPin     10   // Shot switch relay Gnd (Black)
#define shotVccPin     11   // Shot switch relay Vcc (Brown)
#define shotSignalPin  12   // Shot switch relay signal (Green)

#define boilerSetpoint 95   // Target boilertemp

int   tempRaw = 0;          // Raw 0-1024 input from LM35AH
float tempFloat = 0;        // Floatingpoint temperature in celsius
int   tempInt = 0;          // Integer temperature in celsius

// Timer variables
unsigned long shotStart;
boolean lastShotState = LOW;

//PID: Define Variables we'll be connecting to
double pidSetpoint, pidInput, pidOutput;

//PID: Specify the links and initial tuning parameters, last three: P,I,D
double pidP = 500;
double pidI = 3;
double pidD = 4;
PID boilerPID(&pidInput, &pidOutput, &pidSetpoint,pidP,0,pidD, DIRECT);
// Parameters and what they do (sort of)
// P_Param:  the bigger the number the harder the controller pushes.
// I_Param:  the SMALLER the number (except for 0, which turns it off,)  the more quickly the controller reacts to load changes, but the greater the risk of oscillations.
// D_Param: the bigger the number  the more the controller dampens oscillations (to the point where performance can be hindered)

// LCD setup: since the LCD does not send data back, we should only define the txPin
SoftwareSerial LCD = SoftwareSerial(0, lcdTxPin);
// Serial console setup:
SoftwareSerial Console = SoftwareSerial(consoleRxPin, consoleTxPin);

void setup() {
  // Pin setup
  pinMode(lcdTxPin, OUTPUT);
  pinMode(consoleRxPin, INPUT);
  pinMode(consoleTxPin, OUTPUT);
  pinMode(ssrSigPin, OUTPUT);
  pinMode(ssrGndPin, OUTPUT);
  pinMode(tempVccPin, OUTPUT);
  pinMode(tempGndPin, OUTPUT);
  pinMode(shotSignalPin, INPUT);
  pinMode(shotVccPin, OUTPUT);
  pinMode(shotGndPin, OUTPUT);

  // Setup supply voltages 
  digitalWrite(ssrGndPin,LOW);
  digitalWrite(tempVccPin,HIGH);
  digitalWrite(tempGndPin,LOW);
  digitalWrite(shotVccPin,HIGH);
  digitalWrite(shotGndPin,LOW);

  // PID: initialize the variables we're linked to
  pidInput = double(tempInt);
  pidSetpoint = boilerSetpoint;
  boilerPID.SetOutputLimits(0,10000); //tell the PID to range the pidOutput from 0 to 2000 
  boilerPID.SetSampleTime(1000);
  pidOutput = 0; //start the pidOutput at 0% and let the PID adjust it from there

  // Lets initialize the processes
  boilerPID.SetMode(AUTOMATIC);
  LCD.begin(9600);
  Console.begin(9600);
  // Wait for LCD and stuff to initialize
  delay(1000);
  displaySplash(2000);
}

int i=0;
void loop() {
  if (i % 10 == 0) {
    readTemp();
    updateConsole();
    updateLcd();
    updatePid();      
 }
 updateRelay();
 delay(10);
 i++;
 if (i == 100)
   i=0;
}

void readTemp() {
  tempRaw = (analogRead(tempSigPin));
  tempFloat = ((5.0 * tempRaw * 100.0)/1024.0);
  tempInt = int(tempFloat);
}

void updatePid() {
  // Avoid Integral windup  
  if (tempInt < (boilerSetpoint-5))
    boilerPID.SetTunings(pidP,0,pidD);
  else if (tempInt > (boilerSetpoint+5))
    boilerPID.SetTunings(pidP,0,pidD);
  else
    boilerPID.SetTunings(pidP,pidI,pidD);
  //Update PID input
  pidInput = double(tempInt);
  //Compute PID values
  boilerPID.Compute();
}

void updateRelay() {
  if (i < (pidOutput/100))
    digitalWrite(ssrSigPin,HIGH);
  else
    digitalWrite(ssrSigPin,LOW);
}
  
void updateConsole() {
  Console.print(tempInt);
  Console.print("\n");
}

void updateLcd() {
  // Display mode
  lcdSelectLineOne();
  if (digitalRead(shotSignalPin) == LOW) {
    if (lastShotState == HIGH) {
      boilerPID.SetMode(AUTOMATIC);      
    }
  }
  if (digitalRead(shotSignalPin) == HIGH) {
    if (lastShotState == LOW) {
      boilerPID.SetMode(MANUAL);
      pidOutput = 0;
      shotStart = millis();
      LCD.print("                ");
      lcdSelectLineOne();
    }
    LCD.print("Pull    ");
    if (((millis()-shotStart)/1000) > 9) {
      lcdSelectLineTwo();
      LCD.print("");
      LCD.print((millis()-shotStart)/1000);
      LCD.print(" sec");
    }
    lastShotState = HIGH;
  }
  else if (tempInt < (boilerSetpoint-2)) { 
    LCD.print("Heating ");
    lastShotState = LOW;
  } else if (tempInt > 105) {
    LCD.print("Steam   ");
    lcdSelectLineTwo();
    LCD.print("        ");
    lastShotState = LOW;
  } else if (tempInt > (boilerSetpoint+2)) {
    LCD.print("High    ");
    lcdSelectLineTwo();
    LCD.print("        ");
    lastShotState = LOW;
  } else {
    LCD.print("Ready   ");
    lcdSelectLineTwo();
    LCD.print("        ");
    lastShotState = LOW;
  }

  // Display temperature
  lcdGoTo(10);
  LCD.print("T:");
  if (tempInt < 100)
    LCD.print(" ");
  LCD.print(tempInt);
  LCD.write(223);   // Degrees symbol
  
  // Print boiler power:
  lcdGoTo(26);
  LCD.print("P:");
  if ((int(pidOutput)/100) < 10) 
    LCD.print("  ");
  else if  ((int(pidOutput)/100) < 100)
    LCD.print(" ");
  LCD.print(int(pidOutput)/100);
  LCD.print("%");
}

void displaySplash(int displaytime) {
  Console.print("Digital Silvia checking in!\n\n");
  lcdClear();
  lcdSelectLineOne();
  LCD.print("Rancilio");
  lcdSelectLineTwo();
  LCD.print("Digital Silvia");
  delay(displaytime);
  lcdClear();
}

void lcdSelectLineOne() {  //puts the cursor at line 0 char 0.
  LCD.write(0xFE);   //command flag
  LCD.write(128);    //position
}

void lcdSelectLineTwo() {  //puts the cursor at line 0 char 0.
  LCD.write(0xFE);   //command flag
  LCD.write(192);    //position
}

void lcdGoTo(int position) {             //position = line 1: 0-15, line 2: 16-31, 31+ defaults back to 0
  if (position<16) {
    LCD.write(0xFE);               //command flag
    LCD.write((position+128));     //position
  } else if (position<32) {
    LCD.write(0xFE);               //command flag
    LCD.write((position+48+128));  //position 
  } else {
    lcdGoTo(0);
  }
}

void lcdClear() {
 LCD.write(0xFE);    //command flag
 LCD.write(0x01);    //clear command.
}

void lcdBacklightOn() {    //turns on the backlight
  LCD.write(0x7C);   //command flag for backlight stuff
  LCD.write(157);    //light level.
}

void lcdBacklightOff() {   //turns off the backlight
  LCD.write(0x7C);   //command flag for backlight stuff
  LCD.write(128);    //light level for off.
}

void lcdSerCommand() {     //a general function to call the command flag for issuing all other commands   
  LCD.write(0xFE);
}
