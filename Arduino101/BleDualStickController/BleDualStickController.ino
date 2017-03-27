/*
 * Arduino 101 Sketch sends controller data from 2 analog joystick over bluetooth LE as UART serial data.
 * It also received commands from connected app to activate relays.
 * Used for custom VR haptic controller with Gear VR.
 * 
 * Created by  Leon Hong Chu   @chuartdo
 * See the bottom of this file for the licence and credits 
 */

#include <CurieBLE.h>
#include "CurieIMU.h"

#define lcd_display 1
#ifdef lcd_display 
 #include <Wire.h>
 #include "rgb_lcd.h"
 rgb_lcd lcd;
#endif 


byte txData[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
const int ledPin = 15;  // Led indicator for BLE connection

/* Joystick configuration based on 15 pin */
const int joyStick1XPin = A0;
const int joyStick1YPin = A1;
const int joyStick2XPin = A2;
const int joyStick2YPin = A3;
const int button1Pin = 0;
const int button2Pin = 1;
const int button3Pin = 2;
const int button4Pin = 4;

/* ===== Relay pin assignment for activating feedback devices */
int relayPins[] = { 8,9,10,11 };
int relayPinCount = 4;

BLEPeripheral blePeripheral;   

// Configure Nordic Semiconductor UART service 
BLEService UartService = BLEService("6E400001B5A3F393E0A9E50E24DCCA9E");
BLECharacteristic TransmitCharacteristic = BLECharacteristic("6E400003B5A3F393E0A9E50E24DCCA9E", BLENotify , 20); 
BLECharacteristic ReceiveCharacteristic = BLECharacteristic("6E400002B5A3F393E0A9E50E24DCCA9E", BLEWriteWithoutResponse, 20); 

void setup() {
  Serial.begin(9600);
  pinMode( ledPin, OUTPUT); 
  pinMode( button1Pin, INPUT); 
  pinMode( button2Pin, INPUT); 
  pinMode( button3Pin, INPUT); 
  pinMode( button4Pin, INPUT); 
  pinMode( joyStick1XPin, INPUT); 
  pinMode( joyStick1YPin, INPUT);
  pinMode( joyStick2XPin, INPUT);
  pinMode( joyStick2YPin, INPUT);

  for( int i = 0; i < relayPinCount; i++ ) {
    pinMode( relayPins[i], OUTPUT); 
  }

  #ifdef lcd_display
    lcd.begin(16, 2);
    lcd.print("Calibrate JOYSTX");
    lcd.setCursor(0,1);
    lcd.print("Rotate ");
    lcd.setRGB(100, 100, 100);
  #endif
  
  // ===== Set advertised device name:
  blePeripheral.setLocalName("DualJoy");

  // set characteristics and event handler
  blePeripheral.setAdvertisedServiceUuid(UartService.uuid());
  blePeripheral.addAttribute(UartService);
  blePeripheral.addAttribute(TransmitCharacteristic);
  blePeripheral.addAttribute(ReceiveCharacteristic);

  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  ReceiveCharacteristic.setEventHandler(BLEWritten, ReceiveCharacteristicWritten);

  blePeripheral.begin();

  // Onboard accelerometer setup
  CurieIMU.begin();
  CurieIMU.setAccelerometerRange(2);
  CurieIMU.setGyroRange(250);

  detectController();
}

 
int mode = 1 ;  // Fall back to use internal accelerometer for testing without connected joystick and buttons

void detectController() {
  if (digitalRead(button1Pin)> 0 &&
      digitalRead(button2Pin)> 0)
      mode =21;
}

void loop() {
  
  blePeripheral.poll();  
  
  switch (mode) {
    case 1:  
         AccelerometerLoop(); 
         break;
    case 2:
         GyroLoop();
         break;
    default:
       readJoystickLoop();
  }
}


#ifdef lcd_display
  void writeLCD(String  prefix, float x, float y, int row) {
      String outputMsg;
      outputMsg = String(prefix);
      outputMsg += String(x);
      outputMsg += " ";
      outputMsg += String(y);
      outputMsg += "\0";
  
      for (int i = outputMsg.length(); i<16 ; i++);
        outputMsg += " ";
      lcd.setCursor(0,row);
      Serial.print(outputMsg);
      lcd.print(outputMsg);
  }
  
  int colorCycle = 1;
  
  void changeColor() {
 
    int r = (0x04 & colorCycle) > 0?50:10;
    int g = (0x02 & colorCycle) > 0?50:10;
    int b = (0x01 & colorCycle) > 0?50:10;
    lcd.setRGB(r, g, b);
    colorCycle += 1;
    if (colorCycle >= 8)
       colorCycle = 0;
  }

#endif


bool sendData = false;

void blePeripheralConnectHandler(BLECentral& central) {
  Serial.print("Connected to ");
  Serial.println(central.address());
  sendData = true;
  #ifdef lcd_display
   lcd.setRGB(0, 0, 20); // Dim LCD
  #endif
  digitalWrite(ledPin, HIGH);
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  Serial.print("Disconnected ");
  Serial.println(central.address());
  sendData = false;
  turnOffAllRelay();
  digitalWrite(ledPin, LOW);
}

void activateRelay(int relayNum, bool state) {
  if (relayNum >= 0 && relayNum < relayPinCount ) {
    
    #ifdef lcd_display
      colorCycle = relayNum + 1;
      if (state > 0)
        changeColor();
      
    else
      Serial.print("Activate relay ");
      Serial.println(relayNum);   
      Serial.print("  ");
      Serial.println(state?"ON":"OFF");
    #endif
    digitalWrite(relayPins[relayNum] ,state);

 }
}
/* Obtain 3 byte command from ble/ Andorid and turn on relay based on number */
/* 3 byte command, relayNum, value */

void ReceiveCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {

  int relayNum=-1;  
  if (characteristic.value()) {      
    int len = characteristic.valueLength(); 
    Serial.print(len);   //print out the character to the serial monitor
    Serial.println(" Received");

    byte* command = (byte*) characteristic.value();
   
    if (len == 3) {
        relayNum = (int) command[1];
        if (relayNum >= 48)  // Map Ascii character for testing via command line
          relayNum -=48;
          
        bool state = command[2] > 0?HIGH:LOW;
        activateRelay(relayNum, state);
    }
  }
}

byte* floatToByteArray(float f, byte* ret) {
    unsigned int asInt = *((int*)&f);
    int i;
    for (i = 0; i < 4; i++) {
        ret[i] = (asInt >> 8 * i) & 0xFF;
    }
    return ret;
}

byte* longToByteArray(long f, byte* ret) {
    unsigned int asInt = *((long*)&f);
    int i;
    for (i = 0; i < 4; i++) {
        ret[3-i] = (asInt >> 8 * i) & 0xFF;
    }
    return ret;
}

void GyroLoop () {
  float gx, gy, gz;
  CurieIMU.readGyroScaled(gx, gy, gz);
  sendControlData(gx, gy, gz, 0, 0);
}

void AccelerometerLoop () {
  float ax, ay, az;
  CurieIMU.readAccelerometerScaled(ax, ay, az);
  #ifdef lcd_display
   writeLCD("Acc: ",ax,ay,0);
   writeLCD("z: ",az,0,1);
  #endif
  sendControlData(ax, ay, az, 0, 0);
}

// Control data fit within 20 bytes. 2 joystick axis data + 4 button states
// ===== Modify to place custom controller data

void sendControlData(float x1, float y1, float x2, float y2, long buttons) {
   floatToByteArray(x1, &txData[0]);
   floatToByteArray(y1, &txData[4]);
   floatToByteArray(x2, &txData[8]);
   floatToByteArray(y2, &txData[12]);
   longToByteArray(buttons, &txData[16]);

/*
for (int i=0; i<20; i++) {
  Serial.print(txData[i],HEX);
}
*/
    Serial.print(x1); Serial.print(" ");
    Serial.print(y1); Serial.print(" ");
    Serial.print(x2); Serial.print(" ");
    Serial.print(y2); Serial.print(" ");
    Serial.println(buttons); 
   
    TransmitCharacteristic.setValue(txData, 20);  
}


float mapfloat(float val, float in_min, float in_max, float out_min, float out_max)
{
  return out_min + (out_max - out_min) * (( val - in_min) / ( in_max - in_min));
}

float normalize(float val, float minimum, float center, float maximum) {
  if (val > center) {
    val =  mapfloat(val,center, maximum, 0, 1.1);
    if (val > 1.0)
      val = 1.0;
  }
  else {
     val =  mapfloat(val,minimum, center, -1.0, 0);
  }
  return val;
}
 
// Read data store in following encoded bytes  
// Convert to nomalized value -1 to 1  O is near center


float getMinMax(int source, int* min, int* max) {
  if (source < *min) 
     *min = source;
  else if (source > *max)
     *max = source;
  return source;
}

void turnOffAllRelay() {
  for (int i=0; i< relayPinCount; i++) {
    digitalWrite(relayPins[i] ,LOW);
  }  
}

bool buttonState[4];
int buttonStates() {
  int state = 0;
  
  if (!buttonState[0] ) {
    state += 1;
  }
  if (!buttonState[1]) {
    state += 2;
  }
  if (!buttonState[2]) {
    state += 4;
  }
  if (!buttonState[3]) {
    state += 8;
  }
  return state;
}

int max_j1x= -1, min_j1x= 9999, max_j1y=-1, min_j1y=9999;
int max_j2x= -1, min_j2x= 9999, max_j2y=-1, min_j2y=9999;
int ctr_x1=600, ctr_y1=600, ctr_x2=600, ctr_y2=600;

bool calibration = true;
bool recordCenter = true;


void readJoystickLoop () {
  int x1a, y1a, x2a, y2a;
  
  x1a = analogRead(joyStick1XPin);
  y1a = analogRead(joyStick1YPin);
  x2a = analogRead(joyStick2XPin);
  y2a = analogRead(joyStick2YPin);

  if (recordCenter) {  // Get initial center state of joysticks on power on
    #ifdef lcd_display
      lcd.setRGB(50, 50, 50);
      lcd.print("Calibrate JOYSTX");
      lcd.setCursor(0,1);
      lcd.print("Record Center POS");
      delay(1500);   
    #endif
    
    ctr_x1 = x1a;
    ctr_y1 = y1a;
    ctr_x2 = x2a;
    ctr_y2 = y2a;
    recordCenter = false;

    #ifdef lcd_display
     lcd.setRGB(0, 100, 0);
    #endif
  }

  buttonState[0]=digitalRead(button1Pin)==0;
  buttonState[1]=digitalRead(button2Pin)==0;
  buttonState[2]=digitalRead(button3Pin)==0;
  buttonState[3]=digitalRead(button4Pin)==0;
  float buttons =  buttonStates();

 
  String outputMsg = "";
   
  // Get max and min range of joysticks
   getMinMax(x1a,&min_j1x,&max_j1x);
   getMinMax(y1a,&min_j1y,&max_j1y);
   getMinMax(x2a,&min_j2x,&max_j2x);
   getMinMax(y2a,&min_j2y,&max_j2y);

  // normalize stick movement range to -1 and 1 
  float fx1 = normalize(x1a,min_j1x, ctr_x1, max_j1x);
  float fy1 = normalize(y1a,min_j1y, ctr_y1, max_j1y);
  float fx2 = normalize(x2a,min_j2x, ctr_x2, max_j2x);
  float fy2 = normalize(y2a,min_j2y, ctr_y2, max_j2y);
  
  if (sendData) {    
    #ifdef lcd_display
      writeLCD(" 1: ",fx1,fy1,0);
      writeLCD(" 2: ",fx2,fy2,1);
    #endif
    sendControlData(  fx1,  fy1, fx2, fy2, buttons);
    
  } else {

     #ifdef lcd_display
        writeLCD("J1: ",x1a,y1a,0);
        writeLCD("J2: ",x2a,y2a,1);
     #endif

    // Turn on Relay switch with button press
    for (int i=0; i< 3; i++) {
      activateRelay(i ,buttonState[i]?HIGH:LOW);
    }
     
    // Recenter joystick when all button pressed down
    if (buttons == 0) {
      recordCenter = true;
    }
  }
 
}

/*
  The Nordic Semiconductor UART profile for Bluetooth Low Energy
  
  Copyright (c) 2015 Intel Corporation. All rights reserved. 
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-
  1301 USA
*/
