/*
  Behavior:
  
    while touching 1 goes forward
    while touching 2 goes backwards
    sends same info to 2nd arduino
    also listens on radio for commands to go forward/backward: code for both is same

  Uses
    a capacitance touch board mpr121
    a stepper driver DRV8825
    https://www.pololu.com/product/2133/resources
    https://www.pololu.com/file/0J590/drv8825.pdf
    and a radio HC-12
*/

#include <Wire.h>

// cap touch mpr121
#include "mpr121.h"
const int TouchIRQPin = 2;  // "has something happened"
boolean touchStates[12]; //to keep track of the previous touch states
const int ButtonForward = 0; // touch pt 0 is forward
const int ButtonBackward = 1; // touch pt 1 is forward
int button_last = -1; // what we did last

// stepper driver drv8825, default 1/step per pulse
// max step freq: 250kHz
// min duration StepPin HIGH = 1.9microsec 
// min duration StepPin LOW = 1.9microsec
// min duration DirectionPin till StepPin = 650nsec
const int StepperStepPin = 9;
const int StepperStepHoldDuration = 2; // microsec
const int StepperDirectionPin = 8;
const int StepperDirectionPreDuration = 1; // microsec
const int StepperSteps = 200; // steps per revolution
const float StepperRPS = 0.5; // Desired revolutions-per-second when moving. Can use fractional: 1.5
const int StepperInterval = ((StepperRPS * 1000.0) / StepperSteps ); // nb, can get roundoff, probably ok
unsigned int long stepper_last_at = 0; // when we did the last step

void setup(){
  // Motor, as early as possible. Probably put some pulls on these pins
  pinMode(StepperStepPin,OUTPUT);
  pinMode(StepperDirectionPin,OUTPUT);
  // we should "home" probably?

  Serial.begin(9600);
  Serial.println("Stepper pins set");
  
  // Touch
  Serial.println("setup mpr121...");
  pinMode(TouchIRQPin, INPUT);
  digitalWrite(TouchIRQPin, HIGH); //enable pullup resistor
  Wire.begin();
  mpr121_setup();
  Serial.println("mpr121 set");

  // HC-12 Setup?
  // If we are "primary", check on "secondary"
  // if we are secondary, wait on primary

  Serial.println("Ready");
}

void loop() {
  readTouchInputs();

  // While touched, run motor
  if (touchStates[ButtonForward] == 1) {
    onestep(HIGH); // "forward"
    }
  else if (touchStates[ButtonBackward] == 1) {
    onestep(LOW); // "backwards"
    }
  // if no "touch", don't run the motor
}

void onestep(boolean whichway) {
  // run one step in the direction
  if (millis() - stepper_last_at > StepperInterval ) {
    stepper_last_at = millis();

    // FIXME: are delays between needed?
    digitalWrite(StepperDirectionPin, whichway);
    delayMicroseconds(StepperDirectionPreDuration);

    digitalWrite(StepperStepPin,HIGH);
    delayMicroseconds(StepperStepHoldDuration);

    digitalWrite(StepperStepPin,LOW);
    // no delay, because we know the StepperInterval > required hold
    }
}

void readTouchInputs(){
  if(!checkInterrupt()){
    
    //read the touch state from the MPR121
    Wire.requestFrom(0x5A,2); 
    byte LSB = Wire.read();
    byte MSB = Wire.read();
    
    uint16_t touched = ((MSB << 8) | LSB); //16bits that make up the touch states

    for (int i=0; i < 12; i++){  // Check what electrodes were pressed
      if(touched & (1<<i)){
      
        if(touchStates[i] == 0){
          //pin i was just touched
          Serial.print("pin ");
          Serial.print(i);
          Serial.println(" was just touched");
        
        }else if(touchStates[i] == 1){
          //pin i is still being touched
        }  
      
        touchStates[i] = 1;      
      }else{
        if(touchStates[i] == 1){
          Serial.print("pin ");
          Serial.print(i);
          Serial.println(" is no longer being touched");
          
          //pin i is no longer being touched
       }
        
        touchStates[i] = 0;
      }
    
    }
    
  }
}

void mpr121_setup(void){

  set_register(0x5A, ELE_CFG, 0x00); 
  
  // Section A - Controls filtering when data is > baseline.
  set_register(0x5A, MHD_R, 0x01);
  set_register(0x5A, NHD_R, 0x01);
  set_register(0x5A, NCL_R, 0x00);
  set_register(0x5A, FDL_R, 0x00);

  // Section B - Controls filtering when data is < baseline.
  set_register(0x5A, MHD_F, 0x01);
  set_register(0x5A, NHD_F, 0x01);
  set_register(0x5A, NCL_F, 0xFF);
  set_register(0x5A, FDL_F, 0x02);
  
  // Section C - Sets touch and release thresholds for each electrode
  set_register(0x5A, ELE0_T, TOU_THRESH);
  set_register(0x5A, ELE0_R, REL_THRESH);
 
  set_register(0x5A, ELE1_T, TOU_THRESH);
  set_register(0x5A, ELE1_R, REL_THRESH);
  
  set_register(0x5A, ELE2_T, TOU_THRESH);
  set_register(0x5A, ELE2_R, REL_THRESH);
  
  set_register(0x5A, ELE3_T, TOU_THRESH);
  set_register(0x5A, ELE3_R, REL_THRESH);
  
  set_register(0x5A, ELE4_T, TOU_THRESH);
  set_register(0x5A, ELE4_R, REL_THRESH);
  
  set_register(0x5A, ELE5_T, TOU_THRESH);
  set_register(0x5A, ELE5_R, REL_THRESH);
  
  set_register(0x5A, ELE6_T, TOU_THRESH);
  set_register(0x5A, ELE6_R, REL_THRESH);
  
  set_register(0x5A, ELE7_T, TOU_THRESH);
  set_register(0x5A, ELE7_R, REL_THRESH);
  
  set_register(0x5A, ELE8_T, TOU_THRESH);
  set_register(0x5A, ELE8_R, REL_THRESH);
  
  set_register(0x5A, ELE9_T, TOU_THRESH);
  set_register(0x5A, ELE9_R, REL_THRESH);
  
  set_register(0x5A, ELE10_T, TOU_THRESH);
  set_register(0x5A, ELE10_R, REL_THRESH);
  
  set_register(0x5A, ELE11_T, TOU_THRESH);
  set_register(0x5A, ELE11_R, REL_THRESH);
  
  // Section D
  // Set the Filter Configuration
  // Set ESI2
  set_register(0x5A, FIL_CFG, 0x04);
  
  // Section E
  // Electrode Configuration
  // Set ELE_CFG to 0x00 to return to standby mode
  set_register(0x5A, ELE_CFG, 0x0C);  // Enables all 12 Electrodes
  
  
  // Section F
  // Enable Auto Config and auto Reconfig
  /*set_register(0x5A, ATO_CFG0, 0x0B);
  set_register(0x5A, ATO_CFGU, 0xC9);  // USL = (Vdd-0.7)/vdd*256 = 0xC9 @3.3V   set_register(0x5A, ATO_CFGL, 0x82);  // LSL = 0.65*USL = 0x82 @3.3V
  set_register(0x5A, ATO_CFGT, 0xB5);*/  // Target = 0.9*USL = 0xB5 @3.3V
  
  set_register(0x5A, ELE_CFG, 0x0C);
  
}

boolean checkInterrupt(void){
  return digitalRead(TouchIRQPin);
}

void set_register(int address, unsigned char r, unsigned char v){
    Wire.beginTransmission(address);
    Wire.write(r);
    Wire.write(v);
    Wire.endTransmission();
}
