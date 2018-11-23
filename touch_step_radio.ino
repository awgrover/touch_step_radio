/*
  Behavior:
  
    status leds show heartbeat. nb, onboard led is pin 11, shows sending to hc-12
    while touching 1 goes forward
    while touching 2 goes backwards
    sends same info to 2nd arduino
    also listens on radio for commands to go forward/backward: code for both is same

  Uses
    a capacitance touch board mpr121: 2,A5,A4
    a stepper driver DRV8825: 8,9
    https://www.pololu.com/product/2133/resources
    https://www.pololu.com/file/0J590/drv8825.pdf
    and a radio HC-12: 10,11
    status leds: 3,4,5
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

// radio hc-12
#include <SoftwareSerial.h>
const byte HC12RxdPin = 4;                  // Recieve Pin on HC12
const byte HC12TxdPin = 5;                  // Transmit Pin on HC12
SoftwareSerial HC12(HC12TxdPin,HC12RxdPin); // Create Software Serial Port

boolean we_are_primary = false; // different behavior for primary/secondary
int Heartbeat=1000; // timeout for heartbeat
unsigned long heartbeat_at = 0; // the last time we saw radio from someone

// Status LEDs
int ReadyLED = 3; // on when we finish setting up
int HeartbeatLED = 4; // on if heartbeat was seen before 'Heartbeat' timeout

void setup(){
  pinMode(ReadyLED, OUTPUT); digitalWrite(ReadyLED, LOW);

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
  Serial.println("mpr121 set: we are primary");
  we_are_primary = true;

  // HC-12
  HC12.begin(9600); // slow, but ok
  Serial.println("Radio set");
  // If we are "primary", check on "secondary"
  // if we are secondary, wait on primary

  Serial.println("Ready");

  // Status LEDs
  pinMode(HeartbeatLED, OUTPUT); digitalWrite(HeartbeatLED, LOW);
  pinMode(ReadyLED, OUTPUT); digitalWrite(ReadyLED, HIGH);
}

void loop() {
  static int current_touch = 0; // 0==off, 0==forward, 1==backwards

  // "primary" monitors the mpr121 & sends commands
  if (we_are_primary) {
    int new_touch = readTouchInputs();
    if (new_touch != -1) { // -1 means no change
      current_touch = new_touch;
      Serial.print("Touch ");Serial.println(current_touch);
      send_touch( new_touch ); // 0==off, 1==forward, 2==backwards
      }

    receive_command(); // really just for heartbeat, don't care about value
    }

  // secondary receives commands
  else { // secondary
    int new_touch = receive_command();
    if (new_touch != -1 ) { // change?
      current_touch = new_touch;
      Serial.print("In ");Serial.println(current_touch);
    }
  }

  // While touched (i.e. current), run motor
  if (current_touch == 1) {
    onestep(HIGH); // "forward"
    }
  else if (current_touch == 2) {
    onestep(LOW); // "backwards"
    }
  // if no "touch", don't run the motor

  heartbeat_check(); // just watch
}

void send_touch(int new_touch) {
  // only when something has changed, so send it
  HC12.print(new_touch); // in ascii
}

int receive_command() {
  // read a command from the other arduino
  // update heartbeat_at
  // return -1==nothing, 0=off, 1=forward, 2=backward

  int result = -1; // none

  while (HC12.available() > 0) {
    char incoming = HC12.read();

    switch ( incoming ) {

      // Touch from other, in ascii
      case '0':
      case '1':
      case '2':
        result = '0' - incoming; // so numeric 0,1,2
        break;

      // Heartbeat from other
      case 'B':
        // doesn't change the "last" result
        if (heartbeat_at == 0) { // 1st time
          Serial.println("Radio from other worked");
          }
        heartbeat_at = millis(); // saw something
        digitalWrite(HeartbeatLED, HIGH);
        break;
    }

  return result;
  }
}
    
void heartbeat_check() {
  // is the radio (still) working?

  static unsigned long last_send = 0;

  // Say "I'm here" periodically
  if (millis() - last_send > (Heartbeat-100)) { // 100msec before heartbeat timeout
    last_send = millis();
    HC12.print('B');
  }

  // Flatline if no heartbeat
  if (millis() - heartbeat_at > Heartbeat) {
    digitalWrite(HeartbeatLED, LOW);
    Serial.println("Flatline");
  }
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

int readTouchInputs(){
  // The new touch input, so >-1 only when something changes
  // NB: -1==nochange, 0==nothing, 1==first, 2==second
  // So we can tell if anything is changed/touched

  int new_touch = -1; // if nothing changes it

  if(!checkInterrupt()){
    
    //read the touch state from the MPR121
    Wire.requestFrom(0x5A,2); 
    byte LSB = Wire.read();
    byte MSB = Wire.read();
    
    uint16_t touched = ((MSB << 8) | LSB); //16bits that make up the touch states

    for (int i=0; i < 12; i++) {  // Check what electrodes were pressed
      if(touched & (1<<i)) {
      
        if(touchStates[i] == 0) {
          //pin i was just touched: change
          Serial.print("pin ");
          Serial.print(i);
          Serial.println(" was just touched");
          new_touch = i + 1; // 1==first touch point
        
        }else if(touchStates[i] == 1) {
          //pin i is still being touched
        }  
      
        touchStates[i] = 1;      
      } else {
        if(touchStates[i] == 1) {
          Serial.print("pin ");
          Serial.print(i);
          Serial.println(" is no longer being touched");
          
          //pin i is no longer being touched
          new_touch = 0; // untouched, could get overwrote if multiple touches change
        }
        
        touchStates[i] = 0;
      }
    
    } // end loop
    
  }

  return new_touch;
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
