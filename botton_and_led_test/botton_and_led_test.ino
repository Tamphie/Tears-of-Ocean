#include <ESP32Encoder.h>
#include "APA102.h"

#define FORCE_SENSOR_PIN 34 //fsr
#define BTN  32 // declare the button ED pin number
#define BIN_1 4 //motor ENA
#define MOTORIN1 25
#define MOTORIN2 26

#define VELOCITY 15


enum state{
  Idling,
  Spindrift,
  Ringdown
};
byte state = Idling;

//led strip
const uint8_t dataPin = 22;
const uint8_t clockPin = 23;
// Create an object for writing to the LED strip.
APA102<dataPin, clockPin> ledStrip;
// Set the number of LEDs to control.
const uint16_t ledCount = 60;
// Create a buffer for holding the colors (3 bytes per color).
rgb_color colors[ledCount];
// Set the brightness to use (the maximum is 31).
int brightness = 1;

//button
volatile bool buttonIsPressed = false;
volatile bool debounce = false;
//int state = 1;


ESP32Encoder encoder;
int ForceReading = 0;
int theta = 0;
int thetaDes = 0;
int thetaMax = 3400;     // 75.8 * 6 counts per revolution
int D = 0;
int error = 0;
int errorP = 0;
int sumError = 0;
float sumErrorKi = 0;
int Vdes = 0;

int Kp = 20;   // TUNE THESE VALUES TO CHANGE CONTROLLER PERFORMANCE
int Ki = 5;
int KiMax = 0;

//Setup interrupt variables ----------------------------
volatile int count = 0; // encoder count
//volatile bool interruptCounter = false;    // check timer interrupt 1
volatile bool deltaT = false;     // check timer interrupt 2
volatile bool ForceIsDetected = false;
//int totalInterrupts = 0;   // counts the number of triggering of the alarm
hw_timer_t * timer0 = NULL;
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;

// setting PWM properties ----------------------------
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int resolution = 8;
const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 150;

//Initialization ------------------------------------
void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  debounce = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
  timerStop(timer0);
}

//Initialization ------------------------------------
void IRAM_ATTR isr() {  // the function to be called when interrupt is triggered
 timerStart(timer0);
}

void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  count = encoder.getCount( );
  encoder.clearCount ( );
  deltaT = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}


void setup() {
  // put your setup code here, to run once:
  pinMode(BTN, INPUT);
  pinMode(MOTORIN1,OUTPUT);
  pinMode(MOTORIN2,OUTPUT);
  pinMode(FORCE_SENSOR_PIN,INPUT);
  attachInterrupt(BTN, isr, RISING);

  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachFullQuad(33, 27); // Attache pins for use as encoder pins
  encoder.setCount(0);  // set starting count value after attaching

  // configure LED PWM functionalitites
  ledcSetup(ledChannel_1, freq, resolution);
//ledcSetup(ledChannel_2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(BIN_1, ledChannel_1);
  //ledcAttachPin(BIN_2, ledChannel_2);
  
  timer0 = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered 
  timerAlarmWrite(timer0, 200000, true); 

  timer1 = timerBegin(1, 80, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered
  timerAlarmWrite(timer1, 5000, true); // 10000 * 1 us = 10 ms, autoreload true

  // at least enable the timer alarms
  timerAlarmEnable(timer0); // enable
  timerAlarmEnable(timer1); // enable


}
rgb_color hsvToRgb(uint16_t h, uint8_t s, uint8_t v)
{
    uint8_t f = (h % 60) * 255 / 60;
    uint8_t p = (255 - s) * (uint16_t)v / 255;
    uint8_t q = (255 - f * (uint16_t)s / 255) * (uint16_t)v / 255;
    uint8_t t = (255 - (255 - f) * (uint16_t)s / 255) * (uint16_t)v / 255;
    uint8_t r = 0, g = 0, b = 0;
    switch((h / 60) % 6){
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        case 5: r = v; g = p; b = q; break;
    }
    return rgb_color(r, g, b);
}
  
void loop() {
   //plotControlData();
   Serial.print(debounce);
 switch (state) {
    case Idling:
      if (CheckForButtonPress() == true) { 
        
        ledStrip_on();
        state = Spindrift;
      }
      break;
  case Spindrift: 
      if(analogRead(FORCE_SENSOR_PIN) >4095 ||CheckForButtonPress() == true){
        ledStrip_off();
        state = Idling;
      }
     
      break;
      
 
 
 }

plotControlData();
}
//Other functions

void plotControlData() {
 // Serial.print("state:");
//  Serial.print(state);
 /* Serial.print("Position:");
  Serial.print(theta);
  Serial.print(" ");
  Serial.print("Desired_Position:");  
  Serial.print(thetaDes);
  Serial.print(" ");
  Serial.print("D:");
  Serial.println(D);*/
  //delay(10);
  //Serial.print("PWM_Duty:");
 // Serial.println(D);
 // Serial.print("The force sensor value = ");
 // Serial.print(ForceReading); // print the raw analog reading
/*
  if (ForceReading < 10)       // from 0 to 9
    Serial.println(" -> no pressure");
  else if (ForceReading < 200) // from 10 to 199
    Serial.println(" -> light touch");
  else if (ForceReading < 500) // from 200 to 499
    Serial.println(" -> light squeeze");
  else if (ForceReading < 800) // from 500 to 799
    Serial.println(" -> medium squeeze");
  else // from 800 to 1023
    Serial.println(" -> big squeeze");
*/
  
}


void ledStrip_on(){
        
  uint8_t time = millis() >> 4;

  for(uint16_t i = 0; i < ledCount; i++)
  {
    uint8_t p = time - i * 8;
    colors[i] = hsvToRgb((uint32_t)p * 359 / 256, 255, 255);
  }
  ForceReading = analogRead(FORCE_SENSOR_PIN);
  brightness = map(ForceReading, 0, 4095, 0, 31);
  ledStrip.write(colors, ledCount, brightness);
Serial.print("brightness:");
Serial.print(brightness);
  delay(500);
}



void ledStrip_off(){
   ledStrip.write(colors, ledCount, 0);
}





bool CheckForButtonPress(){
  if(debounce){
    portENTER_CRITICAL(&timerMux0);
      debounce = false;
    portEXIT_CRITICAL(&timerMux0);
    return true;
  }
  else return false;
}
