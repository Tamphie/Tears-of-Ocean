#include <ESP32Encoder.h>
#define MOTORIN1 25
#define MOTORIN2 26

#define BIN_1 4 //motor ENA
const int freq = 5000;
const int ledChannel_1 = 1;

const int resolution = 8;
const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 150;

ESP32Encoder encoder;
int theta = 0;
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


void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  count = encoder.getCount( );
  encoder.clearCount ( );
  deltaT = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}


void setup() {
  // put your setup code here, to run once:
  pinMode(MOTORIN1,OUTPUT);
  pinMode(MOTORIN2,OUTPUT);
 ledcSetup(ledChannel_1, freq, resolution);
 ledcAttachPin(BIN_1, ledChannel_1);
  digitalWrite(MOTORIN1,LOW);
  digitalWrite(MOTORIN2,HIGH);
  ledcWrite(ledChannel_1, NOM_PWM_VOLTAGE);


  Serial.begin(115200);
  ESP32Encoder::useInternalWeakPullResistors = UP; // Enable the weak pull up resistors
  encoder.attachFullQuad(33, 27); // Attache pins for use as encoder pins
  encoder.setCount(0);  // set starting count value after attaching
  
  timer1 = timerBegin(1, 80, true);  // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered
  timerAlarmWrite(timer1, 10000, true); // 10000 * 1 us = 10 ms, autoreload true

  timerAlarmEnable(timer1); // enable
  
}

void loop() {
   if (deltaT) {
      portENTER_CRITICAL(&timerMux1);
      deltaT = false;
      portEXIT_CRITICAL(&timerMux1);
      theta += count;
      if(theta>3840){
        digitalWrite(MOTORIN2,LOW);
        digitalWrite(MOTORIN1,HIGH);
        ledcWrite(ledChannel_1, NOM_PWM_VOLTAGE);
      }
      if(theta<100){
        digitalWrite(MOTORIN1,LOW);
        digitalWrite(MOTORIN2,HIGH);
        ledcWrite(ledChannel_1, NOM_PWM_VOLTAGE);
      }
      Serial.print("count");
      Serial.print(count);
      Serial.print(" ");
      Serial.print("thata");
      Serial.println(theta);
      //delay(1000);
          //
   }
}
