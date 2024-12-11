
#define FORCE_SENSOR_PIN 34 //fsr
int ForceReading = 0;
void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(FORCE_SENSOR_PIN, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  ForceReading = analogRead(FORCE_SENSOR_PIN);
  Serial.print("ForceReading:");
  Serial.print(ForceReading);
  Serial.print(" ");
}
