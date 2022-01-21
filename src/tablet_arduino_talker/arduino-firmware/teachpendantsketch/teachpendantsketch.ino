#define DEADMAN_PIN 3
#define E_STOP_PIN 4 

void setup() {
   pinMode(DEADMAN_PIN, INPUT_PULLUP);
   pinMode(E_STOP_PIN, INPUT_PULLUP);
   Serial.begin(115200);

}

void loop() {
  delay(20);
  Serial.print( 2*digitalRead(E_STOP_PIN) + 1*digitalRead(DEADMAN_PIN));
}
