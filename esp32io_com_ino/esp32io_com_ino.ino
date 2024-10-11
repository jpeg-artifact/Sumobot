#define ENA_PIN 32  // The ESP32 pin GPIO32 connected to the ENA pin L298N
#define IN1_PIN 15  // The ESP32 pin GPIO15 connected to the IN1 pin L298N
#define IN2_PIN 33  // The ESP32 pin GPIO33 connected to the IN2 pin L298N
#define IN3_PIN 27  // The ESP32 pin GPIO27 connected to the IN3 pin L298N
#define IN4_PIN 12  // The ESP32 pin GPIO12 connected to the IN4 pin L298N
#define ENB_PIN 13  // The ESP32 pin GPIO13 connected to the ENB pin L298N
#define SENS_R_PIN 14
#define SENS_L_PIN 22

void setup(){
  Serial.begin(9600);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(SENS_R_PIN, INPUT);
  pinMode(SENS_L_PIN, INPUT);
}

void loop(){
  int SENS_R_value = analogRead(SENS_R_PIN);
  int SENS_L_value = analogRead(SENS_L_PIN);


}
