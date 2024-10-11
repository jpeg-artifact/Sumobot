#define ENA_PIN 32  // The ESP32 pin GPIO32 connected to the ENA pin L298N
#define IN1_PIN 15  // The ESP32 pin GPIO15 connected to the IN1 pin L298N
#define IN2_PIN 33  // The ESP32 pin GPIO33 connected to the IN2 pin L298N
#define IN3_PIN 27  // The ESP32 pin GPIO27 connected to the IN3 pin L298N
#define IN4_PIN 12  // The ESP32 pin GPIO12 connected to the IN4 pin L298N
#define ENB_PIN 13  // The ESP32 pin GPIO13 connected to the ENB pin L298N
#define SENS_R_PIN 14
#define SENS_L_PIN 4

int turnSpeed = 130;
int forwardSpeed = 130;
float turnMultiplier = 0.5;
unsigned long wheelLockThreshold = 250;

unsigned long lastTime = 0;
unsigned long timeSpentTurning = 0;

int turnThreshold = 4095;

void setup(){
  //Serial.begin(9600);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(SENS_R_PIN, INPUT);
  pinMode(SENS_L_PIN, INPUT);

  analogWrite(ENA_PIN, forwardSpeed);
  analogWrite(ENB_PIN, forwardSpeed);

  delay(5000);
  turnThreshold = ((analogRead(SENS_R_PIN) + analogRead(SENS_L_PIN)) / 2) * 0.8;
  //Serial.println(turnThreshold);
}

void loop(){
  int SENS_R_value = analogRead(SENS_R_PIN);
  int SENS_L_value = analogRead(SENS_L_PIN);
  bool right = SENS_R_value < turnThreshold;
  bool left = SENS_L_value < turnThreshold;
  unsigned long deltaTime = millis() - lastTime;
  lastTime = millis();

  //Serial.println(SENS_L_value);
  //Serial.println(SENS_R_value);

  analogWrite(ENA_PIN, forwardSpeed); // RIGHT
  analogWrite(ENB_PIN, forwardSpeed); // LEFT
  
  // Forward
  if (right && left) {
    timeSpentTurning = 0;
    digitalWrite(IN1_PIN, LOW); // RIGHT REV
    digitalWrite(IN2_PIN, HIGH); // RIGHT FOR
    digitalWrite(IN3_PIN, LOW); // LEFT REV
    digitalWrite(IN4_PIN, HIGH); // LEFT FOR
  }
  else if (!right && !left) {
    timeSpentTurning = 0;
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    
  }
  else if (!right && left) { // Turn right
    timeSpentTurning += deltaTime;
    analogWrite(ENA_PIN, turnSpeed * turnMultiplier);
    analogWrite(ENB_PIN, turnSpeed);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    /*if (timeSpentTurning > wheelLockThreshold) { // Locks wheel if it's turning for long time
      digitalWrite(IN1_PIN, LOW);
    }
    else {
      digitalWrite(IN1_PIN, HIGH);
    }*/
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  }
  else if (right && !left) { // Turn left
    timeSpentTurning += deltaTime;
    analogWrite(ENA_PIN, turnSpeed);
    analogWrite(ENB_PIN, turnSpeed * turnMultiplier);
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    /*if (timeSpentTurning > wheelLockThreshold) { // Locks wheel if it's turning for long time
      digitalWrite(IN3_PIN, LOW);
    }
    else {
      digitalWrite(IN3_PIN, HIGH); // Left fov
    }*/
  }
}
