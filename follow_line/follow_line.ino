#define ENA_PIN 32  // The ESP32 pin GPIO32 connected to the ENA pin L298N
#define IN1_PIN 15  // The ESP32 pin GPIO15 connected to the IN1 pin L298N
#define IN2_PIN 33  // The ESP32 pin GPIO33 connected to the IN2 pin L298N
#define IN3_PIN 27  // The ESP32 pin GPIO27 connected to the IN3 pin L298N
#define IN4_PIN 12  // The ESP32 pin GPIO12 connected to the IN4 pin L298N
#define ENB_PIN 13  // The ESP32 pin GPIO13 connected to the ENB pin L298N
#define SENS_R_PIN 14
#define SENS_L_PIN 4

int turnSpeed = 180; // Hur snabbt däcket snurrar när den svänger. Värde mellan 0 och 255.
int forwardSpeed = 140; // Hur snabbt däcken snurrar när den åker framåt. Värde mellan 0 och 255.
unsigned long wheelLockThreshold = 25; // Hur lång tid sensorn behöver känna av linjen innan däcket låser sig.
int turnTime = 150; // Hur länge däcket är låst

unsigned long lastTime = 0; // Används för att räkna ut delta time.
unsigned long timeSpentTurning = 0; // Håller reda på hur länge bilen har svängt under en svängning.

int turnThreshold = 4095; // Värdet som sensorns avläsnings värde behöver gå under för att svänga. !!DETTA VÄRDE SÄTTS AUTOMATISKTS I SETUP!!

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
  turnThreshold = ((analogRead(SENS_R_PIN) + analogRead(SENS_L_PIN)) / 2) * 0.7; // Läser av sensorerna och tar deras medelvärde, sedan multiplicerar
  //Serial.println(turnThreshold);
}

void loop(){
  int SENS_R_value = analogRead(SENS_R_PIN); // Värdet avläst från högra sensorn
  int SENS_L_value = analogRead(SENS_L_PIN); // Värdet avläst från vänstra sensorn 
  bool right = SENS_R_value < turnThreshold; // En check i fall högra sensorns värde är lägre än turnThreshold
  bool left = SENS_L_value < turnThreshold; // En check i fall vänstra sensorns värde är lägre än turnThreshold
  unsigned long deltaTime = millis() - lastTime; // Delta time beräkningar
  lastTime = millis();

  //Serial.println(SENS_L_value);
  //Serial.println(SENS_R_value);

  // Sätt hastighet på motorerna
  analogWrite(ENA_PIN, forwardSpeed); // RIGHT
  analogWrite(ENB_PIN, forwardSpeed); // LEFT
  
  // Åka framåt
  if (right && left) {
    timeSpentTurning = 0;
    digitalWrite(IN1_PIN, LOW); // RIGHT REV
    digitalWrite(IN2_PIN, HIGH); // RIGHT FOR
    digitalWrite(IN3_PIN, LOW); // LEFT REV
    digitalWrite(IN4_PIN, HIGH); // LEFT FOR
  } // Åka bakåt
  else if (!right && !left) {
    timeSpentTurning = 0;
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    
  } // Svänga höger
  else if (!right && left) { 
    timeSpentTurning += deltaTime;
    //Serial.println(timeSpentTurning);
    analogWrite(ENA_PIN, turnSpeed);
    analogWrite(ENB_PIN, turnSpeed);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);

    if (timeSpentTurning > wheelLockThreshold) { // Lås däck om den har svängt länge
      digitalWrite(IN2_PIN, HIGH);
      delay(turnTime);
    }
  } // Svänga vänster
  else if (right && !left) {
    timeSpentTurning += deltaTime;
    //Serial.println(timeSpentTurning);
    analogWrite(ENA_PIN, turnSpeed);
    analogWrite(ENB_PIN, turnSpeed);
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    
    if (timeSpentTurning > wheelLockThreshold) { // Lås däck om den har svängt länge
      digitalWrite(IN4_PIN, HIGH);
      delay(turnTime);
    }
  }
}
