#include "Wire.h"

// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"
#include "BMP280.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>  // Lägg till detta bibliotek för att hantera JSON
#include <HMC5883L.h>

const char* ssid = "SSIS_IOT";
const char* password = "hRBjs7Ye";

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev I2C_M;

uint8_t buffer_m[6];


int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float heading;
float tiltheading;
float vector_to_destination[2] = {0, 0};
float normal_vector[2] = {0, 0};
float angle_error = 0;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];
float Destination[2] = {36.0, 10.6};

#define sample_num_mdate 5000

#define ENA_PIN 32  // The ESP32 pin GPIO32 connected to the ENA pin L298N
#define IN1_PIN 15  // The ESP32 pin GPIO15 connected to the IN1 pin L298N
#define IN2_PIN 33  // The ESP32 pin GPIO33 connected to the IN2 pin L298N
#define IN3_PIN 27  // The ESP32 pin GPIO27 connected to the IN3 pin L298N
#define IN4_PIN 12  // The ESP32 pin GPIO12 connected to the IN4 pin L298N
#define ENB_PIN 13  // The ESP32 pin GPIO13 connected to the ENB pin L298N
#define SENS_R_PIN 14
#define SENS_L_PIN 4

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

float temperature;
float pressure;
float atm;
float altitude;
BMP280 bmp280;

void setup() {
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(SENS_R_PIN, INPUT);
  pinMode(SENS_L_PIN, INPUT);

  digitalWrite(ENA_PIN, 1);
  digitalWrite(ENB_PIN, 1);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // initialize device. Om den fastnar koppla 10 dof bättre
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  bmp280.init();

  // See datasheet
  //bmp280.setSampling(BMP280::MODE_NORMAL,  // mode
  //              BMP280::SAMPLING_X2,  // temperature, not more than x2
  //              BMP280::SAMPLING_X16, // pressure
  //              BMP280::FILTER_X16,   // filter
  //              BMP280::STANDBY_MS_500); // standby

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

  delay(1000);
  Serial.println("     ");

  //  Mxyz_init_calibrated ();
}

void loop() {
  getAccel_Data();
  getGyro_Data();
  getCompassDate_calibrated();  // compass data has been calibrated here
  getHeading();                 //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
  getTiltHeading();

  Serial.println("calibration parameter: ");
  Serial.print(mx_centre);
  Serial.print("         ");
  Serial.print(my_centre);
  Serial.print("         ");
  Serial.println(mz_centre);
  Serial.println("     ");

  Serial.println("Acceleration(g) of X,Y,Z:");
  Serial.print(Axyz[0]);
  Serial.print(",");
  Serial.print(Axyz[1]);
  Serial.print(",");
  Serial.println(Axyz[2]);
  Serial.println("Gyro(degress/s) of X,Y,Z:");
  Serial.print(Gxyz[0]);
  Serial.print(",");
  Serial.print(Gxyz[1]);
  Serial.print(",");
  Serial.println(Gxyz[2]);
  Serial.println("Compass Value of X,Y,Z:");
  Serial.print(Mxyz[0]);
  Serial.print(",");
  Serial.print(Mxyz[1]);
  Serial.print(",");
  Serial.println(Mxyz[2]);
  Serial.println("The clockwise angle between the magnetic north and X-Axis:");
  Serial.print(heading);
  Serial.println(" ");
  Serial.println("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:");
  Serial.println(tiltheading);
  Serial.println("   ");
  /*
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Ange URL:en som du vill hämta data från
    String url = "https://track.ssis.nu/last/T37";

    http.begin(url);
    int httpCode = http.GET();

    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();

        // Hantera JSON
        DynamicJsonDocument jsonDoc(1024);
        deserializeJson(jsonDoc, payload);

        float x_value = jsonDoc[0]["x"];
        float y_value = jsonDoc[0]["y"];

        Serial.print("x-värde: ");
        Serial.println(x_value, 2);  // Skriv ut med 2 decimaler

        Serial.print("y-värde: ");
        Serial.println(y_value, 2);  // Skriv ut med 2 decimaler
        
        
      }
    } else {
      Serial.println("Fel vid HTTP-förfrågan");
    }

    http.end();
  }

  delay(2000);  // Vänta en stund innan nästa förfrågan
  */
  get_vector_to_destination(-13, 27, Destination[0], Destination[1]);
  get_normal_vector();
  Serial.print("x-to-dest: ");
  Serial.println(vector_to_destination[0]);
  Serial.print("y-to-dest: ");
  Serial.println(vector_to_destination[1]);
  Serial.print("normal vector:");
  Serial.print(normal_vector[0]);
  Serial.println(normal_vector[1]);
  get_angle_error();
  Serial.print("angle-error: ");
  Serial.println(angle_error);
}


void getHeading(void) {
  heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}

void getTiltHeading(void) {
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1] / cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh) / PI;
  if (yh < 0) tiltheading += 360;
}

void Mxyz_init_calibrated() {
  while (!Serial.find("ready"))
    ;

  get_calibration_Data();
}

void get_calibration_Data() {
  for (int i = 0; i < sample_num_mdate; i++) {
    get_one_sample_date_mxyz();
    /*
        Serial.print(mx_sample[2]);
        Serial.print(" ");
        Serial.print(my_sample[2]);                            //you can see the sample data here .
        Serial.print(" ");
        Serial.println(mz_sample[2]);
        */

    if (mx_sample[2] >= mx_sample[1]) mx_sample[1] = mx_sample[2];
    if (my_sample[2] >= my_sample[1]) my_sample[1] = my_sample[2];  //find max value
    if (mz_sample[2] >= mz_sample[1]) mz_sample[1] = mz_sample[2];

    if (mx_sample[2] <= mx_sample[0]) mx_sample[0] = mx_sample[2];
    if (my_sample[2] <= my_sample[0]) my_sample[0] = my_sample[2];  //find min value
    if (mz_sample[2] <= mz_sample[0]) mz_sample[0] = mz_sample[2];
  }

  mx_max = mx_sample[1];
  my_max = my_sample[1];
  mz_max = mz_sample[1];

  mx_min = mx_sample[0];
  my_min = my_sample[0];
  mz_min = mz_sample[0];

  mx_centre = (mx_max + mx_min) / 2;
  my_centre = (my_max + my_min) / 2;
  mz_centre = (mz_max + mz_min) / 2;
}

void get_one_sample_date_mxyz() {
  getCompass_Data();
  mx_sample[2] = Mxyz[0];
  my_sample[2] = Mxyz[1];
  mz_sample[2] = Mxyz[2];
}

void getAccel_Data(void) {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double)ax / 16384;
  Axyz[1] = (double)ay / 16384;
  Axyz[2] = (double)az / 16384;
}

void getGyro_Data(void) {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double)gx * 250 / 32768;
  Gxyz[1] = (double)gy * 250 / 32768;
  Gxyz[2] = (double)gz * 250 / 32768;
}

void getCompass_Data(void) {
  I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01);  //enable the magnetometer
  delay(10);
  I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

  mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0];
  my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2];
  mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4];

  Mxyz[0] = (double)mx * 1200 / 4096;
  Mxyz[1] = (double)my * 1200 / 4096;
  Mxyz[2] = (double)mz * 1200 / 4096;
}

void getCompassDate_calibrated() {
  getCompass_Data();
  Mxyz[0] = Mxyz[0] - mx_centre;
  Mxyz[1] = Mxyz[1] - my_centre;
  Mxyz[2] = Mxyz[2] - mz_centre;
}

void get_vector_to_destination(float x1, float y1, float x2, float y2) {
  vector_to_destination[0] = x2 - x1;
  vector_to_destination[1] = y2 - y1;
  float length = sqrt(vector_to_destination[0] * vector_to_destination[0] + vector_to_destination[1] * vector_to_destination[1]);
  vector_to_destination[0] = vector_to_destination[0] / length;
  vector_to_destination[1] = vector_to_destination[1] / length;
}

void get_normal_vector() {
  normal_vector[0] = cos(heading * 0.01745329);
  normal_vector[1] = sin(heading * 0.01745329);
}

void get_angle_error() {
  angle_error = atan(vector_to_destination[0] / vector_to_destination[1]) - atan(normal_vector[0] / normal_vector[1]) + 1.7;
}

void forward() {
  digitalWrite(IN1_PIN, LOW); // RIGHT REV
  digitalWrite(IN2_PIN, HIGH); // RIGHT FOR
  digitalWrite(IN3_PIN, LOW); // LEFT REV
  digitalWrite(IN4_PIN, HIGH); // LEFT FOR
}

void backward() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

void right() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

void left() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}