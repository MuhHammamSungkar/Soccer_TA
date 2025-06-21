#include <SparkFun_TB6612.h>

// Pin definitions for motors
#define AIN1 7
#define AIN2 8
#define PWMA 9
#define BIN1 5
#define BIN2 4
#define PWMB 3
#define STBY_1 6

#define CIN1 11
#define CIN2 12
#define PWMC 13
#define STBY_2 10

// Motor offsets
const int offsetA = 1;
const int offsetB = 1;
const int offsetC = -1;

// Initialize motors
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY_1);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY_1);
Motor motor3 = Motor(CIN1, CIN2, PWMC, offsetC, STBY_2);

// Constants
const float R = 100.0; // Jarak roda ke pusat robot (mm), sesuaikan dengan robot Anda
const float alpha1 = -PI / 6;  // Sudut motor 1 (30 derajat)
const float alpha2 = PI / 6; // Sudut motor 2 (-30 derajat)
const float alpha3 = -PI / 2;  // Sudut motor 3 (90 derajat)
const float psi = 0.0;        // Orientasi robot awal (0 derajat)

void moveToCoordinate(float x, float y, float omega) {
  // Matriks transformasi inverse kinematics
  float t11 = -sin(psi + alpha1);
  float t12 = cos(psi + alpha1);
  float t13 = R;
  float t21 = -sin(psi + alpha2);
  float t22 = cos(psi + alpha2);
  float t23 = R;
  float t31 = -sin(psi + alpha3);
  float t32 = cos(psi + alpha3);
  float t33 = R;

  // Hitung kecepatan roda
  float v1 = t11 * x + t12 * y + t13 * omega;
  float v2 = t21 * x + t22 * y + t23 * omega;
  float v3 = t31 * x + t32 * y + t33 * omega;

  //Normalisasi kecepatan ke rentang -255 sampai 255
  float maxSpeed = max(max(abs(v1), abs(v2)), abs(v3));
  if (maxSpeed > 255) {
    v1 = (v1 / maxSpeed) * 255;
    v2 = (v2 / maxSpeed) * 255;
    v3 = (v3 / maxSpeed) * 255;
  }

  // Terapkan kecepatan ke motor
  motor1.drive(v1);
  motor2.drive(v2);
  motor3.drive(v3);

  // Debug output
  Serial.print("Moving to (x, y, omega): (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(omega);
  Serial.println(")");
  Serial.print("Motor speeds: ");
  Serial.print(v1);
  Serial.print(", ");
  Serial.print(v2);
  Serial.print(", ");
  Serial.println(v3);
}

//konfigurasi speed motor
// maju = m1, m2 (+)
// mundur = m1, m2 (-)
// maju serong kanan (30,50) = (58, 28, 30)
// maju serong kiri (-30, 50) = (28, 58, -30)
// geser kiri (-90,0) = (-45, 45 ,-90)
// geser kanan (90,0) = (45, -45, 90)
// mundur serong kiri (-50, -90) = (-102, -52, -50)
// mundur serong kanan (50, -90) = (-52, -102, 50)

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Serial.println("Testing movement: Forward and right (50, 70, 0)");
  // moveToCoordinate(50, 70, 0); // Maju dan serong kanan tanpa rotasi
  // delay(2000);

  // Serial.println("Testing movement: Stop");
  // motor1.brake();
  // motor2.brake();
  // motor3.brake();
  // delay(2000);

  Serial.println("Testing movement: Backward and left (50, -90, 0)");
  moveToCoordinate(50, -90, 0); // Mundur dan serong kiri
  delay(2000);

  Serial.println("Testing movement: Stop");
  motor1.brake();
  motor2.brake();
  motor3.brake();
  delay(2000);
}