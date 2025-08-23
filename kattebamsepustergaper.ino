#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo kanaler
#define SERVO0 0
#define SERVO1 1
#define SERVO2 2

// Joystick analogpinne
#define JOY_X A0

// Servo pulsområde (PCA9685)
#define SERVOMIN 150
#define SERVOMAX 600
#define MID_POS 90

// --- Servo 0 variabler ---
int currentAngle0 = MID_POS;

// --- Servo 1 og 2 variabler ---
float servo1Angle = 0;
float servo2Angle = 0;
bool servoForward = true;
unsigned long servoStartTime = 0;
const unsigned long servoDuration = 1000; // 1 sekund frem og tilbake

// Tidsvariabler for Servo 0
unsigned long lastUpdate0 = 0;
const unsigned long updateInterval0 = 10; // ms

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  servoStartTime = millis();
}

void loop() {
  unsigned long now = millis();

  // --- Servo 0: Joystickstyrt ---
  if (now - lastUpdate0 > updateInterval0) {
    lastUpdate0 = now;

    int joyX = analogRead(JOY_X);
    int deadzone = 50; // liten dødsone
    int targetAngle0 = currentAngle0; // standard: behold posisjon

    // Bare oppdater hvis joystick utenfor dødsone
    if (joyX < 512 - deadzone) {
      targetAngle0 = map(joyX, 0, 512, 0, MID_POS);
    } else if (joyX > 512 + deadzone) {
      targetAngle0 = map(joyX, 512, 1023, MID_POS, 180);
    }

    // Smooth movement mot targetAngle0
    if (currentAngle0 < targetAngle0) currentAngle0++;
    else if (currentAngle0 > targetAngle0) currentAngle0--;

    pwm.setPWM(SERVO0, 0, angleToPulse(currentAngle0));
  }

  // --- Servo 1 og 2: lineær interpolasjon ---
  float t = (now - servoStartTime) / (float)servoDuration; // 0..1
  if (t >= 1.0) {
    t = 0;
    servoForward = !servoForward; // snu retning
    servoStartTime = now;
  }

  if (servoForward) {
    servo1Angle = 0 + t * 180;
    servo2Angle = 0 - t * 180;
  } else {
    servo1Angle = 180 - t * 180;
    servo2Angle = -180 + t * 180;
  }

  pwm.setPWM(SERVO1, 0, angleToPulse((int)servo1Angle));
  pwm.setPWM(SERVO2, 0, angleToPulse((int)(servo2Angle + 180))); // -180..0 til 0..180
}
