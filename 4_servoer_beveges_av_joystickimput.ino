//readme:

//Koblinger:

//Komponent: 			Arduino Uno R4 pin:
//Joystick VRx 			A0
//Joystick SW 			D2
//Joystick GND/VCC 		GND/5V
//PCA9685 VCC			5V
//PCA9685 GND			GND
//PCA9685 SDA/SCL			SDA/SCL (I2C, vanligvis A4/A5 eller pin 20/21)



//✅ Oppførsel:

//Trykk knapp:

//Servo 0 → høyre.

//Servo 1 → venstre.

//Begge går tilbake når du slipper.


//Skyv joystick:

//Mot høyre: servo 2 → høyre.

//Mot venstre: servo 3 → venstre.

//Begge går tilbake når du slipper joystick.


//koden:

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialiser servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo-grenser (avhenger av din servo)
#define SERVO_MIN 150
#define SERVO_MID 375
#define SERVO_MAX 600

// Joystick-pins
const int joyXPin = A0;      // VRx
const int joyButtonPin = 2;  // SW

// Grenser for X-bevegelse
const int threshold = 100; // død-sone

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);  // Standard for servoer

  pinMode(joyButtonPin, INPUT_PULLUP);  // Joystick-knapp aktiv lav
}

void loop() {
  int joyX = analogRead(joyXPin);
  bool buttonPressed = digitalRead(joyButtonPin) == LOW;

  // Standard midtstilling for alle servoer
  int servo0 = SERVO_MID;
  int servo1 = SERVO_MID;
  int servo2 = SERVO_MID;
  int servo3 = SERVO_MID;

  // Håndter joystick-knappen
  if (buttonPressed) {
    // Beveg servo 0 mot høyre og servo 1 mot venstre
    servo0 = SERVO_MAX;
    servo1 = SERVO_MIN;
  }

  // Håndter joystick X-bevegelse
  if (joyX > 512 + threshold) {
    // Høyre bevegelse
    servo2 = SERVO_MAX;
  } else if (joyX < 512 - threshold) {
    // Venstre bevegelse
    servo3 = SERVO_MIN;
  }

  // Oppdater servoene
  pwm.setPWM(0, 0, servo0);  // Servo 0
  pwm.setPWM(1, 0, servo1);  // Servo 1
  pwm.setPWM(2, 0, servo2);  // Servo 2
  pwm.setPWM(3, 0, servo3);  // Servo 3

  delay(50);  // Stabil oppdateringshastighet
}
