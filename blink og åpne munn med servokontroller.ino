#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int joystickY = A1;
const int buttonPin = 2;

const int servoMin = 150;  // Minimum pulse length
const int servoMax = 600;  // Maximum pulse length

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);  // 60 Hz for analog servos

  pinMode(buttonPin, INPUT_PULLUP);  // Joystick-knappen
}

void loop() {
  int buttonState = digitalRead(buttonPin);
  int joyY = analogRead(joystickY);

  // Kontroller servo 1 og 2 ved knappetrykk
  if (buttonState == LOW) {
    // Aktiver servo 1 og 2 til en bestemt posisjon
    pwm.setPWM(0, 0, servoMin);  // Servo 1
    pwm.setPWM(1, 0, servoMax);  // Servo 2
    delay(200);
    pwm.setPWM(0, 0, servoMax);
    pwm.setPWM(1, 0, servoMin);
  }

  // Kontroller servo 3 og 4 med joystick opp/ned
  // Juster følsomheten etter behov
  if (joyY > 600) {
    pwm.setPWM(2, 0, servoMin);  // Servo 3
    pwm.setPWM(3, 0, servoMax);  // Servo 4
  } else if (joyY < 400) {
    pwm.setPWM(2, 0, servoMax);
    pwm.setPWM(3, 0, servoMin);
  } else {
    // Midtposisjon, eller ingen bevegelse
    pwm.setPWM(2, 0, (servoMin + servoMax) / 2);
    pwm.setPWM(3, 0, (servoMin + servoMax) / 2);
  }

  delay(100);  // Litt forsinkelse for stabilitet
}
