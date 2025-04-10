#include <Servo.h>

template<uint8_t smoothingK, class uint_t = uint16_t>
class EMA {
public:
  const int maxDifference = 300;

  uint_t operator()(uint_t input) {
    state += input;
    uint_t output = (state + half) >> smoothingK;
    state -= output;
    return output;
  }

  static_assert(
    uint_t(0) < uint_t(-1),
    "The `uint_t` type should be an unsigned integer.");

  constexpr static uint_t half = 1 << (smoothingK - 1);

private:
  uint_t state = 0;
};

int sensorAngle;
int pinNumber = A5;

float maxDegrees = 180;

float maxSensorValue = 1023;
float sensorValue;
float sensorRatio;

const unsigned long interval = 10000;

unsigned long moveStartTime = millis();

float lastPosition;

Servo myServo;

void setup() {
  Serial.begin(9600);
  sensorRatio = maxDegrees / maxSensorValue;

  myServo.attach(22);  // Using pin 3 for the single servo
  moveStartTime = millis();
}

void loop() {
  int sensorInput = analogRead(pinNumber);
  int angle = SensorHandler();
  float speed = CalculateSpeed(sensorInput);

  MotorController(myServo, angle, speed);
}

float SensorHandler() {
  long currentMicroSeconds = micros();

  static EMA<4> filter;
  static unsigned long previousMicroSeconds = currentMicroSeconds - interval;

  if (currentMicroSeconds - previousMicroSeconds >= interval) {
    int rawValue = analogRead(pinNumber);
    int filteredValue = filter(rawValue);
    int difference = abs(rawValue - filteredValue);

    if (difference < filter.maxDifference) {
      sensorAngle = filteredValue * sensorRatio;
    } else {
      sensorAngle = rawValue * sensorRatio;
    }

    previousMicroSeconds += interval;
  }

  return sensorAngle;
}

void MotorController(Servo& servo, int angleToTarget, float servoSpeed) {
  unsigned long progress = millis() - moveStartTime;

  if (progress <= servoSpeed) {
    long mappedAngle = map(progress, 0, servoSpeed, lastPosition, angleToTarget);
    servo.write(mappedAngle);
    lastPosition = mappedAngle;
    
    Serial.println(mappedAngle);
  }
}

float CalculateSpeed(int sensorSignal) {
  return sensorSignal * 100;
}
