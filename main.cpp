#include <Servo.h>

template<uint8_t smoothingK, class uint_t = uint16_t>
class EMA {
public:
  const int maxDifference = 100;

  /// Update the filter with the given input and return the filtered output.
  uint_t operator()(uint_t input) {
    state += input;
    uint_t output = (state + half) >> smoothingK;
    state -= output;
    return output;
  }

  static_assert(
    uint_t(0) < uint_t(-1),  // Check that `uint_t` is an unsigned type
    "The `uint_t` type should be an unsigned integer, otherwise, "
    "the division using bit shifts is invalid.");

  /// Fixed point representation of one half, used for rounding.
  constexpr static uint_t half = 1 << (smoothingK - 1);

private:
  uint_t state = 0;
};

int sensorAngle;
int pinNumber = A5;
int servoAttachAddend = 2;

const int NUMBEROFSERVOS = 22;

float maxDegrees = 180;
float maxSensorValue = 1023;
float sensorValue;

float sensorRatio;

const unsigned long interval = 10000;  // 10000 µs = 100 Hz

unsigned long moveStartTime = millis();  // start moving

float lastPosition;

Servo servosToUse[NUMBEROFSERVOS];

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  //Calculated in setup() in order to be calculated once, therefore saving resources, and making loop() more readable
  sensorRatio = maxDegrees / maxSensorValue;
  sensorValue = analogRead(pinNumber);

  //Serial.println(NUMBEROFSERVOS);

  AssignServos();

  moveStartTime = millis();  // start moving
}


// the loop routine runs over and over again forever:
void loop() {
  MotorController(servosToUse,
                  SensorHandler(),
                  CalculateSpeed(analogRead(A5)));
}

void AssignServos() {
  int currentServoAddend;

  for (int currentServo = 0; currentServo < NUMBEROFSERVOS; currentServo++) {
    currentServoAddend = currentServo + servoAttachAddend;
    servosToUse[currentServo].attach(currentServoAddend);

    Serial.print(currentServoAddend);
  }
}

float SensorHandler() {
  long currentMicroSeconds = micros();

  static EMA<4> filter;
  static unsigned long previousMicroSeconds = currentMicroSeconds - interval;

  if (currentMicroSeconds - previousMicroSeconds >= interval) {
    int rawValue = analogRead(A5);
    int filteredValue = filter(rawValue);
    int difference = abs(rawValue - filteredValue);

    if (difference < filter.maxDifference) {
      //remap sensorValue to corresponding angle, using a specific ratio. maxDegrees can change according to how much movement is required (the reason why it isn't calculated on
      //beforehand), although maxSensorValue shouln't since this is limited by hardware.
      sensorAngle = filteredValue * sensorRatio;
      previousMicroSeconds += interval;
    } else {
      sensorAngle = rawValue * sensorRatio;
      previousMicroSeconds += interval;
    }
  }
  return sensorAngle;
}

//This method controlls the functionality of the servos. First loop is to change the current angle and the second is there to make the change to all servos.
void MotorController(Servo servosToUse[], int angleToTarget, float servoSpeed) {
  for (int currentServo = 0; currentServo < NUMBEROFSERVOS; currentServo++) {
    unsigned long progress = millis() - moveStartTime;

    if (progress <= servoSpeed) {
      long mapedAngle = map(progress,
                            0,
                            servoSpeed,
                            lastPosition,
                            angleToTarget);
      servosToUse[currentServo].write(mapedAngle);
      lastPosition = mapedAngle;
      Serial.println(mapedAngle);
    }
  }
}

float CalculateSpeed(int sensorSignal) {
  int speed = sensorSignal * 100;
  return speed;
}
