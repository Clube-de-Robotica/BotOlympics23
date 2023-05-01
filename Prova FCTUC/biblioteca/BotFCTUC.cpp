#include "BotFCTUC.h"

void BotFCTUC::setupFan() {
    pinMode(PIN_FAN, OUTPUT);
    fanOff();
}

void BotFCTUC::setupButton() {
    pinMode(PIN_BUTTON, INPUT);
}

void BotFCTUC::setupBuzzer() {
    pinMode(PIN_BUZZER, OUTPUT);
    buzzerPlay(0);
}

void BotFCTUC::setupNeopixel() {
    pinMode(PIN_NEOPIXEL, OUTPUT);

    deviceNeoPixel = Adafruit_NeoPixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
    deviceNeoPixel.begin();

    setPixelBrightness(255);
    setPixelColor(0, 255, 0);
}

void BotFCTUC::setupFlameSensor() {
    pinMode(PIN_INFRARED, INPUT);
}

void BotFCTUC::setupColorSensor() {
    deviceColorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);
    deviceColorSensor.begin();
}

void BotFCTUC::setupMotors(bool shouldInvertMotor1, bool shouldInvertMotor2) {
    invertMotor1 = shouldInvertMotor1;
    invertMotor2 = shouldInvertMotor2;

    pinMode(PIN_MOTOR_1_DIRECTION, OUTPUT);
    pinMode(PIN_MOTOR_1_PWM, OUTPUT);
    pinMode(PIN_MOTOR_2_DIRECTION, OUTPUT);
    pinMode(PIN_MOTOR_2_PWM, OUTPUT);

    stopMotors();
}

void BotFCTUC::setupLidar() {
    Wire.begin();
    
    pinMode(PIN_XSHUT_RIGHT, OUTPUT);
    pinMode(PIN_XSHUT_FRONT, OUTPUT);
    pinMode(PIN_XSHUT_LEFT, OUTPUT);

    digitalWrite(PIN_XSHUT_RIGHT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_RIGHT, HIGH);
    delay(200);
    deviceLidarRight.setAddress(ADDR_LIDAR_RIGHT);
    deviceLidarRight.setTimeout(500);
    deviceLidarRight.init(true);

    digitalWrite(PIN_XSHUT_FRONT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_FRONT, HIGH);
    delay(200);
    deviceLidarFront.setAddress(ADDR_LIDAR_FRONT);
    deviceLidarFront.setTimeout(500);
    deviceLidarFront.init(true);

    digitalWrite(PIN_XSHUT_LEFT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_LEFT, HIGH);
    delay(200);
    deviceLidarLeft.setAddress(ADDR_LIDAR_LEFT);
    deviceLidarLeft.setTimeout(500);
    deviceLidarLeft.init(true);

    deviceLidarRight.startContinuous(0);
    deviceLidarFront.startContinuous(0);
    deviceLidarLeft.startContinuous(0);
}

/**
 * @brief Initialize the hardware interface. Must be called to interact with robot.
 * @param shouldInvertMotor1 invert motor 1 if it spins the wrong way
 * @param shouldInvertMotor2 invert motor 2 if it spins the wrong way
 */
void BotFCTUC::begin(bool shouldInvertMotor1, bool shouldInvertMotor2) {
    setupFan();
    setupButton();
    setupBuzzer();
    setupNeopixel();
    setupFlameSensor();
    setupColorSensor();
    setupMotors(shouldInvertMotor1, shouldInvertMotor2);
    setupLidar();
}

/**
 * @brief Turn the fan on.
 * @note Control logic is inverted (1 = Off, 0 = On)
 */
void BotFCTUC::fanOn() {
    digitalWrite(PIN_FAN, LOW);
}

/**
 * @brief Turn the fan off.
 * @note Control logic is inverted (1 = Off, 0 = On)
 */
void BotFCTUC::fanOff() {
    digitalWrite(PIN_FAN, HIGH);
}

/**
 * @brief Waits until the button is pressed. This will block execution.
 */
void BotFCTUC::waitStart() {
    Serial.println("[INFO] - BotFCTUC is waiting to start!");

    static uint32_t lastMillis = 0;
    static bool enabled = false;

    while (!digitalRead(PIN_BUTTON)) {
        if (millis() - lastMillis >= 500) {
            lastMillis = millis();
            setPixelColor(0, (enabled) * 255, 0);
            enabled = !enabled;
        }
    }

    Serial.println("[INFO] - BotFCTUC is starting!");
}

/**
  @brief Read the button state.
  @return true if button is pressed, false if not
  */
bool BotFCTUC::readButton() {
    return digitalRead(PIN_BUTTON);
}

/**
  @brief Play a sound on the buzzer.
  @param tone tone to be played
 */
void BotFCTUC::buzzerPlay(uint8_t tone) {
    analogWrite(PIN_BUZZER, tone);
}

/**
  @brief Set the NeoPixel's color.
  @param red value between [0, 255]
  @param green value between [0, 255]
  @param blue value between [0, 255]
 */
void BotFCTUC::setPixelColor(uint8_t red, uint8_t green, uint8_t blue) {
    deviceNeoPixel.setPixelColor(0, deviceNeoPixel.Color(red, green, blue)); // (pixelID, color)
    deviceNeoPixel.show();
}

/**
  @brief Set the NeoPixel's brightness.
  @param brightness value between [0, 255]
 */
void BotFCTUC::setPixelBrightness(uint8_t brightness) {
    deviceNeoPixel.setBrightness(brightness);
    deviceNeoPixel.show();
}

/**
  @brief Get value from the flame sensor.
  @return value between [0, 1023]
 */
uint16_t BotFCTUC::getFlameValue() {
    return analogRead(PIN_INFRARED);
}

/**
  @brief Get values from the color sensor.
  @return red, green, blue and clear channel values between [0, 1023]
 */
ColorRGBC BotFCTUC::getColorValue() {
    ColorRGBC result;
    uint16_t red, green, blue, clear;
    
    deviceColorSensor.getRawData(&red, &green, &blue, &clear);

    result.r = red / 256;
    result.g = green / 256;
    result.b = blue / 256;
    result.c = clear / 256;

    return result;
}

/**
  @brief Control motor 1 speed.
  @param duty desired duty cycle for the motor, value between [-255, 255]
 */
void BotFCTUC::moveMotor1(int16_t duty) {
    duty = constrain(duty, DUTY_MOTOR_MIN, DUTY_MOTOR_MAX);

    if (invertMotor1) {
        if (duty < 0) {
            digitalWrite(PIN_MOTOR_1_DIRECTION, LOW);
            analogWrite(PIN_MOTOR_1_PWM, abs(duty));
        } else {
            duty = DUTY_MOTOR_MAX - duty;
            digitalWrite(PIN_MOTOR_1_DIRECTION, HIGH);
            analogWrite(PIN_MOTOR_1_PWM, duty);
        }
    } else {
        if (duty > 0) {
            digitalWrite(PIN_MOTOR_1_DIRECTION, LOW);
            analogWrite(PIN_MOTOR_1_PWM, duty);
        }  else {
            duty = DUTY_MOTOR_MAX + duty;
            digitalWrite(PIN_MOTOR_1_DIRECTION, HIGH);
            analogWrite(PIN_MOTOR_1_PWM, duty);
        }
    }
}

/**
  @brief Control motor 2 speed.
  @param duty desired duty cycle for the motor, value between [-255, 255]
 */
void BotFCTUC::moveMotor2(int16_t duty) {
    duty = constrain(duty, DUTY_MOTOR_MIN, DUTY_MOTOR_MAX);

    if (invertMotor2) {
        if (duty < 0) {
            digitalWrite(PIN_MOTOR_2_DIRECTION, LOW);
            analogWrite(PIN_MOTOR_2_PWM, abs(duty));
        } else {
            duty = DUTY_MOTOR_MAX - duty;
            digitalWrite(PIN_MOTOR_2_DIRECTION, HIGH);
            analogWrite(PIN_MOTOR_2_PWM, duty);
        }
    } else {
        if (duty > 0) {
            digitalWrite(PIN_MOTOR_2_DIRECTION, LOW);
            analogWrite(PIN_MOTOR_2_PWM, duty);
        } else {
            duty = DUTY_MOTOR_MAX + duty;
            digitalWrite(PIN_MOTOR_2_DIRECTION, HIGH);
            analogWrite(PIN_MOTOR_2_PWM, duty);
        }
    }
}

/**
  @brief Control both motors simultaneously.
  @param dutyMotor1 desired duty cycle for motor 1, value between [-255, 255]
  @param dutyMotor2 desired duty cycle for motor 2, value between [-255, 255]
 */
void BotFCTUC::moveMotors(int16_t dutyMotor1, int16_t dutyMotor2) {
    moveMotor1(dutyMotor1);
    moveMotor2(dutyMotor2);
}

/**
  @brief Bring both motors to a stop, this will block execution for 50 ms.
 */
void BotFCTUC::stopMotors() {
    moveMotor1(0);
    moveMotor2(0);
    delay(50);
}

/**
 * @brief Get the right LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t BotFCTUC::getLidarRightDistance() {
    uint16_t result = deviceLidarRight.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Get the front LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t BotFCTUC::getLidarFrontDistance() {
    uint16_t result = deviceLidarFront.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Get the left LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t BotFCTUC::getLidarLeftDistance() {
    uint16_t result = deviceLidarLeft.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
  @brief Print all detected I2C devices.
 */
void BotFCTUC::printI2C() {
    Serial.println("I2C scanner. Scanning ...");
    byte count = 0;

    for (byte i = 1; i < 120; i++) {
        Wire.beginTransmission(i);

        if (Wire.endTransmission() == 0) {
            Serial.print("Found address: ");
            Serial.print(i, DEC);
            Serial.print(" (0x");
            Serial.print(i, HEX);
            Serial.println(")");
            count++;
            delay(1); // maybe unneeded?
        }
    }

    Serial.println("Done.");
    Serial.print("Found ");
    Serial.print(count, DEC);
    Serial.println(" device(s).");
}

/**
  @brief Print all LiDAR distance values.
 */
void BotFCTUC::printLidarValue() {
    uint16_t left = getLidarLeftDistance();
    uint16_t front = getLidarFrontDistance();
    uint16_t right = getLidarRightDistance();

    Serial.println(
        "Left: " + String(left) + "mm" + 
        "\nFront: " + String(front) + "mm" + 
        "\nRight: " + String(right) + "mm"
    );
}

/**
  @brief Print flame sensor value.
 */
void BotFCTUC::printFlameValue() {
    Serial.println("Flame sensor: " + String(getFlameValue()));
}