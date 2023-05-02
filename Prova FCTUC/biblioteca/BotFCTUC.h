// =============================================================
// = Header File BotFCTUC                       BotOlympics 2023
// = JNDVasco - Rev 1.0
// = CodeCritical - Rev 2.0
// =
// = Hardware Mapping
// = |---------------------|--------|
// = |     Descrição       |  Pino  |
// = |---------------------|--------|
// = |Motor 1 - Dir        |    4   |
// = |---------------------|--------|
// = |Motor 1 - PWM        |    5   |
// = |---------------------|--------|
// = |Motor 2 - PWM        |    6   |
// = |---------------------|--------|
// = |LED NeoPixel         |    7   |
// = |---------------------|--------|
// = |Motor 2 - Dir        |    8   |
// = |---------------------|--------|
// = |Buzzer               |    9   |
// = |---------------------|--------|
// = |XSHUT LiDAR Direita  |   10   |
// = |---------------------|--------|
// = |XSHUT LiDAR Frente   |   11   |
// = |---------------------|--------|
// = |XSHUT LiDAR Esquerda |   12   |
// = |---------------------|--------|
// = |Sensor infravermelhos|   A0   |
// = |---------------------|--------|
// = |Botão                |   A1   |
// = |---------------------|--------|
// = |Ventoinha            |   A2   |
// = |---------------------|--------|
// =
// =============================================================

#ifndef BOT_FCTUC_H
#define BOT_FCTUC_H

// Auxiliary libraries
#include <Wire.h>

// Third-party auxiliary libraries
#include <./external/Adafruit_NeoPixel.h>
#include <./external/Adafruit_TCS34725.h>
#include <./external/VL53L0X.h>

/**
 * @brief Container for 8-bit RGBC color sensor values.
 */
struct ColorRGBC {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint8_t c = 0;
};

/**
 * @brief Class to interface with robot hardware.
 */
class BotFCTUC {
private:
    static constexpr uint16_t PIN_MOTOR_1_DIRECTION = 4;
    static constexpr uint16_t PIN_MOTOR_1_PWM = 5;
    static constexpr uint16_t PIN_MOTOR_2_DIRECTION = 8;
    static constexpr uint16_t PIN_MOTOR_2_PWM = 6;
    static constexpr uint16_t PIN_NEOPIXEL = 7;
    static constexpr uint16_t PIN_BUZZER = 9;
    static constexpr uint16_t PIN_XSHUT_RIGHT = 10;
    static constexpr uint16_t PIN_XSHUT_FRONT = 11;
    static constexpr uint16_t PIN_XSHUT_LEFT = 12;
    static constexpr uint16_t PIN_INFRARED = A0;
    static constexpr uint16_t PIN_BUTTON = A1;
    static constexpr uint16_t PIN_FAN = A2;

    static constexpr uint16_t ADDR_LIDAR_LEFT= 0x70;
    static constexpr uint16_t ADDR_LIDAR_FRONT = 0x71;
    static constexpr uint16_t ADDR_LIDAR_RIGHT = 0x72;

    static constexpr uint16_t DIST_LIDAR_MIN = 0;
    static constexpr uint16_t DIST_LIDAR_MAX = 2600;

    static constexpr int16_t DUTY_MOTOR_MIN = -255;
    static constexpr int16_t DUTY_MOTOR_MAX = 255;

    bool invertMotor1 = false;
    bool invertMotor2 = false;

    void setupFan();
    void setupButton();
    void setupBuzzer();
    void setupNeopixel();
    void setupFlameSensor();
    void setupColorSensor();
    void setupMotors(bool shouldInvertMotor1, bool shouldInvertMotor2);
    void setupLidar();

public:
    Adafruit_NeoPixel deviceNeoPixel;
    Adafruit_TCS34725 deviceColorSensor;

    VL53L0X deviceLidarRight;
    VL53L0X deviceLidarFront;
    VL53L0X deviceLidarLeft;

    void begin(bool shouldInvertMotor1 = false, bool shouldInvertMotor2 = false);

    void fanOn();
    void fanOff();

    void waitStart();
    bool readButton();

    void buzzerPlay(uint8_t tone);

    void setPixelColor(uint8_t red, uint8_t green, uint8_t blue);
    void setPixelBrightness(uint8_t brightness);

    uint16_t getFlameValue();

    ColorRGBC getColorValue();

    void moveMotor1(int16_t duty);
    void moveMotor2(int16_t duty);
    void moveMotors(int16_t dutyMotor1, int16_t dutyMotor2);
    void stopMotors();

    uint16_t getLidarRightDistance();
    uint16_t getLidarFrontDistance();
    uint16_t getLidarLeftDistance();

    void printI2C();
    void printLidarValue();
    void printFlameValue();
};

#endif