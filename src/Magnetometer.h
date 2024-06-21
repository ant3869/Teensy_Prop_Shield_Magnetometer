#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <Arduino.h>
#include <Wire.h>

class Magnetometer {
public:
    Magnetometer();

    // Initialize the magnetometer with specified I2C pins and rate
    void begin(uint8_t i2cPins = I2C_PINS_18_19, uint32_t i2cRate = I2C_RATE_400);

    void update();

    // Get the X, Y, and Z magnetic field values
    float getX();
    float getY();
    float getZ();

    void calibrate();

private:
    void initialize();

    void standby();

    void activate();

    // Calculate and set the magnetometer offset
    void magOffset();

    // Read raw magnetometer data
    void readMagData(int16_t *destination);

    // Write bias values to the magnetometer
    void writeMagBias(uint8_t address, int16_t *bias);

    // Utility functions for I2C communication
    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    uint8_t readByte(uint8_t address, uint8_t subAddress);
    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);

    int16_t magCount[3]; // Raw magnetometer data
    float mx, my, mz;    // Magnetometer values in milliGauss

    float mRes = 0.1f;   // Magnetometer resolution in milliGauss per LSB (example value, adjust as necessary)
    uint8_t magOSR = 0x03; // Magnetometer oversampling rate (example value, adjust as necessary)
};

#endif // MAGNETOMETER_H