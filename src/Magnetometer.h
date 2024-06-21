#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <Arduino.h>
#include <Wire.h>

class Magnetometer {
public:
    Magnetometer();

    // Initialize the magnetometer with the default I2C rate
    void begin(uint32_t i2cRate = 400000); // Default rate is 400kHz

    void update();

    float getX();
    float getY();
    float getZ();

    void calibrate();

private:
    void initialize();
    void standby();
    void activate();
    void magOffset();
    void readMagData(int16_t *destination);
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
