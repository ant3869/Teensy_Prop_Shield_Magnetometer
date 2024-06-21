#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <Arduino.h>
#include <Wire.h>

#define FXOS8700CQ_ADDRESS 0x1E
#define FXOS8700CQ_WHO_AM_I 0x0D
#define FXOS8700CQ_M_DR_STATUS 0x32
#define FXOS8700CQ_M_OUT_X_MSB 0x33    
#define FXOS8700CQ_M_OUT_X_LSB 0x34
#define FXOS8700CQ_M_OUT_Y_MSB 0x35
#define FXOS8700CQ_M_OUT_Y_LSB 0x36
#define FXOS8700CQ_M_OUT_Z_MSB 0x37
#define FXOS8700CQ_M_OUT_Z_LSB 0x38
#define FXOS8700CQ_M_OFF_X_MSB 0x3F    
#define FXOS8700CQ_M_OFF_X_LSB 0x40
#define FXOS8700CQ_M_OFF_Y_MSB 0x41
#define FXOS8700CQ_M_OFF_Y_LSB 0x42
#define FXOS8700CQ_M_OFF_Z_MSB 0x43
#define FXOS8700CQ_M_OFF_Z_LSB 0x44
#define FXOS8700CQ_M_CTRL_REG1 0x5B
#define FXOS8700CQ_M_CTRL_REG2 0x5C

#define NUM_SAMPLES 5

class Magnetometer {
public:
    Magnetometer();

    void begin(uint32_t i2cRate = 400000);
    void update();
    float getX();
    float getY();
    float getZ();
    void calibrate();
    void readMagSensor(); // Update to handle averaging internally
    float getMagValue();  // Method to get the averaged magnetic field magnitude
    bool initialize();
    bool checkConnection();

private:
    void standby();
    void activate();
    void magOffset();
    void readMagData(int16_t *destination);
    void writeMagBias(uint8_t address, int16_t *bias);
    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    uint8_t readByte(uint8_t address, uint8_t subAddress);
    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);

    int16_t magCount[3]; // Raw magnetometer data
    float mx, my, mz;    // Magnetometer values in milliGauss
    float mRes = 0.1f;   // Magnetometer resolution in milliGauss per LSB (example value, adjust as necessary)
    uint8_t magOSR = 0x03; // Magnetometer oversampling rate (example value, adjust as necessary)

    // Variables for averaging
    float magSensorHistory[NUM_SAMPLES];
    uint8_t magHistoryIndex;
    float magValue; // Averaged magnetic field magnitude
};

#endif
