#include "Magnetometer.h"

Magnetometer::Magnetometer() : magHistoryIndex(0), magValue(0.0) {
    memset(magSensorHistory, 0, sizeof(magSensorHistory));
}

void Magnetometer::begin(uint32_t i2cRate) {
    Wire.begin();
    Wire.setClock(i2cRate);
    if (initialize()) {
        magOffset();
    } else {
        Serial.println("Magnetometer initialization failed. Skipping calibration.");
    }
}

void Magnetometer::update() {
    if (readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_DR_STATUS) & 0x08) {
        readMagData(magCount);
        mx = (float)magCount[0] * mRes;
        my = (float)magCount[1] * mRes;   
        mz = (float)magCount[2] * mRes;
    }
}

float Magnetometer::getX() { return mx; }
float Magnetometer::getY() { return my; }
float Magnetometer::getZ() { return mz; }

void Magnetometer::calibrate() { magOffset(); }

void Magnetometer::readMagSensor() {
    update();
    float magneticFieldMagnitude = sqrt(mx * mx + my * my + mz * mz);

    magSensorHistory[magHistoryIndex] = magneticFieldMagnitude;
    magHistoryIndex = (magHistoryIndex + 1) % NUM_SAMPLES;

    float sum = 0;
    for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        sum += magSensorHistory[i];
    }

    magValue = sum / NUM_SAMPLES;
}

float Magnetometer::getMagValue() {
    return magValue;
}

void Magnetometer::readMagData(int16_t *destination) {
    uint8_t rawData[6];
    readBytes(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OUT_X_MSB, 6, &rawData[0]);
    destination[0] = (int16_t)(((rawData[0] << 8) | rawData[1]));
    destination[1] = (int16_t)(((rawData[2] << 8) | rawData[3]));
    destination[2] = (int16_t)(((rawData[4] << 8) | rawData[5]));
}

void Magnetometer::magOffset() {
    uint16_t ii = 0, sample_count = 0;
    int16_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {0x7fff, 0x7fff, 0x7fff}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};
    float dest1[3] = {0, 0, 0}, dest2[3] = {0, 0, 0};

    Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    delay(4000);

    standby();

    readMagData(mag_temp);
    readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_DR_STATUS);

    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, magOSR << 2 | 0x03);
    activate();

    sample_count = 512;
    for (ii = 0; ii < sample_count; ii++) {
        readMagData(mag_temp);
        for (int jj = 0; jj < 3; jj++) {
            if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
        delay(10);
    }

    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;

    dest1[0] = (float)mag_bias[0] * mRes;
    dest1[1] = (float)mag_bias[1] * mRes;   
    dest1[2] = (float)mag_bias[2] * mRes;

    mag_scale[0] = (mag_max[0] - mag_min[0]) / 2;
    mag_scale[1] = (mag_max[1] - mag_min[1]) / 2;
    mag_scale[2] = (mag_max[2] - mag_min[2]) / 2;

    float avg_rad = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3.0;

    dest2[0] = avg_rad / ((float)mag_scale[0]);
    dest2[1] = avg_rad / ((float)mag_scale[1]);
    dest2[2] = avg_rad / ((float)mag_scale[2]);

    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_X_MSB, ((mag_bias[0] << 1) & 0xFF00) >> 8);
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_X_LSB, (mag_bias[0] << 1) & 0x00FF);
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Y_MSB, ((mag_bias[1] << 1) & 0xFF00) >> 8);
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Y_LSB, (mag_bias[1] << 1) & 0x00FF);
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Z_MSB, ((mag_bias[2] << 1) & 0xFF00) >> 8);
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OFF_Z_LSB, (mag_bias[2] << 1) & 0x00FF);
    Serial.println("Mag Calibration done!");

    activate();
}

void Magnetometer::standby() {
    byte c = readByte(FXOS8700CQ_ADDRESS, 0x2A);
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, c & ~(0x01));
}

void Magnetometer::activate() {
    byte c = readByte(FXOS8700CQ_ADDRESS, 0x2A);
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, c | 0x01);
}

bool Magnetometer::initialize() {
    standby();
    writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, magOSR << 2 | 0x03);
    activate();
    
    return checkConnection();
}

void Magnetometer::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.write(data);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        Serial.print("I2C write error: ");
        Serial.println(error);
    }
}

uint8_t Magnetometer::readByte(uint8_t address, uint8_t subAddress) {
    uint8_t data;
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    uint8_t error = Wire.endTransmission(false); // Use false for repeated start condition
    if (error != 0) {
        Serial.print("I2C write error: ");
        Serial.println(error);
        return 0;
    }
    Wire.requestFrom(address, (size_t)1);
    if (Wire.available()) {
        data = Wire.read();
    } else {
        Serial.println("I2C read error: No data available");
        data = 0;
    }
    return data;
}

void Magnetometer::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest) {
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    uint8_t error = Wire.endTransmission(false); // Use false for repeated start condition
    if (error != 0) {
        Serial.print("I2C write error: ");
        Serial.println(error);
        return;
    }
    Wire.requestFrom(address, (size_t)count);
    if (Wire.available() == count) {
        for (uint8_t i = 0; i < count; i++) {
            dest[i] = Wire.read();
        }
    } else {
        Serial.println("I2C read error: Incomplete data");
    }
}

bool Magnetometer::checkConnection() {
    uint8_t whoAmI = readByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_WHO_AM_I);
    Serial.print("WHO_AM_I register: 0x");
    Serial.println(whoAmI, HEX);
    return (whoAmI == 0xC7);
}
