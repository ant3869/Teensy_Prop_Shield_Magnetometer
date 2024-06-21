#include "Magnetometer.h"
#include "i2c_t3.h"

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

Magnetometer::Magnetometer() {}

void Magnetometer::begin(uint8_t i2cPins, uint32_t i2cRate) {
  Wire.begin(I2C_MASTER, 0x00, i2cPins, I2C_PULLUP_EXT, i2cRate);
  initialize();
  magOffset();
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

void Magnetometer::readMagData(int16_t *destination) {
  uint8_t rawData[6];
  readBytes(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_OUT_X_MSB, 6, rawData);
  destination[0] = (int16_t)(((rawData[0] << 8) | rawData[1]));
  destination[1] = (int16_t)(((rawData[2] << 8) | rawData[3]));
  destination[2] = (int16_t)(((rawData[4] << 8) | rawData[5]));
}

void Magnetometer::magOffset() {
  uint16_t ii = 0, sample_count = 0;
  int16_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {INT16_MIN, INT16_MIN, INT16_MIN}, mag_min[3] = {INT16_MAX, INT16_MAX, INT16_MAX}, mag_temp[3] = {0, 0, 0};
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

  writeMagBias(FXOS8700CQ_ADDRESS, mag_bias);
  Serial.println("Mag Calibration done!");

  activate();
}

void Magnetometer::standby() {
  byte c = readByte(FXOS8700CQ_ADDRESS, 0x2A);
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, c & ~(0x01));
}

void Magnetometer::activate() {
  byte c = readByte(FXOS8700CQ_ADDRESS, 0x2A);
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_CTRL_REG1, c | 0x01);
}

void Magnetometer::initialize() {
  standby();
  writeByte(FXOS8700CQ_ADDRESS, FXOS8700CQ_M_CTRL_REG1, magOSR << 2 | 0x03);
  activate();
}

void Magnetometer::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  uint8_t result = Wire.endTransmission();
  if (result != 0) {
    Serial.print("I2C write error: "); Serial.println(result);
  }
}

uint8_t Magnetometer::readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data;
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  uint8_t result = Wire.endTransmission(false);
  if (result != 0) {
    Serial.print("I2C write error: "); Serial.println(result);
  }
  Wire.requestFrom(address, (size_t)1);
  data = Wire.read();
  return data;
}

void Magnetometer::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  uint8_t result = Wire.endTransmission(false);
  if (result != 0) {
    Serial.print("I2C write error: "); Serial.println(result);
  }
  Wire.requestFrom(address, count);
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = Wire.read();
  }
}

void Magnetometer::writeMagBias(uint8_t address, int16_t *bias) {
  writeByte(address, FXOS8700CQ_M_OFF_X_MSB, ((bias[0] << 1) & 0xFF00) >> 8);
  writeByte(address, FXOS8700CQ_M_OFF_X_LSB, (bias[0] << 1) & 0x00FF);
  writeByte(address, FXOS8700CQ_M_OFF_Y_MSB, ((bias[1] << 1) & 0xFF00) >> 8);
  writeByte(address, FXOS8700CQ_M_OFF_Y_LSB, (bias[1] << 1) & 0x00FF);
  writeByte(address, FXOS8700CQ_M_OFF_Z_MSB, ((bias[2] << 1) & 0xFF00) >> 8);
  writeByte(address, FXOS8700CQ_M_OFF_Z_LSB, (bias[2] << 1) & 0x00FF);
}