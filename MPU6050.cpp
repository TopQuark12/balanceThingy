#include "MPU6050.h"

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    Wire.beginTransmission(address); // Initialize the Tx buffer
    Wire.write(subAddress);          // Put slave register address in Tx buffer
    Wire.endTransmission(false);     // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count); // Read bytes from slave register address
    while (Wire.available())
    {
        dest[i++] = Wire.read();
    } // Put read results in the Rx buffer
}

void readGyroData(uint8_t MPUnum, int16_t *destination)
{
    uint8_t rawData[6];                                       // x/y/z gyro register data stored here
    readBytes(MPUnum, 0x43, 6, &rawData[0]);                  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void readAccelData(uint8_t MPUnum, int16_t *destination)
{
    uint8_t rawData[6];                                       // x/y/z accel register data stored here
    readBytes(MPUnum, 0x3B, 6, &rawData[0]);                  // Read the six raw data registers into data array
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void readAllData(uint8_t MPUnum, int16_t *destination)
{
    uint8_t rawData[14];
    readBytes(MPUnum, ACCEL_XOUT_H, 14, &rawData[0]);
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
    destination[3] = ((int16_t)rawData[6] << 8) | rawData[7];
    destination[4] = ((int16_t)rawData[8] << 8) | rawData[9];
    destination[5] = ((int16_t)rawData[10] << 8) | rawData[11];
    destination[6] = ((int16_t)rawData[12] << 8) | rawData[13];
}

void GyroBiasWriteToReg(uint8_t MPUnum, int32_t *gyro_bias)
{
    uint8_t data[12];
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    writeByte(MPUnum, XG_OFFSET_H, data[0]);
    writeByte(MPUnum, XG_OFFSET_L, data[1]);
    writeByte(MPUnum, YG_OFFSET_H, data[2]);
    writeByte(MPUnum, YG_OFFSET_L, data[3]);
    writeByte(MPUnum, ZG_OFFSET_H, data[4]);
    writeByte(MPUnum, ZG_OFFSET_L, data[5]);
}


void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address); // Initialize the Tx buffer
    Wire.write(subAddress);          // Put slave register address in Tx buffer
    Wire.write(data);                // Put data in Tx buffer
    Wire.endTransmission();          // Send the Tx buffer
}
