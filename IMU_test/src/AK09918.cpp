/*
    Adapt from the below library to using for using with Raspberry Pi Pico,
    in : January 2024,
    By : MaimonidesA
  
    AK09918.cpp
    A library for Grove - IMU 9DOF(ICM20600 + AK09918)

    Copyright (c) 2018 seeed technology inc.
    Website    : www.seeed.cc
    Author     : Jerry Yip
    Create Time: 2018-06
    Version    : 0.1
    Change Log :

    The MIT License (MIT)
*/

#include <string>
#include "AK09918.h"

static uint8_t* pico_i2c_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t* data){
    uint8_t Register = regAddr;
    i2c_write_blocking(I2C_CHAN, devAddr, &Register, 1, true);
    i2c_read_blocking(I2C_CHAN, devAddr, data, 1, false);
    return data;
}

static uint8_t* pico_i2c_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data){
    uint8_t Register = regAddr;
    i2c_write_blocking(I2C_CHAN, devAddr, &Register, 1, true);
    i2c_read_blocking(I2C_CHAN, devAddr, data, length, false);
    return data;
}

static bool pico_i2c_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data){
    uint8_t reg[] = {regAddr, data};
    if (i2c_write_blocking(I2C_CHAN, devAddr, reg, 2, false) == 2){
        return true;
    }
    else {
        return false;
    }
}


AK09918::AK09918() {
    _addr = AK09918_I2C_ADDR;
}


AK09918_err_type_t AK09918::initialize(AK09918_mode_type_t mode) {
    if (mode == AK09918_SELF_TEST) {
        mode = AK09918_POWER_DOWN;
    }
    _mode = mode;

    if (mode == AK09918_NORMAL) {
        return AK09918_ERR_OK;
    } else {
        return AK09918::switchMode(_mode);
    }
}

AK09918_err_type_t AK09918::isDataReady() {
    if (!pico_i2c_readByte(_addr, AK09918_ST1, _buffer)) {
        return AK09918_ERR_READ_FAILED;
    } else {
        if (_buffer[0] & AK09918_DRDY_BIT) {
            return AK09918_ERR_OK;
        } else {
            return AK09918_ERR_NOT_RDY;
        }
    }
}

AK09918_err_type_t AK09918::isDataSkip() {
    if (!pico_i2c_readByte(_addr, AK09918_ST1, _buffer)) {
        return AK09918_ERR_READ_FAILED;
    } else {
        if (_buffer[0] & AK09918_DOR_BIT) {
            return AK09918_ERR_DOR;
        } else {
            return AK09918_ERR_OK;
        }
    }
}

AK09918_err_type_t AK09918::getData(int32_t* axis_x, int32_t* axis_y, int32_t* axis_z) {
    AK09918_err_type_t err = AK09918::getRawData(axis_x, axis_y, axis_z);
    (*axis_x) = (*axis_x) * 15 / 100;
    (*axis_y) = (*axis_y) * 15 / 100;
    (*axis_z) = (*axis_z) * 15 / 100;

    return err;
}

AK09918_err_type_t AK09918::getRawData(int32_t* axis_x, int32_t* axis_y, int32_t* axis_z) {
    if (_mode == AK09918_NORMAL) {
        AK09918::switchMode(AK09918_NORMAL);
        bool is_end = false;
        int count = 0;
        while (!is_end) {
            if (AK09918::_getRawMode() == 0x00) {
                is_end = true;
            }
            if (count >= 15) {
                return AK09918_ERR_TIMEOUT;
            }
            count ++;
            sleep_ms(1);
        }
    }


    if (!pico_i2c_readBytes(_addr, AK09918_HXL, 8, _buffer)) {
        return AK09918_ERR_READ_FAILED;
    } else {
        *axis_x = (int16_t)(_buffer[1] << 8 | _buffer[0]);
        *axis_y = (int16_t)(_buffer[3] << 8 | _buffer[2]);
        *axis_z = (int16_t)(_buffer[5] << 8 | _buffer[4]);
        if (_buffer[7] & AK09918_HOFL_BIT) {
            return AK09918_ERR_OVERFLOW;
        }
        return AK09918_ERR_OK;
    }
}

AK09918_mode_type_t AK09918::getMode() {
    return _mode;
}

AK09918_err_type_t AK09918::switchMode(AK09918_mode_type_t mode) {
    if (mode == AK09918_SELF_TEST) {
        return AK09918_ERR_WRITE_FAILED;
    }
    _mode = mode;
    if (!pico_i2c_writeByte(_addr, AK09918_CNTL2, mode)) {
        return AK09918_ERR_WRITE_FAILED;
    }
    return AK09918_ERR_OK;
}

// 1.Set Power-down mode. (MODE[4:0] bits = “00000”)
// 2.Set Self-test mode. (MODE[4:0] bits = “10000”)
// 3.Check Data Ready or not by polling DRDY bit of ST1 register.
// 4.When Data Ready, proceed to the next step. Read measurement data. (HXL to HZH)
AK09918_err_type_t AK09918::selfTest() {
    int32_t axis_x, axis_y, axis_z;
    bool is_end = false;
    AK09918_err_type_t err;
    if (!pico_i2c_writeByte(_addr, AK09918_CNTL2, AK09918_POWER_DOWN)) {
        return AK09918_ERR_WRITE_FAILED;
    }
    sleep_ms(1);
    if (!pico_i2c_writeByte(_addr, AK09918_CNTL2, AK09918_SELF_TEST)) {
        return AK09918_ERR_WRITE_FAILED;
    }

    while (!is_end) {
        err = AK09918::isDataReady();
        if (err == AK09918_ERR_OK) {
            is_end = true;
        } else if (err == AK09918_ERR_READ_FAILED) {
            return AK09918_ERR_READ_FAILED;
        }
        sleep_ms(1);
    }

    // read data and check
    if (!pico_i2c_readBytes(_addr, AK09918_HXL, 8, _buffer)) {
        return AK09918_ERR_READ_FAILED;
    } else {
        axis_x = (int32_t)((((int16_t)_buffer[1]) << 8) | _buffer[0]);
        axis_y = (int32_t)((((int16_t)_buffer[3]) << 8) | _buffer[2]);
        axis_z = (int32_t)((((int16_t)_buffer[5]) << 8) | _buffer[4]);

        // Serial.print("X:");
        // Serial.print(axis_x);
        // Serial.print("\tY:");
        // Serial.print(axis_y);
        // Serial.print("\tZ:");
        // Serial.println(axis_z);

        if ((axis_x >= -200) && (axis_x <= 200) && (axis_y >= -200) && (axis_y <= 200) && \
                (axis_z >= -1000) && (axis_z <= -150)) {
            return AK09918_ERR_OK;
        } else {
            return AK09918_ERR_SELFTEST_FAILED;
        }

    }
}

AK09918_err_type_t AK09918::reset() {
    if (!pico_i2c_writeByte(_addr, AK09918_CNTL3, AK09918_SRST_BIT)) {
        return AK09918_ERR_WRITE_FAILED;
    }
    return AK09918_ERR_OK;
}

std::string AK09918::strError(AK09918_err_type_t err) {
    std::string result;
    switch (err) {
        case AK09918_ERR_OK:
            result = "AK09918_ERR_OK: OK";
            break;

        case AK09918_ERR_DOR:
            result = "AK09918_ERR_DOR: Data skipped";
            break;

        case AK09918_ERR_NOT_RDY:
            result = "AK09918_ERR_NOT_RDY: Not ready";
            break;

        case AK09918_ERR_TIMEOUT:
            result = "AK09918_ERR_TIMEOUT: Timeout";
            break;

        case AK09918_ERR_SELFTEST_FAILED:
            result = "AK09918_ERR_SELFTEST_FAILED: Self test failed";
            break;

        case AK09918_ERR_OVERFLOW:
            result = "AK09918_ERR_OVERFLOW: Sensor overflow";
            break;

        case AK09918_ERR_WRITE_FAILED:
            result = "AK09918_ERR_WRITE_FAILED: Fail to write";
            break;

        case AK09918_ERR_READ_FAILED:
            result = "AK09918_ERR_READ_FAILED: Fail to read";
            break;

        default:
            result = "Unknown Error";
            break;
    }
    return result;
}

uint16_t AK09918::getDeviceID() {
    pico_i2c_readBytes(_addr, AK09918_WIA1, 2, _buffer);
    return (((uint16_t)_buffer[0]) << 8) | _buffer[1];
}

uint8_t AK09918::_getRawMode() {
    if (!pico_i2c_readByte(0x0c, AK09918_CNTL2, _buffer)) {
        return 0xFF;
    } else {
        return _buffer[0];
    }
}