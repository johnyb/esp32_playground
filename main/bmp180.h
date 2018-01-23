#ifndef BMP180_H
#define BMP180_H

#include "driver/i2c.h"

#define BMP180_ADDRESS 0x77
enum
{
    BMP085_REGISTER_CAL_AC1            = 0xAA,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC2            = 0xAC,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC3            = 0xAE,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC4            = 0xB0,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC5            = 0xB2,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC6            = 0xB4,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_B1             = 0xB6,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_B2             = 0xB8,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_MB             = 0xBA,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_MC             = 0xBC,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_MD             = 0xBE,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CHIPID             = 0xD0,
    BMP085_REGISTER_VERSION            = 0xD1,
    BMP085_REGISTER_SOFTRESET          = 0xE0,
    BMP085_REGISTER_CONTROL            = 0xF4,
    BMP085_REGISTER_TEMPDATA           = 0xF6,
    BMP085_REGISTER_PRESSUREDATA       = 0xF6,
    BMP085_REGISTER_READTEMPCMD        = 0x2E,
    BMP085_REGISTER_READPRESSURECMD    = 0x34 // 0011 0100
};
typedef enum
{
    BMP085_MODE_ULTRALOWPOWER          = 0,
    BMP085_MODE_STANDARD               = 1,
    BMP085_MODE_HIGHRES                = 2,
    BMP085_MODE_ULTRAHIGHRES           = 3
} bmp085_mode_t;

#define ACK_CHECK_EN (i2c_ack_type_t)0x1

class BMP180 {
    public:
        BMP180() : BMP180(BMP085_MODE_STANDARD) {};
        BMP180(bmp085_mode_t mode) { m_i2cPort = I2C_NUM_0; m_mode = mode; readParameters(); };
        double readPressure();
        double readTemperature();

        void setMode(bmp085_mode_t mode) {
            m_mode = mode;
            readParameters();
        }

        static const char* tag;

    private:
        void readParameters();
        long read24Bit(uint8_t reg);
        long read16Bit(uint8_t reg);
        i2c_cmd_handle_t prepare_read(uint8_t reg);
        bmp085_mode_t m_mode;
        i2c_port_t m_i2cPort;

double m_temp;
        double m_pressure;
        int16_t m_AC1, m_AC2, m_AC3, m_B1, m_B2, m_MC, m_MD;
        uint16_t m_AC4, m_AC5, m_AC6;
        double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;
};

#endif
