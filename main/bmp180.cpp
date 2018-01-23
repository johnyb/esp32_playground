#include "bmp180.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

const char* BMP180::tag = "BMP180";

i2c_cmd_handle_t BMP180::prepare_read(uint8_t reg) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(m_i2cPort, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

    return cmd;
}

long BMP180::read16Bit(uint8_t reg) {
    uint8_t buf[2];
    i2c_cmd_handle_t cmd = prepare_read(reg);
    i2c_master_read_byte(cmd, &buf[0], ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &buf[1], ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t status = i2c_master_cmd_begin(m_i2cPort, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (status != ESP_OK) {
        ESP_LOGE(tag, "Error reading 2 byte (Code %d)", status);
    }

    return buf[0] << 8 | buf[1];
}

long BMP180::read24Bit(uint8_t reg) {
    uint8_t buf[3];
    i2c_cmd_handle_t cmd = prepare_read(reg);
    i2c_master_read_byte(cmd, &buf[0], ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &buf[1], ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &buf[2], ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t status = i2c_master_cmd_begin(m_i2cPort, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (status != ESP_OK) {
        ESP_LOGE(tag, "Error reading pressure (Code %d)", status);
    }

    return (int32_t)(buf[0] << 16 | buf[1] << 8 | buf[2]);
}

void BMP180::readParameters() {
    m_AC1 = read16Bit(BMP085_REGISTER_CAL_AC1);
    m_AC2 = read16Bit(BMP085_REGISTER_CAL_AC2);
    m_AC3 = read16Bit(BMP085_REGISTER_CAL_AC3);
    m_AC4 = read16Bit(BMP085_REGISTER_CAL_AC4);
    m_AC5 = read16Bit(BMP085_REGISTER_CAL_AC5);
    m_AC6 = read16Bit(BMP085_REGISTER_CAL_AC6);
    m_B1 = read16Bit(BMP085_REGISTER_CAL_B1);
    m_B2 = read16Bit(BMP085_REGISTER_CAL_B2);
    m_MC = read16Bit(BMP085_REGISTER_CAL_MC);
    m_MD = read16Bit(BMP085_REGISTER_CAL_MD);

    double c3, c4, b1;
    c3 = 160.0 * pow(2,-15) * m_AC3;
    c4 = pow(10,-3) * pow(2,-15) * m_AC4;
    b1 = pow(160,2) * pow(2,-30) * m_B1;
    c5 = (pow(2,-15) / 160) * m_AC5;
    c6 = m_AC6;
    mc = (pow(2,11) / pow(160,2)) * m_MC;
    md = m_MD / 160.0;
    x0 = m_AC1;
    x1 = 160.0 * pow(2,-13) * m_AC2;
    x2 = pow(160,2) * pow(2,-25) * m_B2;
    y0 = c4 * pow(2,15);
    y1 = c4 * c3;
    y2 = c4 * b1;
    p0 = (3791.0 - 8.0) / 1600.0;
    p1 = 1.0 - 7357.0 * pow(2,-20);
    p2 = 3038.0 * 100.0 * pow(2,-36);
}

double BMP180::readTemperature() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, BMP085_REGISTER_CONTROL, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, BMP085_REGISTER_READTEMPCMD, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    vTaskDelay(pdMS_TO_TICKS(5));
    double UT = read16Bit(BMP085_REGISTER_TEMPDATA);

    double a = (UT - c6) * c5;
    m_temp = a + (mc / (a + md));
    return m_temp;
}

double BMP180::readPressure() {
    readTemperature();
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, BMP085_REGISTER_CONTROL, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, BMP085_REGISTER_READPRESSURECMD + (m_mode << 6), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK( i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000)) );
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    switch(m_mode) {
    case BMP085_MODE_ULTRALOWPOWER:
        vTaskDelay(pdMS_TO_TICKS(5));
        break;
    case BMP085_MODE_STANDARD:
        vTaskDelay(pdMS_TO_TICKS(8));
        break;
    case BMP085_MODE_HIGHRES:
        vTaskDelay(pdMS_TO_TICKS(14));
        break;
    case BMP085_MODE_ULTRAHIGHRES:
    default:
        vTaskDelay(pdMS_TO_TICKS(26));
        break;
    }
    double pu,s,x,y,z;

    pu = read24Bit(BMP085_REGISTER_PRESSUREDATA) >> (8 - m_mode);
    s = m_temp - 25.0;
    x = (x2 * pow(s,2)) + (x1 * s) + x0;
    y = (y2 * pow(s,2)) + (y1 * s) + y0;
    z = (pu - x) / y;
    m_pressure = (p2 * pow(z,2)) + (p1 * z) + p0;

    return m_pressure;
}
