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

    /*
    m_AC1 = 408;
    m_AC2 = -72;
    m_AC3 = -14383;
    m_AC4 = 32741;
    m_AC5 = 32757;
    m_AC6 = 23153;
    m_B1 = 6190;
    m_B2 = 4;
    m_MC = -8711;
    m_MD = 2868;
    //*/
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
    //UT = 27898;

    x1 = (UT - m_AC6) * m_AC5 / (1 << 15);
    x2 = m_MC * (1 << 11) / (x1 + m_MD);
    ESP_LOGD(tag, "x1 = %ld, x2 = %ld", x1, x2);

    b5 = x1 + x2;
    ESP_LOGD(tag, "b5 = %ld", b5);
    m_temp = (b5 + 8) / (1 << 4);
    return m_temp / 10;
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
    long pu, b6, b3, x3, p;
    unsigned long b4, b7;

    pu = read24Bit(BMP085_REGISTER_PRESSUREDATA) >> (8 - m_mode);
    //pu = 23843;
    //m_mode = BMP085_MODE_ULTRALOWPOWER;

    b6 = b5 - 4000;
    ESP_LOGD(tag, "b5 = %ld, b6 = %ld", b5, b6);
    x1 = (m_B2 * (b6 * b6 / (1 << 12))) / (1 << 11);
    x2 = m_AC2 * b6 / (1 << 11);
    x3 = x1 + x2;
    ESP_LOGD(tag, "x1 = %ld, x2 = %ld, x3 = %ld", x1, x2, x3);
    b3 = (((m_AC1 * 4 + x3) << m_mode) + 2) / 4;
    ESP_LOGD(tag, "b3 = %ld", b3);
    x1 = m_AC3 * b6 / (1 << 13);
    x2 = (m_B1 * (b6 * b6 / (1 << 12))) / (1 << 16);
    x3 = ((x1 + x2) + 2) / (1 << 2);
    ESP_LOGD(tag, "x1 = %ld, x2 = %ld, x3 = %ld", x1, x2, x3);
    b4 = m_AC4 * (unsigned long)(x3 + 32768) / (1 << 15);
    ESP_LOGD(tag, "b4 = %ld", b4);
    b7 = ((unsigned long)pu - b3) * (50000 >> m_mode);
    ESP_LOGD(tag, "b7 = %ld", b7);
    if (b7 < 0x80000000) {
        p = (b7 * 2) / b4;
    } else {
        p = (b7 / b4) * 2;
    }
    ESP_LOGD(tag, "p = %ld", p);
    x1 = (p / (1 << 8)) * (p / (1 << 8));
    ESP_LOGD(tag, "x1 = %ld", x1);
    x1 = (x1 * 3038) / (1 << 16);
    ESP_LOGD(tag, "x1 = %ld", x1);
    x2 = (-7357 * p) / (1 << 16);
    ESP_LOGD(tag, "x2 = %ld", x2);
    p = p + (x1 + x2 + 3791) / (1 << 4);
    ESP_LOGD(tag, "p = %ld", p);
    m_pressure = p;

    //return m_pressure / 100;
    return m_pressure / (pow((1.0 - (115.0 / 44330.0)), 5.255)) / 100;
}
