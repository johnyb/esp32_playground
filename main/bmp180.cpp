#include "bmp180.h"
#include "driver/i2c.h"
#include "esp_log.h"

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
	m_UT = read16Bit(BMP085_REGISTER_TEMPDATA);

    int32_t X1 = (m_UT - (int32_t)m_AC6) * ((int32_t)m_AC5) >> 15;
    int32_t X2 = ((int32_t)m_MC << 11) / (X1 + (int32_t)m_MD);
    int32_t B5 =  X1 + X2;
    return ((B5 + 8) >> 4) / 10.0;
}

int32_t BMP180::readPressure() {
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

    int32_t UP = read24Bit(BMP085_REGISTER_PRESSUREDATA) >> (8 - m_mode);
    int32_t X1 = (m_UT - (int32_t)m_AC6) * ((int32_t)m_AC5) >> 15;
    int32_t X2 = ((int32_t)m_MC << 11) / (X1 + (int32_t)m_MD);
    int32_t B5 =  X1 + X2;
    int32_t B6 = B5 - 4000;
    X1 = (m_B2 * (B6 * B6) >> 12) >> 11;
    X2 = (m_AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = (((((int32_t)m_AC1) * 4 + X3) << m_mode) + 2) >> 2;
    X1 = (m_AC3 * B6) >> 13;
    X2 = (m_B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = ((uint32_t)m_AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((((uint32_t)UP) - B3) * (uint32_t)(50000UL >> m_mode));
    int32_t P;
    if (B7 < 0x80000000) {
        P = (B7 << 1) / B4;
    } else {
        P = (B7 / B4) << 1;
    }
    X1 = (P >> 8) * (P >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P) >> 16;

	return P + ((X1 + X2 + (int32_t)3791) >> 4);
}
