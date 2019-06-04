#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "driver/i2c.h"


//configure
#define BME280_REGISTER_SOFTRESET 0xE0
#define i2caddr 0x76 
#define BME280_REGISTER_CONTROLHUMID 0xF2
#define BME280_REGISTER_STATUS 0XF3
#define BME280_REGISTER_CONTROL 0xF4
#define BME280_REGISTER_CONFIG  0xF5
//temperature 
#define BME280_REGISTER_TEMPDATA 0xFA // temperature register 
#define BME280_REGISTER_DIG_T1 0x88
#define BME280_REGISTER_DIG_T2 0x8A
#define BME280_REGISTER_DIG_T3 0x8C
//humidity
#define  BME280_REGISTER_HUMIDDATA 0xFD //humidty register
#define BME280_REGISTER_DIG_H2 0xE1
#define BME280_REGISTER_DIG_H3 0xE3
#define BME280_REGISTER_DIG_H4 0xE4
#define BME280_REGISTER_DIG_H5 0xE5
#define BME280_REGISTER_DIG_H6 0xE7
#define BME280_REGISTER_DIG_H1 0xA1
//pressure
#define BME280_REGISTER_PRESSUREDATA 0xF7 //pressure register 
#define BME280_REGISTER_DIG_P1 0x8E
#define BME280_REGISTER_DIG_P2 0x90
#define BME280_REGISTER_DIG_P3 0x92
#define BME280_REGISTER_DIG_P4 0x94
#define BME280_REGISTER_DIG_P5 0x96
#define BME280_REGISTER_DIG_P6 0x98
#define BME280_REGISTER_DIG_P7 0x9A
#define BME280_REGISTER_DIG_P8 0x9C
#define BME280_REGISTER_DIG_P9 0x9E
#define I2C_MASTER_ACK_EN   true    /*!< Enable ack check for master */
#define I2C_MASTER_ACK_DIS  false   /*!< Disable ack check for master */

uint16_t bme280_calib_dig_T1; ///< temperature compensation value
int16_t bme280_calib_dig_T2; ///< temperature compensation value
int16_t bme280_calib_dig_T3; ///< temperature compensation value
uint8_t bme280_calib_dig_H1; ///< humidity compensation value
int16_t bme280_calib_dig_H2; ///< humidity compensation value
uint8_t bme280_calib_dig_H3; ///< humidity compensation value
int16_t bme280_calib_dig_H4; ///< humidity compensation value
int16_t bme280_calib_dig_H5; ///< humidity compensation value
int8_t bme280_calib_dig_H6; ///< humidity compensation value
uint16_t bme280_calib_dig_P1; ///< pressure compensation value
int16_t bme280_calib_dig_P2; ///< pressure compensation value
int16_t bme280_calib_dig_P3; ///< pressure compensation value
int16_t bme280_calib_dig_P4; ///< pressure compensation value
int16_t bme280_calib_dig_P5; ///< pressure compensation value
int16_t bme280_calib_dig_P6; ///< pressure compensation value
int16_t bme280_calib_dig_P7; ///< pressure compensation value
int16_t bme280_calib_dig_P8; ///< pressure compensation value
int16_t bme280_calib_dig_P9; ///< pressure compensation value


constexpr uint32_t kDefaultClockSpeed = 100000;  /*!< Clock speed in Hz, default: 100KHz */
constexpr uint32_t kDefaultTimeout = 1000;       /*!< Timeout in milliseconds, default: 1000ms */
i2c_port_t port;            /*!< I2C port: I2C_NUM_0 or I2C_NUM_1 */
uint32_t ticksToWait;       /*!< Timeout in ticks for read and write */

esp_err_t begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed = kDefaultClockSpeed);

esp_err_t begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en,
                    uint32_t clk_speed = kDefaultClockSpeed);
esp_err_t close();
void setTimeout(uint32_t ms);
esp_err_t writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data, int32_t timeout = -1);
esp_err_t writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data, int32_t timeout = -1);
esp_err_t writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data, int32_t timeout = -1);
esp_err_t writeBytes(uint8_t devAddr, uint8_t regAddr, size_t length, const uint8_t *data, int32_t timeout = -1);

esp_err_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, int32_t timeout = -1);
esp_err_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, int32_t timeout = -1);
esp_err_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, int32_t timeout = -1);
esp_err_t readBytes(uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *data, int32_t timeout = -1);
esp_err_t testConnection(uint8_t devAddr, int32_t timeout = -1);

esp_err_t testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, (timeout < 0 ? ticksToWait : pdMS_TO_TICKS(timeout)));
    i2c_cmd_link_delete(cmd);
    return err;
}



esp_err_t begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed) {
    return begin(sda_io_num, scl_io_num, GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE, clk_speed);
}

esp_err_t begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num, gpio_pullup_t sda_pullup_en,
        gpio_pullup_t scl_pullup_en, uint32_t clk_speed) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_num;
    conf.sda_pullup_en = sda_pullup_en;
    conf.scl_io_num = scl_io_num;
    conf.scl_pullup_en = scl_pullup_en;
    conf.master.clk_speed = clk_speed;
    esp_err_t err = i2c_param_config(port, &conf);
    if (!err) err = i2c_driver_install(port, conf.mode, 0, 0, 0);
    return err;
}

void setTimeout(uint32_t ms) {
    ticksToWait = pdMS_TO_TICKS(ms);
}

void scanner() {
    constexpr int32_t scanTimeout = 20;
    printf(LOG_COLOR_W "\n>> I2C scanning ..." LOG_RESET_COLOR "\n");
    uint8_t count = 0;
    for (size_t i = 0x3; i < 0x78; i++) {
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf(LOG_COLOR_W "- Device found at address 0x%X%s", i, LOG_RESET_COLOR "\n");
            count++;
        }
    }
    if (count == 0)
        printf(LOG_COLOR_E "- No I2C devices found!" LOG_RESET_COLOR "\n");
    printf("\n");
}

esp_err_t writeBytes(uint8_t devAddr, uint8_t regAddr, size_t length, const uint8_t *data, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    i2c_master_write_byte(cmd, regAddr, I2C_MASTER_ACK_EN);
    i2c_master_write(cmd, (uint8_t*) data, length, I2C_MASTER_ACK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, (timeout < 0 ? ticksToWait : pdMS_TO_TICKS(timeout)));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data, int32_t timeout) {
    return writeBytes(devAddr, regAddr, 1, &data, timeout);
}

esp_err_t readBytes(uint8_t devAddr, uint8_t regAddr, size_t length, uint8_t *data, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
    i2c_master_write_byte(cmd, regAddr, I2C_MASTER_ACK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_READ, I2C_MASTER_ACK_EN);
    i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, (timeout < 0 ? ticksToWait : pdMS_TO_TICKS(timeout)));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, int32_t timeout) {
    return readBytes(devAddr, regAddr, 1, data, timeout);
}

uint8_t read8(uint8_t reg) {
    uint8_t value;
    readByte(i2caddr, reg, &value);
    return value;
}

uint16_t read16(uint8_t reg)
{
    uint16_t value16;
    uint8_t value8[2];
    readBytes(i2caddr, reg, 2, value8);
    value16 = (value8[0] << 8) | value8[1];
    return value16;
}

uint16_t read16_LE(uint8_t reg) {
    uint16_t temp = read16(reg);
    return (temp >> 8) | (temp << 8);
}

int16_t readS16_LE(uint8_t reg)
{
    return (int16_t)read16_LE(reg);
}

uint32_t read24(uint8_t reg){
    uint32_t value24;
    uint8_t value8[3];
    readBytes(i2caddr, reg, 3, value8);
    value24 = (value8[0] << 16) | (value8[1] << 8) | value8[2];
    return value24;
}

void write8(uint8_t reg, uint8_t value) {
    writeByte(i2caddr, reg, value);
}

void bme280_parameters(void){
  bme280_calib_dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
  bme280_calib_dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
  bme280_calib_dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);
  bme280_calib_dig_H1 = read8(BME280_REGISTER_DIG_H1);
  bme280_calib_dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
  bme280_calib_dig_H3 = read8(BME280_REGISTER_DIG_H3);
  bme280_calib_dig_H4 = (read8(BME280_REGISTER_DIG_H4) << 4) | (read8(BME280_REGISTER_DIG_H4+1) & 0xF);
  bme280_calib_dig_H5 = (read8(BME280_REGISTER_DIG_H5+1) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
  bme280_calib_dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
  bme280_calib_dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
  bme280_calib_dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
  bme280_calib_dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
  bme280_calib_dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
  bme280_calib_dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
  bme280_calib_dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
  bme280_calib_dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
  bme280_calib_dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
  bme280_calib_dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);
}

void setSampling() {
    write8(BME280_REGISTER_CONTROLHUMID, 5);
    write8(BME280_REGISTER_CONFIG, 0);
    write8(BME280_REGISTER_CONTROL, 183);
}

bool isReadingCalibration(void)
{
  uint8_t const rStatus = read8(BME280_REGISTER_STATUS);

  return (rStatus & (1 << 0)) != 0;
}

void bme280_config(void){
  write8(BME280_REGISTER_SOFTRESET, 0xB6);
  delay(300);
  while (isReadingCalibration())
    delay(100);
  setSampling();
  delay(100);
  bme280_parameters();
}

float readTemperature(void)
{
    int32_t var1, var2, t_fine;

    int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);

    if (adc_T == 0x800000) // value in case temp measurement was disabled
        return NAN;

    adc_T >>= 4;

    var1 = ((((adc_T>>3) - ((int32_t)bme280_calib_dig_T1 <<1))) *
            ((int32_t)bme280_calib_dig_T2)) >> 11;
             
    var2 = (((((adc_T>>4) - ((int32_t)bme280_calib_dig_T1)) *
              ((adc_T>>4) - ((int32_t)bme280_calib_dig_T1))) >> 12) *
            ((int32_t)bme280_calib_dig_T3)) >> 14;

    t_fine = var1 + var2;

    float T = (t_fine * 5 + 128) >> 8;
    return T/100;
}

float readHumidity(void) {
    int32_t t_fine;

    readTemperature(); // must be done first to get t_fine

    int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
    if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return NAN;
        
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme280_calib_dig_H4) << 20) -
                    (((int32_t)bme280_calib_dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)bme280_calib_dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)bme280_calib_dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)bme280_calib_dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)bme280_calib_dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    float h = (v_x1_u32r>>12);
    return  h / 1024.0;
}

float readPressure(void) {
    int32_t t_fine;
    int64_t var1, var2, p;

    readTemperature(); // must be done first to get t_fine

    int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
    if (adc_P == 0x800000) // value in case pressure measurement was disabled
        return NAN;
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bme280_calib_dig_P6;
    var2 = var2 + ((var1*(int64_t)bme280_calib_dig_P5)<<17);
    var2 = var2 + (((int64_t)bme280_calib_dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)bme280_calib_dig_P3)>>8) +
           ((var1 * (int64_t)bme280_calib_dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bme280_calib_dig_P1)>>33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)bme280_calib_dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)bme280_calib_dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bme280_calib_dig_P7)<<4);
    return (float)p/256;
}

void setup() {
  Serial.begin(9600);
  ESP_ERROR_CHECK(begin(GPIO_NUM_21, GPIO_NUM_22, 400000));
  setTimeout(10);
  scanner();
  bme280_config();
  delay(1000);  
}

void loop() {
  Serial.print("Temperature = ");
  Serial.print(readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(readHumidity());
  Serial.println(" %");

  Serial.println();
  delay(500);
}