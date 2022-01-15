#include "BMP280.h"

#include <stddef.h>
#include <util/delay.h>

typedef struct BMP280_calib_t {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} BMP280_calib;

static TWI_write_reg_cb_t    write_reg      = NULL;
static TWI_read_reg_burst_cb read_reg_burst = NULL;
static BMP280_calib calib;

void register_BMP_cb(TWI_write_reg_cb_t write, TWI_read_reg_burst_cb read) {
    if (!write || !read) return;

    write_reg = write;
    read_reg_burst = read;
}

uint8_t BMP280_init() {
    uint8_t buf = 0;
    if (!write_reg || ! read_reg_burst) return 1;

    read_reg_burst(&buf, BMP280_I2C_ADDRESS, BMP280_REGISTER_CHIPID, 1);  // should return 0x58
    if (BMP280_CHIPID != buf) return buf;

    uint8_t* cal = (uint8_t*)&calib;
    write_reg(BMP280_I2C_ADDRESS, BMP280_REGISTER_SOFTRESET, 0xB6); //reset
    _delay_ms(100);
    write_reg(BMP280_I2C_ADDRESS, BMP280_REGISTER_CONTROL, 0xFF); //high resolution and normal mode
    write_reg(BMP280_I2C_ADDRESS, BMP280_REGISTER_CONFIG, 0x1C);  // fast measurement, best filtering
    
    read_reg_burst(cal,BMP280_I2C_ADDRESS,BMP280_REGISTER_DIG_T1, BMP280_CALIB_SIZE);

    return 0;
}

void BMP280_get_data(BMP280_final* final) {
    int32_t t_fine, adc_T, adc_P;
    int64_t var1, var2, p;
    uint8_t data[6];
    
    //readout raw data
    read_reg_burst(data, BMP280_I2C_ADDRESS, BMP280_REGISTER_PRESSUREDATA, 6);
    adc_P = (((uint32_t)data[0] << 16) | ((uint16_t)data[1] << 8) | data[2]) >> 4;
    adc_T = (((uint32_t)data[3] << 16) | ((uint16_t)data[4] << 8) | data[5]) >> 4;
    
    // calculate temperature from raw
    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    final->temp = (t_fine * 5 + 128);  // celsius FP32 S24.8 = x/256/100 F
    
    // //calculate pressure from raw in Pa
    // var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
    // var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)calib.dig_P6);
    // var2 = var2 + ((var1*((int32_t)calib.dig_P5))<<1);
    // var2 = (var2>>2)+(((int32_t)calib.dig_P4)<<16);
    // var1 = (((calib.dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)calib.dig_P2) * var1)>>1))>>18;
    // var1 =((((32768+var1))*((int32_t)calib.dig_P1))>>15);

    // if (var1 == 0) {
    //     return; // avoid exception caused by division by zero
    // }

    // p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;

    // if (p < 0x80000000) {
    //     p = (p << 1) / ((uint32_t)var1);
    // } else {
    //     p = (p / (uint32_t)var1) * 2;
    // }

    // var1 = (((int32_t)calib.dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
    // var2 = (((int32_t)(p>>2)) * ((int32_t)calib.dig_P8))>>13;
    // final->baro = ((uint32_t)((int32_t)p + ((var1 + var2 + calib.dig_P7) >> 4)));  result in Pascal

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;

    if (!var1) return; // avoid exception caused by division by zero

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    final->baro = (uint32_t)p;  // result in FP S24.8 Pascal  Pa = x / 256
}
