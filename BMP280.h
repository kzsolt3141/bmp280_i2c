#ifndef BMP280_H_
#define BMP280_H_

#include <stdint.h>

#define BMP280_I2C_ADDRESS            (0x76)
#define BMP280_CHIPID                 (0x58)

#define BMP280_REGISTER_DIG_T1         (0x88)
#define BMP280_REGISTER_DIG_T2         (0x8A)
#define BMP280_REGISTER_DIG_T3         (0x8C)
#define BMP280_REGISTER_DIG_P1         (0x8E)
#define BMP280_REGISTER_DIG_P2         (0x90)
#define BMP280_REGISTER_DIG_P3         (0x92)
#define BMP280_REGISTER_DIG_P4         (0x94)
#define BMP280_REGISTER_DIG_P5         (0x96)
#define BMP280_REGISTER_DIG_P6         (0x98)
#define BMP280_REGISTER_DIG_P7         (0x9A)
#define BMP280_REGISTER_DIG_P8         (0x9C)
#define BMP280_REGISTER_DIG_P9         (0x9E)
#define BMP280_REGISTER_CHIPID         (0xD0)
#define BMP280_REGISTER_VERSION        (0xD1)
#define BMP280_REGISTER_SOFTRESET      (0xE0)
#define BMP280_REGISTER_CAL26          (0xE1)  // R calibration stored in 0xE1-0xF0
#define BMP280_REGISTER_CONTROL        (0xF4)
#define BMP280_REGISTER_CONFIG         (0xF5)
#define BMP280_REGISTER_PRESSUREDATA   (0xF7)
#define BMP280_REGISTER_TEMPDATA       (0xFA)

#define BMP280_CALIB_SIZE              (24)

typedef struct BMP280_final_t {
    int32_t baro;  // Pa S24.8
    int32_t temp;  // Celsius/100 S24.8
} BMP280_final;

uint8_t BMP280_init();
void BMP280_get_data(BMP280_final* final);

typedef uint8_t (*TWI_write_reg_cb_t)(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);
typedef uint8_t (*TWI_read_reg_burst_cb)(uint8_t *buffer, uint8_t slave_addr, uint8_t reg_addr, uint8_t size);
void register_BMP_cb(TWI_write_reg_cb_t write, TWI_read_reg_burst_cb read);

#endif /* BMP280_H_ */
