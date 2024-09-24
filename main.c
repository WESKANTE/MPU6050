#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

// Dirección del dispositivo en el bus I2C
static int addr = 0x68;

// Definir pines para SDA y SCL
#define I2C_SDA_PIN 8  // Pin SDA
#define I2C_SCL_PIN 9  // Pin SCL

// Definir variables globales para los offsets
static int16_t accel_offset_x = 0;
static int16_t accel_offset_y = 0;
static int16_t accel_offset_z = 0;
static int16_t gyro_offset_x = 0;
static int16_t gyro_offset_y = 0;
static int16_t gyro_offset_z = 0;

// Número de muestras para la calibración
#define CALIBRATION_SAMPLES 1000

//Envía un comando al MPU6050 para restablecerlo escribiendo 0x00 en 0x6B
static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}
//Lee los datos crudos (raw) del MPU usando el bus I2C
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;

    // Leer acelerómetro desde registro 0x3B
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Aplicar los offsets al acelerómetro
    accel[0] -= accel_offset_x;
    accel[1] -= accel_offset_y;
    accel[2] -= accel_offset_z;

    // Leer giroscopio a partir del registro 0x43
    val = 0x43;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Aplicar los offsets al acelerómetro
    gyro[0] -= gyro_offset_x;
    gyro[1] -= gyro_offset_y;
    gyro[2] -= gyro_offset_z;

    // Leer temperatura
    val = 0x41;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);
    *temp = buffer[0] << 8 | buffer[1];
}

// Función para calibrar los offsets del acelerómetro
static void calibrate_accelerometer() {
    int32_t accel_sum[3] = {0, 0, 0};
    int32_t gyro_sum[3] = {0, 0, 0};
    int16_t acceleration[3], gyro[3], temp;

    printf("Calibrating accelerometer...\n");

    // Tomar varias muestras y sumarlas
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        mpu6050_read_raw(acceleration, gyro, &temp);
        accel_sum[0] += acceleration[0];
        accel_sum[1] += acceleration[1];
        accel_sum[2] += acceleration[2];
        gyro_sum[0] += gyro[0];
        gyro_sum[1] += gyro[1];
        gyro_sum[2] += gyro[2];
        sleep_ms(1);  // Pequeña pausa para no saturar la lectura
    }

    // Promediar las muestras
    accel_offset_x = accel_sum[0] / CALIBRATION_SAMPLES;
    accel_offset_y = accel_sum[1] / CALIBRATION_SAMPLES;
    accel_offset_z = accel_sum[2] / CALIBRATION_SAMPLES;

    gyro_offset_x = gyro_sum[0] / CALIBRATION_SAMPLES;
    gyro_offset_y = gyro_sum[1] / CALIBRATION_SAMPLES;
    gyro_offset_z = gyro_sum[2] / CALIBRATION_SAMPLES;

    printf("Calibration complete!\n");
    printf("Offsets: X = %d, Y = %d, Z = %d\n", accel_offset_x, accel_offset_y, accel_offset_z);
}


int main() {
    stdio_init_all();
    printf("Hello, MPU6050! Reading raw data from registers...\n");

    // Inicializar I2C en los pines personalizados
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));

    mpu6050_reset();

    // Calibrar el acelerómetro
    calibrate_accelerometer();

    int16_t acceleration[3], gyro[3], temp;

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);
        printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        //printf("Temp. = %f\n", (temp / 340.0) + 36.53);
        sleep_ms(100);
    }
}
