#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include <stdbool.h>

// MPU-9250 9-DoF IMU (accel + gyro + AK8963 magnetometer), read directly
// over I2C register access (no external library dependency).
//
// Shares the I2C bus with the ToF sensors (sensor.h) - SDA/SCL are wired
// together across every I2C device, so whichever module calls Wire.begin()
// first sets the bus up for everyone else (safe to call again with the
// same pins).
//
// Suggested pins (avoid strapping pins 0/2/12/15 and pins already used by
// motor.h/sensor.h/battery.h/encoder.h):
//   INT -> GPIO 13  (data-ready interrupt; wired/reserved for later - this
//                    driver currently polls registers in mpu9250_read()
//                    instead of using it)
//   AD0 -> tie to GND on the breakout board for address 0x68 (used below);
//          tie to VCC instead for 0x69 and update MPU9250_I2C_ADDR to match
#define MPU9250_I2C_SDA  21   // shared with sensor.h
#define MPU9250_I2C_SCL  22   // shared with sensor.h
#define MPU9250_INT_PIN  13
#define MPU9250_I2C_ADDR 0x68 // AD0 tied low

typedef struct {
    float accel_g[3];   // x, y, z, in g
    float gyro_dps[3];  // x, y, z, in degrees/second
    float mag_ut[3];    // x, y, z, in microtesla
    float temp_c;
} mpu9250_data_t;

// Brings up I2C (if not already), verifies the MPU9250's WHO_AM_I, wakes
// it, configures accel/gyro full-scale ranges + low-pass filtering,
// enables I2C bypass so the AK8963 magnetometer (a separate chip inside
// the same package) is reachable at its own address, and starts it in
// continuous measurement mode. Returns false if either chip doesn't
// respond as expected.
bool mpu9250_init(void);

// Reads accel/gyro/temp (always fresh) and the magnetometer (only when
// the AK8963 reports new data ready - otherwise the previous mag reading
// is kept), converts everything to physical units, and logs the result.
// Returns false only on an I2C read failure.
bool mpu9250_read(mpu9250_data_t *out);

#endif // MPU9250_H
