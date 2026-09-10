#include "mpu9250.h"

#include <Arduino.h>
#include <Wire.h>

// ---- MPU9250 (accel/gyro) registers ----
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_INT_PIN_CFG   0x37
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

#define WHO_AM_I_MPU9250  0x71
#define WHO_AM_I_MPU9255  0x73
#define WHO_AM_I_MPU6500  0x70 // same accel/gyro core, no onboard magnetometer

// AK8963 magnetometer (its own chip, reachable once bypass is enabled)
#define AK8963_ADDR       0x0C
#define AK8963_WIA        0x00
#define AK8963_ST1        0x02
#define AK8963_HXL        0x03
#define AK8963_ST2        0x09
#define AK8963_CNTL1      0x0A
#define AK8963_ASAX       0x10
#define AK8963_WIA_VALUE  0x48

// Full-scale ranges chosen for a small, fast-spinning robot: +-4g covers
// impacts/vibration headroom, +-1000dps covers pivot-turn angular rates.
#define ACCEL_FS_SEL_4G   0x08  // AFS_SEL=1 (bits 4:3)
#define GYRO_FS_SEL_1000  0x10  // FS_SEL=2  (bits 4:3)
#define ACCEL_SENS_LSB_PER_G   8192.0f
#define GYRO_SENS_LSB_PER_DPS  32.8f
#define MAG_UT_PER_LSB         (4912.0f / 32760.0f)  // 16-bit mag output

static float s_mag_asa[3] = { 1.0f, 1.0f, 1.0f }; // per-axis sensitivity adjustment
static float s_last_mag_ut[3] = { 0.0f, 0.0f, 0.0f };
static bool s_mag_available = false; // false on an MPU6500 (no AK8963) or if it just didn't respond

static bool write_reg(uint8_t dev_addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom((int)dev_addr, (int)len) != len) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

static bool ak8963_init(void) {
    uint8_t wia = 0;
    if (!read_regs(AK8963_ADDR, AK8963_WIA, &wia, 1) || wia != AK8963_WIA_VALUE) {
        Serial.printf("[MPU9250] AK8963 WHO_AM_I mismatch (got 0x%02X, want 0x%02X)\n",
                      wia, AK8963_WIA_VALUE);
        return false;
    }

    // Fuse ROM access mode to read factory sensitivity adjustment values.
    write_reg(AK8963_ADDR, AK8963_CNTL1, 0x0F);
    delay(10);
    uint8_t asa_raw[3];
    if (!read_regs(AK8963_ADDR, AK8963_ASAX, asa_raw, 3)) {
        Serial.println("[MPU9250] AK8963 ASA read FAILED");
        return false;
    }
    for (int i = 0; i < 3; i++) {
        s_mag_asa[i] = ((asa_raw[i] - 128) * 0.5f / 128.0f) + 1.0f;
    }

    // Power down, then continuous measurement mode 2 (100Hz), 16-bit output.
    write_reg(AK8963_ADDR, AK8963_CNTL1, 0x00);
    delay(10);
    write_reg(AK8963_ADDR, AK8963_CNTL1, 0x16);
    delay(10);

    Serial.printf("[MPU9250] AK8963 init OK (asa=%.3f,%.3f,%.3f)\n",
                  s_mag_asa[0], s_mag_asa[1], s_mag_asa[2]);
    return true;
}

bool mpu9250_init(void) {
    Wire.begin(MPU9250_I2C_SDA, MPU9250_I2C_SCL);
    pinMode(MPU9250_INT_PIN, INPUT);

    // Tolerant of either chip, since it's not always obvious which one is
    // actually on a given breakout without checking the silkscreen/WHO_AM_I:
    // MPU9250/9255 have accel+gyro+mag, MPU6500 is the same accel/gyro core
    // with no onboard magnetometer.
    uint8_t who_am_i = 0;
    if (!read_regs(MPU9250_I2C_ADDR, REG_WHO_AM_I, &who_am_i, 1)
        || (who_am_i != WHO_AM_I_MPU9250 && who_am_i != WHO_AM_I_MPU9255
            && who_am_i != WHO_AM_I_MPU6500)) {
        Serial.printf("[MPU9250] init FAILED - WHO_AM_I=0x%02X (addr=0x%02X)\n",
                      who_am_i, MPU9250_I2C_ADDR);
        return false;
    }
    bool is_6500 = (who_am_i == WHO_AM_I_MPU6500);
    Serial.printf("[MPU9250] detected %s (WHO_AM_I=0x%02X)\n",
                  is_6500 ? "MPU6500 (no magnetometer)" : "MPU9250/9255", who_am_i);

    write_reg(MPU9250_I2C_ADDR, REG_PWR_MGMT_1, 0x80); // reset
    delay(100);
    write_reg(MPU9250_I2C_ADDR, REG_PWR_MGMT_1, 0x01); // wake, auto-select clock source
    delay(10);

    write_reg(MPU9250_I2C_ADDR, REG_SMPLRT_DIV, 0x00);
    write_reg(MPU9250_I2C_ADDR, REG_CONFIG, 0x03);          // gyro DLPF ~41Hz
    write_reg(MPU9250_I2C_ADDR, REG_GYRO_CONFIG, GYRO_FS_SEL_1000);
    write_reg(MPU9250_I2C_ADDR, REG_ACCEL_CONFIG, ACCEL_FS_SEL_4G);
    write_reg(MPU9250_I2C_ADDR, REG_ACCEL_CONFIG2, 0x03);   // accel DLPF ~41Hz

    if (is_6500) {
        s_mag_available = false;
    } else {
        // Bypass mode: lets us talk to the AK8963 directly at its own
        // address instead of through the MPU9250's auxiliary I2C master.
        write_reg(MPU9250_I2C_ADDR, REG_INT_PIN_CFG, 0x02);
        delay(10);

        s_mag_available = ak8963_init();
        if (!s_mag_available) {
            Serial.println("[MPU9250] magnetometer not found - continuing with accel/gyro only");
        }
    }

    Serial.printf("[MPU9250] init OK (addr=0x%02X, sda=%d, scl=%d, mag=%d)\n",
                  MPU9250_I2C_ADDR, MPU9250_I2C_SDA, MPU9250_I2C_SCL, (int) s_mag_available);
    return true;
}

bool mpu9250_read(mpu9250_data_t *out) {
    uint8_t raw[14];
    if (!read_regs(MPU9250_I2C_ADDR, REG_ACCEL_XOUT_H, raw, 14)) {
        Serial.println("[MPU9250] accel/gyro read FAILED");
        return false;
    }

    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    accel_raw[0] = (int16_t)((raw[0] << 8) | raw[1]);
    accel_raw[1] = (int16_t)((raw[2] << 8) | raw[3]);
    accel_raw[2] = (int16_t)((raw[4] << 8) | raw[5]);
    temp_raw     = (int16_t)((raw[6] << 8) | raw[7]);
    gyro_raw[0]  = (int16_t)((raw[8]  << 8) | raw[9]);
    gyro_raw[1]  = (int16_t)((raw[10] << 8) | raw[11]);
    gyro_raw[2]  = (int16_t)((raw[12] << 8) | raw[13]);

    for (int i = 0; i < 3; i++) {
        out->accel_g[i]  = accel_raw[i] / ACCEL_SENS_LSB_PER_G;
        out->gyro_dps[i] = gyro_raw[i]  / GYRO_SENS_LSB_PER_DPS;
    }
    out->temp_c = (temp_raw / 333.87f) + 21.0f;

    if (s_mag_available) {
        uint8_t st1 = 0;
        if (read_regs(AK8963_ADDR, AK8963_ST1, &st1, 1) && (st1 & 0x01)) {
            uint8_t mag_raw[7]; // 6 data bytes + ST2 (must be read to latch the data)
            if (read_regs(AK8963_ADDR, AK8963_HXL, mag_raw, 7)) {
                bool overflow = (mag_raw[6] & 0x08) != 0; // ST2 HOFL bit
                if (!overflow) {
                    int16_t mx = (int16_t)((mag_raw[1] << 8) | mag_raw[0]); // mag is little-endian
                    int16_t my = (int16_t)((mag_raw[3] << 8) | mag_raw[2]);
                    int16_t mz = (int16_t)((mag_raw[5] << 8) | mag_raw[4]);
                    s_last_mag_ut[0] = mx * MAG_UT_PER_LSB * s_mag_asa[0];
                    s_last_mag_ut[1] = my * MAG_UT_PER_LSB * s_mag_asa[1];
                    s_last_mag_ut[2] = mz * MAG_UT_PER_LSB * s_mag_asa[2];
                }
            }
        }
    }
    out->mag_ut[0] = s_last_mag_ut[0];
    out->mag_ut[1] = s_last_mag_ut[1];
    out->mag_ut[2] = s_last_mag_ut[2];

    Serial.printf("[MPU9250] accel(g)=%.2f,%.2f,%.2f gyro(dps)=%.1f,%.1f,%.1f "
                  "mag(uT)=%.1f,%.1f,%.1f temp=%.1fC\n",
                  out->accel_g[0], out->accel_g[1], out->accel_g[2],
                  out->gyro_dps[0], out->gyro_dps[1], out->gyro_dps[2],
                  out->mag_ut[0], out->mag_ut[1], out->mag_ut[2],
                  out->temp_c);

    return true;
}
