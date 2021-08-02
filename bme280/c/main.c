#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

static const char* dev_name = "/dev/i2c-1";
uint8_t dev_addr = 0x76;

int fd;
int digT[3], digP[9], digH[6];
double t_fine;

int i2c_read(uint8_t reg_addr, uint8_t* data, uint16_t length) {
    struct i2c_msg messages[] = {
        { dev_addr, 0, 1, &reg_addr },
        { dev_addr, I2C_M_RD, length, data },
    };
    struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };

    if (ioctl(fd, I2C_RDWR, &ioctl_data) != 2) {
        fprintf(stderr, "i2c_read: failed to ioctl: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

int i2c_write(uint8_t reg_addr, const uint8_t* data, uint16_t length) {
    uint8_t* buffer = (uint8_t*)malloc(length + 1);
    if (buffer == NULL) {
        fprintf(stderr, "i2c_write: failed to memory allocate\n");
        return -1;
    }
    buffer[0] = reg_addr;
    memcpy(&buffer[1], data, length);

    struct i2c_msg message = { dev_addr, 0, length + 1, buffer };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };

    if (ioctl(fd, I2C_RDWR, &ioctl_data) != 1) {
        fprintf(stderr, "i2c_write: failed to ioctl: %s\n", strerror(errno));
        free(buffer);
        return -1;
    }

    free(buffer);
    return 0;
}

void get_calib_param() {
    uint8_t calib[32] = {};
    int i;
    
    i2c_read(0x88, calib, 24);
    i2c_read(0xA1, &calib[24], 1);
    i2c_read(0xE1, &calib[25], 7);

    digT[0] = ((calib[1] << 8) | calib[0]);
    digT[1] = ((calib[3] << 8) | calib[2]);
    digT[2] = ((calib[5] << 8) | calib[4]);
    digP[0] = ((calib[7] << 8) | calib[6]);
    digP[1] = ((calib[9] << 8) | calib[8]);
    digP[2] = ((calib[11]<< 8) | calib[10]);
    digP[3] = ((calib[13]<< 8) | calib[12]);
    digP[4] = ((calib[15]<< 8) | calib[14]);
    digP[5] = ((calib[17]<< 8) | calib[16]);
    digP[6] = ((calib[19]<< 8) | calib[18]);
    digP[7] = ((calib[21]<< 8) | calib[20]);
    digP[8] = ((calib[23]<< 8) | calib[22]);
    digH[0] = ( calib[24] );
    digH[1] = ((calib[26]<< 8) | calib[25]);
    digH[2] = ( calib[27] );
    digH[3] = ((calib[28]<< 4) | (0x0F & calib[29]));
    digH[4] = ((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F));
    digH[5] = ( calib[31] );
    
    for (i = 1; i < 2; i++) if (digT[i] & 0x8000) digT[i] = (-digT[i] ^ 0xFFFF) + 1;
    for (i = 1; i < 8; i++) if (digP[i] & 0x8000) digP[i] = (-digP[i] ^ 0xFFFF) + 1;
    for (i = 0; i < 6; i++) if (digH[i] & 0x8000) digH[i] = (-digH[i] ^ 0xFFFF) + 1;
}

double compensate_P(uint32_t adc_P)  {
    double pressure = 0.0;

    double v1 = (t_fine / 2.0) - 64000.0;
    double v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5];
    v2 = v2 + ((v1 * digP[4]) * 2.0);
    v2 = (v2 / 4.0) + (digP[3] * 65536.0);
    v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((digP[1] * v1) / 2.0)) / 262144;
    v1 = ((32768 + v1) * digP[0]) / 32768;

    if (v1 == 0) return 0;
    pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125;
    if (pressure < 0x80000000) pressure = (pressure * 2.0) / v1;
    else pressure = (pressure / v1) * 2;
    v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096;
    v2 = ((pressure / 4.0) * digP[7]) / 8192.0;

    pressure = pressure + ((v1 + v2 + digP[6]) / 16.0);
    return pressure / 100;
}

double compensate_T(uint32_t adc_T)  {
    double v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1];
    double v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2];
    t_fine = v1 + v2;
    double temperature = t_fine / 5120.0;
    return temperature;
}

double compensate_H(uint32_t adc_H)  {
    double var_h = t_fine - 76800.0;
    if (var_h == 0) return 0;
    var_h = (adc_H - (digH[3] * 64.0 + digH[4]/16384.0 * var_h)) * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h * (1.0 + digH[2] / 67108864.0 * var_h)));

    if (var_h > 100.0) return 100.0;
    else if (var_h < 0.0) return 0.0;
    else return var_h;
}

void read_data(double *temp, double *pres, double *hum) {
    uint8_t data[8] = {};
    i2c_read(0xF7, data, 8);

    uint32_t pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    uint32_t temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    uint32_t hum_raw  = (data[6] << 8)  |  data[7];
    
    *temp = compensate_T(temp_raw);
    *pres = compensate_P(pres_raw);
    *hum = compensate_H(hum_raw);
}

void setup() {
    uint8_t osrs_t = 1;            // Temperature oversampling x 1
    uint8_t osrs_p = 1;          // Pressure oversampling x 1
    uint8_t osrs_h = 1;            // Humidity oversampling x 1
    uint8_t mode   = 3;            // Normal mode
    uint8_t t_sb   = 5;            // Tstandby 1000ms
    uint8_t filter = 0;            // Filter off
    uint8_t spi3w_en = 0;        // 3-wire SPI Disable

    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;

    i2c_write(0xF2, &ctrl_hum_reg, 1);
    i2c_write(0xF4, &ctrl_meas_reg, 1);
    i2c_write(0xF5, &config_reg, 1);
}

int main() {
    fd = open(dev_name, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open i2c: %s\n", strerror(errno));
        return INT_MIN;
    }

    setup();
    get_calib_param();

    double temp, pres, hum;
    read_data(&temp, &pres, &hum);

    printf("temp: %-6.2f°C\n", temp);
    printf("pressure : %7.2f hPa\n", pres);
    printf("hum : %6.2f %%\n", hum);

    close(fd);
    return 0;
}
