#ifndef LIDAR_H
#define LIDAR_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

// Lidar_Lite_HP_V3 default I2C device address.
const int LIDAR_DEFAULT_ADDR = 0x62;

class Lidar
{
private:
public:
    Lidar();
    ~Lidar();
    void configure(uint8_t config);
    unsigned short read_distance();
    void take_range();
    void wait_for_busy();
    uint8_t get_busy_flag();
    void write(uint8_t reg_addr, uint8_t *data_bytes, uint16_t num_bytes);
    void read(uint8_t &reg_addr, uint8_t *data_bytes, uint16_t &num_bytes);
};

#endif