#include "Lidar.h"

Lidar::Lidar() {}

Lidar::~Lidar() {}

void Lidar::configure(unsigned char config)
{
    unsigned char sig_count_max;
    unsigned char acq_config_reg;
    unsigned char ref_count_max;
    unsigned char threshold_bypass;

    switch (config)
    {
    case 0: // Default mode, balanced performance
        sig_count_max = 0x80;
        acq_config_reg = 0x08;
        ref_count_max = 0x05;
        threshold_bypass = 0x00;
        break;
    case 1: // Short range, high speed
        sig_count_max = 0x1d;
        acq_config_reg = 0x08;
        ref_count_max = 0x03;
        threshold_bypass = 0x00;
        break;
    case 2: // Default range, higher speed short range
        sig_count_max = 0x80;
        acq_config_reg = 0x00;
        ref_count_max = 0x03;
        threshold_bypass = 0x00;
        break;
    case 3: // Maximum range
        sig_count_max = 0xff;
        acq_config_reg = 0x08;
        ref_count_max = 0x05;
        threshold_bypass = 0x00;
        break;

    case 4: // High sensitivity detection, high erroneous measurements
        sig_count_max = 0x80;
        acq_config_reg = 0x08;
        ref_count_max = 0x05;
        threshold_bypass = 0x80;
        break;
    case 5: // Low sensitivity detection, low erroneous measurements
        sig_count_max = 0x80;
        acq_config_reg = 0x08;
        ref_count_max = 0x05;
        threshold_bypass = 0xb0;
        break;
    case 6: // Short range, high speed, higher error
        sig_count_max = 0x04;
        acq_config_reg = 0x01;
        ref_count_max = 0x03;
        threshold_bypass = 0x00;
        break;

    default: // Default mode, balanced performance
        sig_count_max = 0x80;
        acq_config_reg = 0x08;
        ref_count_max = 0x05;
        threshold_bypass = 0x00;
        break;
    }

    write((uint8_t)0x02, &sig_count_max, (uint16_t)1);
    write((uint8_t)0x04, &acq_config_reg, (uint16_t)1);
    write((uint8_t)0x12, &ref_count_max, (uint16_t)1);
    write((uint8_t)0x1c, &threshold_bypass, (uint16_t)1);
}

/*------------------------------------------------------------------------------
  Take Range
  Initiate a distance measurement by writing to register 0x00.
  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void Lidar::take_range()
{
    uint8_t data_byte = 0x01;
    write((uint8_t)0x00, &data_byte, (uint16_t)1);
}

/*------------------------------------------------------------------------------
  Wait for Busy Flag
  Blocking function to wait until the Lidar Lite's internal busy flag goes low
  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void Lidar::wait_for_busy()
{
    uint16_t busy_counter = 0; // busy_counter counts number of times busy flag is checked, for timeout
    uint8_t busy_flag = 1;     // busy_flag monitors when the deivce is done with a measurement

    while (busy_flag) // Loop until device is not busy
    {
        // Handle timeout condition, exit while loop goto bailout
        if (busy_counter > 9999)
        {
            break;
        }

        busy_flag = get_busy_flag();

        // Increment busy_counter for timeout
        busy_counter++;
    }

    // bailout reports error over serial
    if (busy_counter > 9999)
    {
        Serial.println("> bailing out of wait_for_busy()");
    }
}

uint8_t Lidar::get_busy_flag()
{
    uint8_t busy_flag; // busy_flag monitors when the device is done with a measurement

    // Read status register to check busy flag
    read((uint8_t)0x01, &busy_flag, (uint16_t)1);
}

/*------------------------------------------------------------------------------
  Write
  Perform I2C write to device. The I2C peripheral in the LidarLite v3 HP
  will receive multiple bytes in one I2C transmission. The first byte is
  always the register address. The the bytes that follow will be written
  into the specified register address first and then the internal address
  in the Lidar Lite will be auto-incremented for all following bytes.
  Parameters
  ------------------------------------------------------------------------------
  regAddr:   register address to write to
  dataBytes: pointer to array of bytes to write
  numBytes:  number of bytes in 'dataBytes' array to write
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void Lidar::write(uint8_t reg_addr, uint8_t *data_bytes, uint16_t num_bytes)
{
    int nack_catcher = 0;

    Wire.beginTransmission(LIDAR_DEFAULT_ADDR);

    // First byte of every write sets the Lidar internal register address pointer
    Wire.write(reg_addr);

    // Subsequent bytes are data writes
    Wire.write(data_bytes, (size_t)num_bytes);

    // A nack means the device is not responding. Report the error over serial.
    nack_catcher = Wire.endTransmission();
    if (nack_catcher != 0)
    {
        Serial.println("> nack");

        delayMicroseconds(100);
    }
}

/*------------------------------------------------------------------------------
  Read
  Perform I2C read from device.  The I2C peripheral in the LidarLite v3 HP
  will send multiple bytes in one I2C transmission. The register address must
  be set up by a previous I2C write. The bytes that follow will be read
  from the specified register address first and then the internal address
  pointer in the Lidar Lite will be auto-incremented for following bytes.
  Will detect an unresponsive device and report the error over serial.
  Parameters
  ------------------------------------------------------------------------------
  regAddr:   register address to write to
  dataBytes: pointer to array of bytes to write
  numBytes:  number of bytes in 'dataBytes' array to write
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------ */
void Lidar::read(uint8_t reg_addr, uint8_t *data_bytes, uint16_t num_bytes)
{
    unsigned short i = 0;
    int nack_catcher = 0;

    // Set the interial register address pointer in the Lidar Lite
    Wire.beginTransmission(LIDAR_DEFAULT_ADDR);
    Wire.write(reg_addr);

    // A nack means the device is not responding, report the error over serial
    nack_catcher = Wire.endTransmission(false);
    if (nack_catcher != 0)
    {
        Serial.println("> nack");
    }

    // Perform read. save in data_bytes array
    Wire.requestFrom(LIDAR_DEFAULT_ADDR, num_bytes);
    if (num_bytes <= Wire.available())
    {
        while (i < num_bytes)
        {
            data_bytes[i] = Wire.read();
            i++;
        }
    }
}