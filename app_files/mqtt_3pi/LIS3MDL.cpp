#include "LIS3MDL.h"

// Defines ////////////////////////////////////////////////////////////////

// The mbed two-wire interface uses a 8-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define LIS3MDL_SA1_HIGH_ADDRESS  0b00111100
#define LIS3MDL_SA1_LOW_ADDRESS   0b00111000

#define TEST_REG_ERROR -1

//I2C pins and frequency
#define DEFAULT_SDA   p9
#define DEFAULT_SCL   p10
#define DEFAULT_FREQUENCY 10000

#define LIS3MDL_WHO_ID  0x3D

//Divisor to convert between raw values and gauss
#define M_DIVISOR  6842.0

#define DEBUG   0

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

extern Serial pc;

// Constructors ////////////////////////////////////////////////////////////////

LIS3MDL::LIS3MDL(void)
{
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
  //Initialize correction and scale to additive and multiplicative identity
  correction = {0, 0, 0};
  scale = {1.0, 1.0, 1.0};

  //Initialize I2C Master interface with default pins
  master = new I2C(DEFAULT_SDA, DEFAULT_SCL);
  //Set Frequency to default (10 kHz)
  master->frequency(DEFAULT_FREQUENCY);
}

LIS3MDL::LIS3MDL(PinName sda, PinName scl)
{
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
  //Initialize correction and scale to additive and multiplicative identity
  correction = {0, 0, 0};
  scale = {1.0, 1.0, 1.0};

  //Initialize I2C Master interface with default pins
  master = new I2C(sda, scl);
  //Set Frequency to default (10 kHz)
  master->frequency(DEFAULT_FREQUENCY);
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in read() since the last call to timeoutOccurred()?
bool LIS3MDL::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void LIS3MDL::setTimeout(uint16_t timeout)
{
  io_timeout = timeout;
}

uint16_t LIS3MDL::getTimeout()
{
  return io_timeout;
}

bool LIS3MDL::init(deviceType device, sa1State sa1)
{
  // perform auto-detection unless device type and SA1 state were both specified
  if (device == device_auto || sa1 == sa1_auto)
  {
    // check for LIS3MDL if device is unidentified or was specified to be this type
    if (device == device_auto || device == device_LIS3MDL)
    {
      // check SA1 high address unless SA1 was specified to be low
      if (sa1 != sa1_low && testReg(LIS3MDL_SA1_HIGH_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        sa1 = sa1_high;
        if (device == device_auto) { device = device_LIS3MDL; }
      }
      // check SA1 low address unless SA1 was specified to be high
      else if (sa1 != sa1_high && testReg(LIS3MDL_SA1_LOW_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        sa1 = sa1_low;
        if (device == device_auto) { device = device_LIS3MDL; }
      }
    }

    // make sure device and SA1 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa1 == sa1_auto)
    {
      return false;
    }
  }

  _device = device;

  switch (device)
  {
    case device_LIS3MDL:
      address = (sa1 == sa1_high) ? LIS3MDL_SA1_HIGH_ADDRESS : LIS3MDL_SA1_LOW_ADDRESS;
      break;
  }

  return true;
}

/*
Enables the LIS3MDL's magnetometer. Also:
- Selects ultra-high-performance mode for all axes
- Sets ODR (output data rate) to default power-on value of 10 Hz
- Sets magnetometer full scale (gain) to default power-on value of +/- 4 gauss
- Enables continuous conversion mode
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LIS3MDL::enableDefault(void)
{
  if (_device == device_LIS3MDL)
  {
    // 0xF0 = 0b11110000
    // OM = 11 (ultra-high-performance mode for X and Y); DO = 111 (80 Hz ODR)
    // TEMP_EN = 1
    writeReg(CTRL_REG1, 0xFC);

    // 0x00 = 0b00000000
    // FS = 00 (+/- 4 gauss full scale)
    writeReg(CTRL_REG2, 0x00);

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    writeReg(CTRL_REG3, 0x00);

    // 0x0C = 0b00001100
    // OMZ = 11 (low power mode for Z)
    writeReg(CTRL_REG4, 0x00);
  }
}

// Writes to a magnetometer register
void LIS3MDL::writeReg(uint8_t reg, uint8_t value)
{
  int length = 2;
  char data[2] = {reg, value};

  last_status = master->write(address, data, length);
}

// Reads from a magnetometer register
uint8_t LIS3MDL::readReg(uint8_t reg)
{
  uint8_t value;
  char c;
  char reg_c = (char) reg;

  last_status = master->write(address, &reg_c, 1);
  PRINTF("Write success: %d\n", last_status);
  last_status = master->read(address, &c, 1);
  PRINTF("Read success: %d\n", last_status);
  value = (uint8_t) c; 

  return value;
}

// Reads the 3 mag channels and stores them in vector m
void LIS3MDL::read()
{
  char buf[6];
  //assert MSB to enable subaddress updating
  char data = (char)(OUT_X_L | 0x80);

  master->write(address, &data, 1);

  master->read(address, buf, 6);

  // combine high and low bytes
  m.x = (int16_t)(buf[1] << 8 | buf[0]);
  m.y = (int16_t)(buf[3] << 8 | buf[2]);
  m.z = (int16_t)(buf[5] << 8 | buf[4]);
}

//Takes in the raw LIS3MDL data, edits with calibration settings, and
//converts to a bearing in degrees, returning bearing to user
double LIS3MDL::data_handler(void)
{
  double bearing;
  double x_corrected = (m.x - correction.x) * scale.x;
  double y_corrected = (m.y - correction.y) * scale.y;
  PRINTF("Correction x: %d y: %d\n", correction.x, correction.y);
  PRINTF("Scale x: %.2f y: %.2f\n", scale.x, scale.y);
  if(y_corrected > 0)
    bearing = 270.0 + atan(x_corrected / y_corrected) * 180 / M_PI;
  else if(y_corrected < 0)
    bearing = 90.0 + atan(x_corrected / y_corrected) * 180 / M_PI;
  else if(x_corrected < 0)
    bearing = 180.0;
  else
    bearing = 0.0;
  //pc.printf("Compass heading: %.3f degrees\n", bearing);
  return bearing;
}

void LIS3MDL::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

//Tests if can successfully read and write to a register
int16_t LIS3MDL::testReg(uint8_t address, regAddr reg)
{
  char data = (char) reg;
  char c; 
  PRINTF("CHAR: %d\n", data);
  //If write fails
  int result = master->write(address, &data, 1, false);
    if(result)
    {
      PRINTF("WRITE ERROR: %3d\n", result);
      return TEST_REG_ERROR;
    }
    //If read fails
    if(master->read(address, &c, 1, false))
    {
      PRINTF("READ ERROR\n");
      return TEST_REG_ERROR;
    }
    else
    {
      PRINTF("RETURNED CHAR: %d\n", c);
      return c;
    }
}