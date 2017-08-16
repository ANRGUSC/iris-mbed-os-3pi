#include "LSM6.h"

// Defines ////////////////////////////////////////////////////////////////

// The mbed two-wire interface uses an 8-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define DS33_SA0_HIGH_ADDRESS 0b11010110
#define DS33_SA0_LOW_ADDRESS  0b11010100

#define TEST_REG_ERROR -1

#define DS33_WHO_ID    0x69

//default I2C pins and frequency
#define DEFAULT_SDA		p9
#define DEFAULT_SCL		p10
#define DEFAULT_FREQUENCY	10000

//Factors to convert raw data to degrees per second or g's
#define XL_FACTOR 0.000061
#define G_FACTOR 0.00875

//Any gyro data -NOISE_MARGIN < x < NOISE_MARGIN is considered noise and is
//treated as 0 for data_handler total rotation calculations
#define NOISE_MARGIN 0.2

#define DEBUG   0

#if (DEBUG) 
#define PRINTF(...) pc.printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* (DEBUG) & DEBUG_PRINT */

extern Serial pc;

//Old gyro data needed for data handler
double old_gyro[3] = {0.0, 0.0, 0.0};

// Constructors ////////////////////////////////////////////////////////////////

LSM6::LSM6(void)
{
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
  total_rotation = 0.0;

  //Initialize I2C Master interface with default pins
  master = new I2C(DEFAULT_SDA, DEFAULT_SCL);
  //Set Frequency to default (10 kHz)
  master->frequency(DEFAULT_FREQUENCY);
}

LSM6::LSM6(PinName sda, PinName scl)
{
	_device = device_auto;

	io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
  total_rotation = 0.0;

  //Initialize I2C Master interface with user defined pins
	master = new I2C(sda, scl);
	//Set Frequency to default (1.66kHz)
  master->frequency(DEFAULT_FREQUENCY);
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in readAcc(), readGyro(), or read() since the last call to timeoutOccurred()?
bool LSM6::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void LSM6::setTimeout(uint16_t timeout)
{
  io_timeout = timeout;
}

uint16_t LSM6::getTimeout()
{
  return io_timeout;
}

bool LSM6::init(deviceType device, sa0State sa0)
{
  PRINTF("deviceType: %u state: %u\n", device, sa0);
  // perform auto-detection unless device type and SA0 state were both specified
  if (device == device_auto || sa0 == sa0_auto)
  {
    // check for LSM6DS33 if device is unidentified or was specified to be this type
    if (device == device_auto || device == device_DS33)
    {
      // check SA0 high address unless SA0 was specified to be low
      if (sa0 != sa0_low && testReg(DS33_SA0_HIGH_ADDRESS, WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_high;
        if (device == device_auto) { device = device_DS33; }
      }
      // check SA0 low address unless SA0 was specified to be high
      else if (sa0 != sa0_high && testReg(DS33_SA0_LOW_ADDRESS, WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_low;
        if (device == device_auto) { device = device_DS33; }
      }
    }

    // make sure device and SA0 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa0 == sa0_auto)
    {
      return false;
    }
  }

  _device = device;

  switch (device)
  {
    case device_DS33:
      address = (sa0 == sa0_high) ? DS33_SA0_HIGH_ADDRESS : DS33_SA0_LOW_ADDRESS;
      break;
  }

  return true;
}

/*
Enables the LSM6's accelerometer and gyro. Also:
- Sets sensor full scales (gain) to default power-on values, which are
  +/- 2 g for accelerometer and 245 dps for gyro
- Selects 1.66 kHz (high performance) ODR (output data rate) for accelerometer
  and 1.66 kHz (high performance) ODR for gyro. (These are the ODR settings for
  which the electrical characteristics are specified in the datasheet.)
- Enables automatic increment of register address during multiple byte access
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LSM6::enableDefault(void)
{
  
  if (_device == device_DS33)
  {
    // Accelerometer

    // 0x80 = 0b10000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
    writeReg(CTRL1_XL, 0x80);
    PRINTF("Success: %d\n", last_status);

    // Gyro

    // 0x80 = 0b10000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
    writeReg(CTRL2_G, 0x80);
    PRINTF("Success: %d\n", last_status);

    // Common

    // 0x04 = 0b00000100
    // IF_INC = 1 (automatically increment register address)
    writeReg(CTRL3_C, 0x04);
    PRINTF("Success: %d\n", last_status);
  }
}

//Enables the axes for the accelerometer or gyroscope
//Takes in the sensor type to enable, and three boolean values describing
//which axes to enable
void LSM6::enableAxes(sensorType sensor, bool x, bool y, bool z)
{	
	uint8_t value = 0x00;
  uint8_t num = 1;
	uint8_t reg[2] = {0,0};
	if(x)
		value |= 0x08;
	if(y)
		value |= 0x10;
	if(z)
		value |= 0x20;

  if(sensor == all_sensors)
  {
    reg[0] = CTRL9_XL;
    reg[1] = CTRL10_C;
    num = 2;
  }
	else if(sensor == accelerometer)
		reg[0] = CTRL9_XL;
	else
		reg[0] = CTRL10_C;

	for(int i=0; i<num; i++)
    writeReg(reg[i], value);
  if(!last_status)
    PRINTF("Axes successfully enabled\n");
}

//Writes to a LSM6 register
void LSM6::writeReg(uint8_t reg, uint8_t value)
{
  int length = 2;
  char data[2] = {reg, value};

  last_status = master->write(address, data, length);
}

//Reads from a LSM6 register
uint8_t LSM6::readReg(uint8_t reg)
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

// Reads the 3 accelerometer channels and stores them in vector a
void LSM6::readAcc(void)
{
	char buf[6];
  char data = (char)OUTX_L_XL;
    	
  // automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  master->write(address, &data, 1);

  master->read(address, buf, 6);

  // combine high and low bytes
  a.x = (int16_t)(buf[1] << 8 | buf[0]);
  a.y = (int16_t)(buf[3] << 8 | buf[2]);
  a.z = (int16_t)(buf[5] << 8 | buf[4]);
}

// Reads the 3 gyro channels and stores them in vector g
void LSM6::readGyro(void)
{
	char buf[6];
  char data = (char)OUTX_L_G;

  	// automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  	master->write(address, &data, 1);

  	master->read(address, buf, 6);

  	// combine high and low bytes
  	g.x = (int16_t)(buf[1] << 8 | buf[0]);
  	g.y = (int16_t)(buf[3] << 8 | buf[2]);
  	g.z = (int16_t)(buf[5] << 8 | buf[4]);
}

// Reads all 6 channels of the LSM6 and stores them in the object variables
void LSM6::read(void)
{
  readAcc();
  readGyro();
}

//Takes in the raw LSM6 data, adjusts to units of dps and g, and prints to
//the console
//Also keeps track of total rotation using current and old gyro values
void LSM6::data_handler(void)
{
  double accel[3] = {a.x * XL_FACTOR, a.y * XL_FACTOR, a.z * XL_FACTOR};
  double gyro[3] = {g.x * G_FACTOR, g.y * G_FACTOR, g.z * G_FACTOR};
  double current_rotation;
  int curr_time = timer.read_ms();
  PRINTF("current time: %d", curr_time);

  timer.reset();
  
  PRINTF("Accelerometer x:%.3f y:%.3f z:%.3f\n", accel[0], accel[1],
      accel[2] );
  PRINTF("Gyroscope x:%.1f y:%.1f z:%.1f\n", gyro[0], gyro[1],
      gyro[2]);

  //Use rectangular integration approximation to estimate position
  //Check current rotation to minimize drift
  current_rotation = (gyro[2] + old_gyro[2]) / 2.0 * curr_time / 1000;
  if(current_rotation > NOISE_MARGIN || current_rotation < -1.0 * NOISE_MARGIN)
    total_rotation += current_rotation;
  PRINTF("Current rotation: %.1f degrees\n", current_rotation);
  PRINTF("Position: %.1f degrees\n", total_rotation);

  old_gyro[0] = gyro[0];
  old_gyro[1] = gyro[1];
  old_gyro[2] = gyro[2];
}

void LSM6::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

//Check to see if can read and write to a register
int16_t LSM6::testReg(uint8_t address, regAddr reg)
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