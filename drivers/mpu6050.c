/*
 * File      : mpu6050.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-03     weety    first version.
 */
 
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "mpu6050.h"

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND)
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define DIR_READ			0x80
#define DIR_WRITE			0x00

// MPU 6050 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_FIFO_EN			0x23
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74
#define MPUREG_PRODUCT_ID		0x0C
#define MPUREG_TRIM1			0x0D
#define MPUREG_TRIM2			0x0E
#define MPUREG_TRIM3			0x0F
#define MPUREG_TRIM4			0x10

// Register bits defined for MPU 6050
#define BIT_SLEEP			0x40
#define BIT_H_RESET			0x80
#define BITS_CLKSEL			0x07
#define MPU_CLK_SEL_PLLGYROX		0x01
#define MPU_CLK_SEL_PLLGYROZ		0x03
#define MPU_EXT_SYNC_GYROX		0x02
#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2	0x00
#define BITS_DLPF_CFG_188HZ		0x01
#define BITS_DLPF_CFG_98HZ		0x02
#define BITS_DLPF_CFG_42HZ		0x03
#define BITS_DLPF_CFG_20HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF	0x07
#define BITS_DLPF_CFG_MASK		0x07
#define BIT_INT_ANYRD_2CLEAR		0x10
#define BIT_RAW_RDY_EN			0x01
#define BIT_I2C_IF_DIS			0x10
#define BIT_INT_STATUS_DATA		0x01

#define MPU_WHOAMI_6000			0x68
#define ICM_WHOAMI_20608		0xaf

// ICM2608 specific registers

/* this is an undocumented register which
   if set incorrectly results in getting a 2.7m/s/s offset
   on the Y axis of the accelerometer
*/
#define MPUREG_ICM_UNDOC1		0x11
#define MPUREG_ICM_UNDOC1_VALUE		0xc9

// Product ID Description for ICM2608
// There is none

#define ICM20608_REV_00		0

// Product ID Description for MPU6000
// high 4 bits 	low 4 bits
// Product Name	Product Revision
#define MPU6000ES_REV_C4		0x14
#define MPU6000ES_REV_C5		0x15
#define MPU6000ES_REV_D6		0x16
#define MPU6000ES_REV_D7		0x17
#define MPU6000ES_REV_D8		0x18
#define MPU6000_REV_C4			0x54
#define MPU6000_REV_C5			0x55
#define MPU6000_REV_D6			0x56
#define MPU6000_REV_D7			0x57
#define MPU6000_REV_D8			0x58
#define MPU6000_REV_D9			0x59
#define MPU6000_REV_D10			0x5A

#define MPU6000_ACCEL_DEFAULT_RANGE_G			8
#define MPU6000_ACCEL_DEFAULT_RATE			1000
#define MPU6000_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define MPU6000_GYRO_DEFAULT_RANGE_G			8
#define MPU6000_GYRO_DEFAULT_RATE			1000
#define MPU6000_GYRO_DEFAULT_DRIVER_FILTER_FREQ		30

//#define MPU6000_DEFAULT_ONCHIP_FILTER_FREQ		42
#define MPU6000_DEFAULT_ONCHIP_FILTER_FREQ		256

#define MPU6000_ONE_G					9.80665f

#define M_PI_F							3.1415926f

typedef struct {
	struct rt_device device;
	struct rt_i2c_bus_device *i2c_bus;
	rt_uint16_t addr;
	unsigned sample_rate;
	unsigned dlpf_freq;
	uint8_t product_id;
	float gyro_range_scale;
	float gyro_range_rad_s;
	float accel_range_scale;
	float accel_range_m_s2;
} mpu_device_t;

static mpu_device_t mpu_device = {0};

#define BE_SWAP16(v) ((((uint16_t)(v) & 0xFF00u) >> 8) | (((uint16_t)(v) & 0x00FFu) << 8))

static rt_err_t write_reg(struct rt_i2c_bus_device *i2c, rt_uint16_t addr, rt_uint8_t reg , rt_uint8_t val)
{
	struct rt_i2c_msg msgs[2];

	msgs[0].addr  = addr;
	msgs[0].flags = RT_I2C_WR;
	msgs[0].buf   = &reg;
	msgs[0].len   = 1;

	msgs[1].addr  = addr;
	msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;
	msgs[1].buf   = (rt_uint8_t *)&val;
	msgs[1].len   = 1;

	if (rt_i2c_transfer(i2c, msgs, 2) != 2)
		return -RT_ERROR;

	return RT_EOK;
}

static rt_err_t read_reg(struct rt_i2c_bus_device *i2c, rt_uint16_t addr, rt_uint8_t reg , rt_uint8_t* val)
{
	struct rt_i2c_msg msgs[2];

	msgs[0].addr  = addr;
	msgs[0].flags = RT_I2C_WR;
	msgs[0].buf   = &reg;
	msgs[0].len   = 1;

	msgs[1].addr  = addr;
	msgs[1].flags = RT_I2C_RD; /* Read from slave */
	msgs[1].buf   = (rt_uint8_t *)val;
	msgs[1].len   = 1;

	if (rt_i2c_transfer(i2c, msgs, 2) != 2)
		return -RT_ERROR;

	return RT_EOK;
}

static rt_err_t read_multi_reg(struct rt_i2c_bus_device *i2c, rt_uint16_t addr, rt_uint8_t reg , rt_uint8_t* val, uint8_t len)
{
	struct rt_i2c_msg msgs[2];

	msgs[0].addr  = addr;
	msgs[0].flags = RT_I2C_WR;
	msgs[0].buf   = &reg;
	msgs[0].len   = 1;

	msgs[1].addr  = addr;
	msgs[1].flags = RT_I2C_RD; /* Read from slave */
	msgs[1].buf   = (rt_uint8_t *)val;
	msgs[1].len   = len;

	if (rt_i2c_transfer(i2c, msgs, 2) != 2)
			return -RT_ERROR;

	return RT_EOK;
}

static rt_err_t write_checked_reg(struct rt_i2c_bus_device *i2c, rt_uint16_t addr, rt_uint8_t reg , rt_uint8_t val)
{
	rt_uint8_t r_buff;
	rt_err_t res = RT_EOK;
	
	res |= write_reg(i2c, addr, reg , val);
	res |= read_reg(i2c, addr, reg , &r_buff);
	
	if(r_buff != val || res != RT_EOK)
	{
		return -RT_ERROR;
	}
	
	return RT_EOK;
}

rt_err_t _set_sample_rate(mpu_device_t *mpu, unsigned desired_sample_rate_hz)
{
	rt_err_t res;
	
	if (desired_sample_rate_hz == 0) {
		desired_sample_rate_hz = MPU6000_GYRO_DEFAULT_RATE;
	}

	uint8_t div = 1000 / desired_sample_rate_hz;

	if (div > 200) { div = 200; }

	if (div < 1) { div = 1; }

	res = write_checked_reg(mpu->i2c_bus, mpu->addr, MPUREG_SMPLRT_DIV, div - 1);
	mpu->sample_rate = 1000 / div;
	
	return res;
}

rt_err_t _set_dlpf_filter(mpu_device_t *mpu, uint16_t frequency_hz)
{
	uint8_t filter;

	/*
	   choose next highest filter frequency available
	 */
	if (frequency_hz == 0) {
		mpu->dlpf_freq = 0;
		filter = BITS_DLPF_CFG_2100HZ_NOLPF;

	} else if (frequency_hz <= 5) {
		mpu->dlpf_freq = 5;
		filter = BITS_DLPF_CFG_5HZ;

	} else if (frequency_hz <= 10) {
		mpu->dlpf_freq = 10;
		filter = BITS_DLPF_CFG_10HZ;

	} else if (frequency_hz <= 20) {
		mpu->dlpf_freq = 20;
		filter = BITS_DLPF_CFG_20HZ;

	} else if (frequency_hz <= 42) {
		mpu->dlpf_freq = 42;
		filter = BITS_DLPF_CFG_42HZ;

	} else if (frequency_hz <= 98) {
		mpu->dlpf_freq = 98;
		filter = BITS_DLPF_CFG_98HZ;

	} else if (frequency_hz <= 188) {
		mpu->dlpf_freq = 188;
		filter = BITS_DLPF_CFG_188HZ;

	} else if (frequency_hz <= 256) {
		mpu->dlpf_freq = 256;
		filter = BITS_DLPF_CFG_256HZ_NOLPF2;

	} else {
		mpu->dlpf_freq = 0;
		filter = BITS_DLPF_CFG_2100HZ_NOLPF;
	}

	return write_checked_reg(mpu->i2c_bus, mpu->addr, MPUREG_CONFIG, filter);
}

rt_err_t set_accel_range(mpu_device_t *mpu, unsigned max_g_in)
{
	// workaround for bugged versions of MPU6k (rev C)
	switch (mpu->product_id) {
	case MPU6000ES_REV_C4:
	case MPU6000ES_REV_C5:
	case MPU6000_REV_C4:
	case MPU6000_REV_C5:
		write_checked_reg(mpu->i2c_bus, mpu->addr, MPUREG_ACCEL_CONFIG, 1 << 3);
		mpu->accel_range_scale = (MPU6000_ONE_G / 4096.0f);
		mpu->accel_range_m_s2 = 8.0f * MPU6000_ONE_G;
		return RT_EOK;
	}

	uint8_t afs_sel;
	float lsb_per_g;
	float max_accel_g;

	if (max_g_in > 8) { // 16g - AFS_SEL = 3
		afs_sel = 3;
		lsb_per_g = 2048;
		max_accel_g = 16;

	} else if (max_g_in > 4) { //  8g - AFS_SEL = 2
		afs_sel = 2;
		lsb_per_g = 4096;
		max_accel_g = 8;

	} else if (max_g_in > 2) { //  4g - AFS_SEL = 1
		afs_sel = 1;
		lsb_per_g = 8192;
		max_accel_g = 4;

	} else {                //  2g - AFS_SEL = 0
		afs_sel = 0;
		lsb_per_g = 16384;
		max_accel_g = 2;
	}

	write_checked_reg(mpu->i2c_bus, mpu->addr, MPUREG_ACCEL_CONFIG, afs_sel << 3);
	mpu->accel_range_scale = (MPU6000_ONE_G / lsb_per_g);
	mpu->accel_range_m_s2 = max_accel_g * MPU6000_ONE_G;

	return RT_EOK;
}

rt_err_t mpu6050_init(rt_device_t dev)
{	
	rt_err_t res = RT_EOK;
	mpu_device_t *mpu = dev->user_data;

	uint8_t tries = 5;
	uint8_t reg_val;
	rt_err_t r_res;
	rt_kprintf("mpu6050 init\n");
	
	/* read product id */
	read_reg(mpu->i2c_bus, mpu->addr, MPUREG_PRODUCT_ID, &mpu->product_id);

	while (--tries != 0) {
		write_reg(mpu->i2c_bus, mpu->addr, MPUREG_PWR_MGMT_1, BIT_H_RESET);
		time_delay_us(10000);

		// Wake up device and select GyroZ clock. Note that the
		// MPU6000 starts up in sleep mode, and it can take some time
		// for it to come out of sleep
		write_checked_reg(mpu->i2c_bus, mpu->addr, MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
		time_delay_us(1000);

		// Disable I2C bus (recommended on datasheet)
		//write_checked_reg(mpu->i2c_bus, mpu->addr, MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
		
		r_res = read_reg(mpu->i2c_bus, mpu->addr, MPUREG_PWR_MGMT_1, &reg_val);
		if ( r_res == RT_EOK && reg_val == MPU_CLK_SEL_PLLGYROZ) {
			break;
		}

		time_delay_us(2000);
	}
	
	r_res = read_reg(mpu->i2c_bus, mpu->addr, MPUREG_PWR_MGMT_1, &reg_val);
	if (reg_val != MPU_CLK_SEL_PLLGYROZ) {
		return RT_ERROR;
	}
	
	time_delay_us(1000);

	// SAMPLE RATE
	if(_set_sample_rate(mpu, MPU6000_ACCEL_DEFAULT_RATE) != RT_EOK){
		rt_kprintf("err, mpu6050, set sample rate fail\n");
		return RT_ERROR;
	}
	time_delay_us(1000);
	
	// FS & DLPF   FS=2000 deg/s, DLPF = 20Hz (low pass filter)
	// was 90 Hz, but this ruins quality and does not improve the
	// system response
	if(_set_dlpf_filter(mpu, MPU6000_DEFAULT_ONCHIP_FILTER_FREQ) != RT_EOK){
		rt_kprintf("err, mpu6050, set lpf fail\n");
		return RT_ERROR;
	}
	time_delay_us(1000);
	
	// Gyro scale 2000 deg/s ()
	if(write_checked_reg(mpu->i2c_bus, mpu->addr, MPUREG_GYRO_CONFIG, BITS_FS_2000DPS) != RT_EOK ){
		rt_kprintf("err, mpu6050, set gyr scale fail\n");
		return RT_ERROR;
	}
	time_delay_us(1000);
	
	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI
	mpu->gyro_range_scale = (0.0174532 / 16.4);//1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);
	mpu->gyro_range_rad_s = (2000.0f / 180.0f) * M_PI_F;
	
	set_accel_range(mpu, 8);

	time_delay_us(1000);

	return res;
}

rt_err_t mpu6050_gyr_read_raw(mpu_device_t *mpu, int16_t gyr[3])
{
	rt_err_t res;
	uint16_t raw[3];
	res = read_multi_reg(mpu->i2c_bus, mpu->addr, MPUREG_GYRO_XOUT_H, (uint8_t*)raw, 6);
	// big-endian to little-endian
	gyr[0] = BE_SWAP16(raw[0]);
	gyr[1] = BE_SWAP16(raw[1]);
	gyr[2] = BE_SWAP16(raw[2]);

	return res;
}

rt_err_t mpu6050_gyr_read_rad(mpu_device_t *mpu, float gyr[3])
{
	int16_t gyr_raw[3];
	rt_err_t res;
	res = mpu6050_gyr_read_raw(mpu, gyr_raw);
	
	gyr[0] = mpu->gyro_range_scale*gyr_raw[0];
	gyr[1] = mpu->gyro_range_scale*gyr_raw[1];
	gyr[2] = mpu->gyro_range_scale*gyr_raw[2];
	
	return res;
}

rt_err_t mpu6050_acc_read_raw(mpu_device_t *mpu, int16_t acc[3])
{
	int16_t raw[3];
	rt_err_t res;
	res = read_multi_reg(mpu->i2c_bus, mpu->addr, MPUREG_ACCEL_XOUT_H, (rt_uint8_t*)raw, 6);
	// big-endian to little-endian
	acc[0] = BE_SWAP16(raw[0]);
	acc[1] = BE_SWAP16(raw[1]);
	acc[2] = BE_SWAP16(raw[2]);

	return res;
}

rt_err_t mpu6050_acc_read_scale(mpu_device_t *mpu, float acc[3])
{
	int16_t acc_raw[3];
	rt_err_t res;
	res = mpu6050_acc_read_raw(mpu, acc_raw);
	
	acc[0] = mpu->accel_range_scale*acc_raw[0];
	acc[1] = mpu->accel_range_scale*acc_raw[1];
	acc[2] = mpu->accel_range_scale*acc_raw[2];
	
	return res;
}

rt_size_t mpu6050_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
	rt_err_t res;
	mpu_device_t *mpu = (mpu_device_t*)dev->user_data;
	if(pos == GYR_RAW_POS)	/* read raw gyr data */
	{
		res = mpu6050_gyr_read_raw(mpu, ((int16_t*)buffer));
		if(res != RT_EOK)
		{
			return 0;
		}
	}else if(pos == GYR_SCALE_POS)	/* read rad gyr data */
	{
		res = mpu6050_gyr_read_rad(mpu, ((float*)buffer));
		if(res != RT_EOK)
		{
			return 0;
		}
	}else if(pos == ACC_RAW_POS)	/* read raw acc data */
	{
		res = mpu6050_acc_read_raw(mpu, ((int16_t*)buffer));
		if(res != RT_EOK)
		{
			return 0;
		}
	}else if(pos == ACC_SCALE_POS)	/* read scaled acc data */
	{
		res = mpu6050_acc_read_scale(mpu, ((float*)buffer));
		if(res != RT_EOK)
		{
			return 0;
		}
	}else{
		/* unknow pos */
		return 0;
	}
	
	return size;
}

rt_err_t mpu6050_control(rt_device_t dev, int cmd, void *args)
{
	rt_err_t res = RT_EOK;
	
	switch(cmd)
	{
		default:
			return RT_ERROR;
	}
	
	return res;
}

rt_err_t rt_mpu6050_init(char* i2c_bus_name)
{	
	rt_err_t res = RT_EOK;;
	
	/* set device type */
	mpu_device.device.type    = RT_Device_Class_Char;
	mpu_device.device.init    = mpu6050_init;
	mpu_device.device.open    = RT_NULL;
	mpu_device.device.close   = RT_NULL;
	mpu_device.device.read    = mpu6050_read;
	mpu_device.device.write   = RT_NULL;
	mpu_device.device.control = mpu6050_control;
	mpu_device.device.user_data = &mpu_device;
	mpu_device.addr = MPU6050_DEFAULT_ADDRESS;

	/* register to device manager */
	res |= rt_device_register(&(mpu_device.device), "mpu6050", RT_DEVICE_FLAG_RDWR);

	mpu_device.i2c_bus = rt_i2c_bus_device_find(i2c_bus_name);
	if(mpu_device.i2c_bus == RT_NULL)
	{
		rt_kprintf("i2c bus %s not found!\r\n", i2c_bus_name);
		return RT_EEMPTY;
	}

	return res;
}

int cmd_mpu6050_init(int argc, char *argv[])
{
	rt_mpu6050_init("i2c1");
	rt_device_open(&(mpu_device.device), RT_DEVICE_OFLAG_RDWR);

	return 0;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

MSH_CMD_EXPORT_ALIAS(cmd_mpu6050_init, mpu6050_init, mpu6050 init);

#endif

