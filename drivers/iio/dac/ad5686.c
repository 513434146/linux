/*
 * AD5686R, AD5685R, AD5684R Digital to analog converters  driver
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define AD5686_DAC_CHANNELS			4

#define AD5686_ADDR(x)				((x) << 16)
#define AD5686_CMD(x)				((x) << 20)

#define AD5686_ADDR_DAC(chan)			(0x1 << (chan))
#define AD5686_ADDR_ALL_DAC			0xF

#define AD5686_CMD_NOOP				0x0
#define AD5686_CMD_WRITE_INPUT_N		0x1
#define AD5686_CMD_UPDATE_DAC_N			0x2
#define AD5686_CMD_WRITE_INPUT_N_UPDATE_N	0x3
#define AD5686_CMD_POWERDOWN_DAC		0x4
#define AD5686_CMD_LDAC_MASK			0x5
#define AD5686_CMD_RESET			0x6
#define AD5686_CMD_INTERNAL_REFER_SETUP		0x7
#define AD5686_CMD_DAISY_CHAIN_ENABLE		0x8
#define AD5686_CMD_READBACK_ENABLE		0x9

#define AD5686_LDAC_PWRDN_NONE			0x0
#define AD5686_LDAC_PWRDN_1K			0x1
#define AD5686_LDAC_PWRDN_100K			0x2
#define AD5686_LDAC_PWRDN_3STATE		0x3

/**
 * struct ad5686_chip_info - chip specific information
 * @int_vref_mv:	AD5620/40/60: the internal reference voltage
 * @channel:		channel specification
*/

struct ad5686_chip_info {
	u16				int_vref_mv;
	struct iio_chan_spec		channel[AD5686_DAC_CHANNELS];
};

struct ad5686_state;

typedef int (*ad5686_write_func)(struct ad5686_state *st,
				 u8 cmd, u8 addr, u16 val, u8 shift);

typedef int (*ad5686_read_func)(struct ad5686_state *st, u8 addr);

/**
 * struct ad5446_state - driver instance specific data
 * @spi:		spi_device
 * @chip_info:		chip model specific constants, available modes etc
 * @reg:		supply regulator
 * @vref_mv:		actual reference voltage used
 * @pwr_down_mask:	power down mask
 * @pwr_down_mode:	current power down mode
 * @data:		spi transfer buffers
 */

struct ad5686_state {
	struct device 			*dev;
	const struct ad5686_chip_info	*chip_info;
	struct regulator		*reg;
	unsigned short			vref_mv;
	unsigned			pwr_down_mask;
	unsigned			pwr_down_mode;
	ad5686_write_func		write;
	ad5686_read_func		read;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */

	union {
		__be32 d32;
		u8 d8[4];
	} data[3] ____cacheline_aligned;
};

/**
 * ad5686_supported_device_ids:
 */

enum ad5686_supported_device_ids {
	ID_AD5671R,
	ID_AD5672R,
	ID_AD5675R,
	ID_AD5676,
	ID_AD5676R,
	ID_AD5684,
	ID_AD5684R,
	ID_AD5685R,
	ID_AD5686,
	ID_AD5686R,
	ID_AD5694,
	ID_AD5694R,
	ID_AD5695R,
	ID_AD5696,
	ID_AD5696R,
};

static const char * const ad5686_powerdown_modes[] = {
	"1kohm_to_gnd",
	"100kohm_to_gnd",
	"three_state"
};

static int ad5686_get_powerdown_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	struct ad5686_state *st = iio_priv(indio_dev);

	return ((st->pwr_down_mode >> (chan->channel * 2)) & 0x3) - 1;
}

static int ad5686_set_powerdown_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan, unsigned int mode)
{
	struct ad5686_state *st = iio_priv(indio_dev);

	st->pwr_down_mode &= ~(0x3 << (chan->channel * 2));
	st->pwr_down_mode |= ((mode + 1) << (chan->channel * 2));

	return 0;
}

static const struct iio_enum ad5686_powerdown_mode_enum = {
	.items = ad5686_powerdown_modes,
	.num_items = ARRAY_SIZE(ad5686_powerdown_modes),
	.get = ad5686_get_powerdown_mode,
	.set = ad5686_set_powerdown_mode,
};

static ssize_t ad5686_read_dac_powerdown(struct iio_dev *indio_dev,
		uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct ad5686_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", !!(st->pwr_down_mask &
				       (0x3 << (chan->channel * 2))));
}

static ssize_t ad5686_write_dac_powerdown(struct iio_dev *indio_dev,
		uintptr_t private, const struct iio_chan_spec *chan, const char *buf,
		size_t len)
{
	bool readin;
	int ret;
	struct ad5686_state *st = iio_priv(indio_dev);

	ret = strtobool(buf, &readin);
	if (ret)
		return ret;

	if (readin)
		st->pwr_down_mask |= (0x3 << (chan->channel * 2));
	else
		st->pwr_down_mask &= ~(0x3 << (chan->channel * 2));

	ret = st->write(st, AD5686_CMD_POWERDOWN_DAC, 0,
			st->pwr_down_mask & st->pwr_down_mode, 0);

	return ret ? ret : len;
}

static int ad5686_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad5686_state *st = iio_priv(indio_dev);
	int ret;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		ret = st->read(st, chan->address);
		mutex_unlock(&indio_dev->mlock);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	}
	return -EINVAL;
}

static int ad5686_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad5686_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val > (1 << chan->scan_type.realbits) || val < 0)
			return -EINVAL;

		mutex_lock(&indio_dev->mlock);
		ret = st->write(st,
				AD5686_CMD_WRITE_INPUT_N_UPDATE_N,
				chan->address,
				val,
				chan->scan_type.shift);
		mutex_unlock(&indio_dev->mlock);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct iio_info ad5686_info = {
	.read_raw = ad5686_read_raw,
	.write_raw = ad5686_write_raw,
	.driver_module = THIS_MODULE,
};

static const struct iio_chan_spec_ext_info ad5686_ext_info[] = {
	{
		.name = "powerdown",
		.read = ad5686_read_dac_powerdown,
		.write = ad5686_write_dac_powerdown,
		.shared = IIO_SEPARATE,
	},
	IIO_ENUM("powerdown_mode", IIO_SEPARATE, &ad5686_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", &ad5686_powerdown_mode_enum),
	{ },
};

#define AD5868_CHANNEL(chan, bits, _shift) {			\
		.type = IIO_VOLTAGE,				\
		.indexed = 1,					\
		.output = 1,					\
		.channel = chan,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),\
		.address = AD5686_ADDR_DAC(chan),		\
		.scan_type = {					\
			.sign = 'u',				\
			.realbits = (bits),			\
			.storagebits = 16,			\
			.shift = (_shift),			\
		},						\
		.ext_info = ad5686_ext_info,			\
}

static const struct ad5686_chip_info ad5686_chip_info_tbl[] = {
	[ID_AD5671R] = {
		.channel[0] = AD5868_CHANNEL(0, 12, 4),
		.channel[1] = AD5868_CHANNEL(1, 12, 4),
		.channel[2] = AD5868_CHANNEL(2, 12, 4),
		.channel[3] = AD5868_CHANNEL(3, 12, 4),
		.int_vref_mv = 2500,
	},
	[ID_AD5672R] = {
		.channel[0] = AD5868_CHANNEL(0, 12, 4),
		.channel[1] = AD5868_CHANNEL(1, 12, 4),
		.channel[2] = AD5868_CHANNEL(2, 12, 4),
		.channel[3] = AD5868_CHANNEL(3, 12, 4),
		.int_vref_mv = 2500,
	},
	[ID_AD5675R] = {
		.channel[0] = AD5868_CHANNEL(0, 16, 0),
		.channel[1] = AD5868_CHANNEL(1, 16, 0),
		.channel[2] = AD5868_CHANNEL(2, 16, 0),
		.channel[3] = AD5868_CHANNEL(3, 16, 0),
		.int_vref_mv = 2500,
	},
	[ID_AD5676] = {
		.channel[0] = AD5868_CHANNEL(0, 16, 0),
		.channel[1] = AD5868_CHANNEL(1, 16, 0),
		.channel[2] = AD5868_CHANNEL(2, 16, 0),
		.channel[3] = AD5868_CHANNEL(3, 16, 0),
	},
	[ID_AD5676R] = {
		.channel[0] = AD5868_CHANNEL(0, 16, 0),
		.channel[1] = AD5868_CHANNEL(1, 16, 0),
		.channel[2] = AD5868_CHANNEL(2, 16, 0),
		.channel[3] = AD5868_CHANNEL(3, 16, 0),
		.int_vref_mv = 2500,
	},
	[ID_AD5684] = {
		.channel[0] = AD5868_CHANNEL(0, 12, 4),
		.channel[1] = AD5868_CHANNEL(1, 12, 4),
		.channel[2] = AD5868_CHANNEL(2, 12, 4),
		.channel[3] = AD5868_CHANNEL(3, 12, 4),
	},
	[ID_AD5684R] = {
		.channel[0] = AD5868_CHANNEL(0, 12, 4),
		.channel[1] = AD5868_CHANNEL(1, 12, 4),
		.channel[2] = AD5868_CHANNEL(2, 12, 4),
		.channel[3] = AD5868_CHANNEL(3, 12, 4),
		.int_vref_mv = 2500,
	},
	[ID_AD5685R] = {
		.channel[0] = AD5868_CHANNEL(0, 14, 2),
		.channel[1] = AD5868_CHANNEL(1, 14, 2),
		.channel[2] = AD5868_CHANNEL(2, 14, 2),
		.channel[3] = AD5868_CHANNEL(3, 14, 2),
		.int_vref_mv = 2500,
	},
	[ID_AD5686] = {
		.channel[0] = AD5868_CHANNEL(0, 16, 0),
		.channel[1] = AD5868_CHANNEL(1, 16, 0),
		.channel[2] = AD5868_CHANNEL(2, 16, 0),
		.channel[3] = AD5868_CHANNEL(3, 16, 0),
	},
	[ID_AD5686R] = {
		.channel[0] = AD5868_CHANNEL(0, 16, 0),
		.channel[1] = AD5868_CHANNEL(1, 16, 0),
		.channel[2] = AD5868_CHANNEL(2, 16, 0),
		.channel[3] = AD5868_CHANNEL(3, 16, 0),
		.int_vref_mv = 2500,
	},
	[ID_AD5694] = {
		.channel[0] = AD5868_CHANNEL(0, 12, 4),
		.channel[1] = AD5868_CHANNEL(1, 12, 4),
		.channel[2] = AD5868_CHANNEL(2, 12, 4),
		.channel[3] = AD5868_CHANNEL(3, 12, 4),
	},
	[ID_AD5694R] = {
		.channel[0] = AD5868_CHANNEL(0, 12, 4),
		.channel[1] = AD5868_CHANNEL(1, 12, 4),
		.channel[2] = AD5868_CHANNEL(2, 12, 4),
		.channel[3] = AD5868_CHANNEL(3, 12, 4),
		.int_vref_mv = 2500,
	},
	[ID_AD5695R] = {
		.channel[0] = AD5868_CHANNEL(0, 14, 2),
		.channel[1] = AD5868_CHANNEL(1, 14, 2),
		.channel[2] = AD5868_CHANNEL(2, 14, 2),
		.channel[3] = AD5868_CHANNEL(3, 14, 2),
		.int_vref_mv = 2500,
	},
	[ID_AD5696] = {
		.channel[0] = AD5868_CHANNEL(0, 16, 0),
		.channel[1] = AD5868_CHANNEL(1, 16, 0),
		.channel[2] = AD5868_CHANNEL(2, 16, 0),
		.channel[3] = AD5868_CHANNEL(3, 16, 0),
	},
	[ID_AD5696R] = {
		.channel[0] = AD5868_CHANNEL(0, 16, 0),
		.channel[1] = AD5868_CHANNEL(1, 16, 0),
		.channel[2] = AD5868_CHANNEL(2, 16, 0),
		.channel[3] = AD5868_CHANNEL(3, 16, 0),
		.int_vref_mv = 2500,
	},
};

static int ad5686_probe(struct device *dev, enum ad5686_supported_device_ids chip_type,
			const char *name, ad5686_write_func write, ad5686_read_func read)
{
	struct ad5686_state *st;
	struct iio_dev *indio_dev;
	int ret, voltage_uv = 0;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (indio_dev == NULL)
		return  -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	st->dev = dev;
	st->write = write;
	st->read = read;

	st->reg = devm_regulator_get_optional(dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;

		ret = regulator_get_voltage(st->reg);
		if (ret < 0)
			goto error_disable_reg;

		voltage_uv = ret;
	}

	st->chip_info = &ad5686_chip_info_tbl[chip_type];

	/* If available, internal reference is on by default */
	if (st->chip_info->int_vref_mv)
		st->vref_mv = st->chip_info->int_vref_mv;
	else if (voltage_uv)
		st->vref_mv = voltage_uv / 1000;
	else
		dev_warn(dev, "reference voltage unspecified\n");

	/* Set all the power down mode for all channels to 1K pulldown */
	st->pwr_down_mode = 0x55;

	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->info = &ad5686_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channel;
	indio_dev->num_channels = AD5686_DAC_CHANNELS;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);
	return ret;
}

static int ad5686_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad5686_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return 0;
}

#if IS_ENABLED(CONFIG_SPI)

static int ad5686_spi_write(struct ad5686_state *st,
			    u8 cmd, u8 addr, u16 val, u8 shift)
{
	struct spi_device *spi = to_spi_device(st->dev);
	val <<= shift;

	st->data[0].d32 = cpu_to_be32(AD5686_CMD(cmd) |
				      AD5686_ADDR(addr) |
				      val);

	return spi_write(spi, &st->data[0].d8[1], 3);
}

static int ad5686_spi_read(struct ad5686_state *st, u8 addr)
{
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0].d8[1],
			.len = 3,
			.cs_change = 1,
		}, {
			.tx_buf = &st->data[1].d8[1],
			.rx_buf = &st->data[2].d8[1],
			.len = 3,
		},
	};
	struct spi_device *spi = to_spi_device(st->dev);
	int ret;

	st->data[0].d32 = cpu_to_be32(AD5686_CMD(AD5686_CMD_READBACK_ENABLE) |
				      AD5686_ADDR(addr));
	st->data[1].d32 = cpu_to_be32(AD5686_CMD(AD5686_CMD_NOOP));

	ret = spi_sync_transfer(spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	return be32_to_cpu(st->data[2].d32);
}

static int ad5686_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);

	return ad5686_probe(&spi->dev, id->driver_data, id->name,
			    ad5686_spi_write, ad5686_spi_read);
}

static int ad5686_spi_remove(struct spi_device *spi)
{
	return ad5686_remove(&spi->dev);
}

static const struct spi_device_id ad5686_spi_id[] = {
	{"ad5672r", ID_AD5672R},
	{"ad5676", ID_AD5676},
	{"ad5676r", ID_AD5676R},
	{"ad5684", ID_AD5684},
	{"ad5684r", ID_AD5684R},
	{"ad5685r", ID_AD5685R},
	{"ad5686", ID_AD5686},
	{"ad5686r", ID_AD5686R},
	{}
};
MODULE_DEVICE_TABLE(spi, ad5686_spi_id);

static struct spi_driver ad5686_spi_driver = {
	.driver = {
		.name = "ad5686",
	},
	.probe = ad5686_spi_probe,
	.remove = ad5686_spi_remove,
	.id_table = ad5686_spi_id,
};

static int __init ad5686_spi_register_driver(void)
{
	return spi_register_driver(&ad5686_spi_driver);
}

static void ad5686_spi_unregister_driver(void)
{
	spi_unregister_driver(&ad5686_spi_driver);
}

#else

static inline int ad5686_spi_register_driver(void)
{
	return 0;
}
static inline void ad5686_spi_unregister_driver(void) { }

#endif

#if IS_ENABLED(CONFIG_I2C)

static int ad5686_i2c_read(struct ad5686_state *st, u8 addr)
{
	struct i2c_client *i2c = to_i2c_client(st->dev);
	char data[3];
	int ret;

	ret = i2c_master_recv(i2c, data, 2);
	if (ret < 0)
		return ret;

	return (((int)data[0] << 8) | data[1]);
}

static int ad5686_i2c_write(struct ad5686_state *st,
			    u8 cmd, u8 addr, u16 val, u8 shift)
{
	struct i2c_client *i2c = to_i2c_client(st->dev);
	u32 msg;
	int ret;

	val <<= shift;
	msg = cpu_to_be32((AD5686_CMD(cmd) | AD5686_ADDR(addr) | val) << 8);

	ret = i2c_master_send(i2c, (char *)&msg, 3);

	return (ret < 0 ? ret : (ret != 3) ? -EIO : 0);
}

static int ad5686_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	return ad5686_probe(&i2c->dev, id->driver_data, id->name,
			    ad5686_i2c_write, ad5686_i2c_read);
}

static int ad5686_i2c_remove(struct i2c_client *i2c)
{
	return ad5686_remove(&i2c->dev);
}

static const struct i2c_device_id ad5686_i2c_id[] = {
	{"ad5671r", ID_AD5671R},
	{"ad5675r", ID_AD5675R},
	{"ad5694", ID_AD5694},
	{"ad5694r", ID_AD5694R},
	{"ad5695r", ID_AD5695R},
	{"ad5696", ID_AD5696},
	{"ad5696r", ID_AD5696R},
	{}
};
MODULE_DEVICE_TABLE(i2c, i2c_device_id);

static struct i2c_driver ad5686_i2c_driver = {
	.driver = {
		.name = "ad5686",
	},
	.probe = ad5686_i2c_probe,
	.remove = ad5686_i2c_remove,
	.id_table = ad5686_i2c_id,
};

static int __init ad5686_i2c_register_driver(void)
{
	return i2c_add_driver(&ad5686_i2c_driver);
}

static void __exit ad5686_i2c_unregister_driver(void)
{
	i2c_del_driver(&ad5686_i2c_driver);
}

#else

static inline int ad5686_i2c_register_driver(void)
{
	return 0;
}
static inline void ad5686_i2c_unregister_driver(void) { }

#endif

static int __init ad5686_init(void)
{
	int ret;

	ret = ad5686_spi_register_driver();
	if (ret)
		return ret;

	ret = ad5686_i2c_register_driver();
	if (ret) {
		ad5686_spi_unregister_driver();
		return ret;
	}

	return 0;
}
module_init(ad5686_init);

static void __exit ad5686_exit(void)
{
	ad5686_i2c_unregister_driver();
	ad5686_spi_unregister_driver();
}
module_exit(ad5686_exit);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices AD5686/85/84 DAC");
MODULE_LICENSE("GPL v2");
