#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/string.h>

#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>

#define MMA8452_STATUS              0x00
#define MMA8452_OUT_X_MSB           0x01
#define MMA8452_WHO_AM_I            0x0D
#define MMA8452_INT_SOURCE          0x0C
#define MMA8452_XYZ_DATA_CFG        0x0E
#define MMA8452_CTRL_REG1           0x2A
#define MMA8452_CTRL_REG4           0x2D
#define MMA8452_CTRL_REG5           0x2E

#define MMA8452_WHO_AM_I_VAL        0x2A

#define MMA8452_CTRL_REG1_ACTIVE    BIT(0)
#define MMA8452_CTRL_REG1_DR_SHIFT  3
#define MMA8452_CTRL_REG1_DR_MASK   GENMASK(5, 3)

#define MMA8452_INT_DRDY            BIT(0)

static const int mma8452_scale_nano[] = {
	976562,
	1953125,
	3906250,
};

static const int mma8452_odr_millihz[] = {
	800000,
	400000,
	200000,
	100000,
	50000,
	12500,
	6250,
	1560,
};

struct mma8452_data {
	struct i2c_client *client;
	struct mutex lock;
	struct iio_trigger *trig;
	u8 fs_bits;
	u8 odr_bits;
};

struct mma8452_scan {
	s16 channels[3];
	s64 timestamp __aligned(8);
};

static s16 mma8452_convert_12bit(u8 msb, u8 lsb)
{
	u16 raw = ((u16)msb << 8) | lsb;

	return sign_extend32(raw >> 4, 11);
}

static int mma8452_set_active(struct mma8452_data *data, bool active)
{
	int ctrl;

	ctrl = i2c_smbus_read_byte_data(data->client, MMA8452_CTRL_REG1);
	if (ctrl < 0)
		return ctrl;

	if (active)
		ctrl |= MMA8452_CTRL_REG1_ACTIVE;
	else
		ctrl &= ~MMA8452_CTRL_REG1_ACTIVE;

	return i2c_smbus_write_byte_data(data->client, MMA8452_CTRL_REG1, ctrl);
}

static int mma8452_set_range(struct mma8452_data *data, u8 fs_bits)
{
	int ret;

	if (fs_bits > 2)
		return -EINVAL;

	ret = mma8452_set_active(data, false);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(data->client,
					MMA8452_XYZ_DATA_CFG,
					fs_bits & 0x03);
	if (ret < 0)
		goto reactivate;

	data->fs_bits = fs_bits;

reactivate:
	mma8452_set_active(data, true);
	return ret;
}

static int mma8452_set_odr(struct mma8452_data *data, u8 odr_bits)
{
	int ctrl;
	int ret;

	if (odr_bits > 7)
		return -EINVAL;

	ret = mma8452_set_active(data, false);
	if (ret < 0)
		return ret;

	ctrl = i2c_smbus_read_byte_data(data->client, MMA8452_CTRL_REG1);
	if (ctrl < 0) {
		ret = ctrl;
		goto reactivate;
	}

	ctrl &= ~MMA8452_CTRL_REG1_DR_MASK;
	ctrl |= (odr_bits << MMA8452_CTRL_REG1_DR_SHIFT) &
		MMA8452_CTRL_REG1_DR_MASK;

	ret = i2c_smbus_write_byte_data(data->client, MMA8452_CTRL_REG1, ctrl);
	if (ret < 0)
		goto reactivate;

	data->odr_bits = odr_bits;

reactivate:
	mma8452_set_active(data, true);
	return ret;
}

static int mma8452_read_axis(struct mma8452_data *data, int axis, int *val)
{
	int ret;
	u8 buf[2];
	u8 reg = MMA8452_OUT_X_MSB + axis * 2;

	ret = i2c_smbus_read_i2c_block_data(data->client, reg, sizeof(buf), buf);
	if (ret < 0)
		return ret;
	if (ret != sizeof(buf))
		return -EIO;

	*val = mma8452_convert_12bit(buf[0], buf[1]);

	return 0;
}

static int mma8452_read_xyz(struct mma8452_data *data, s16 *x, s16 *y, s16 *z)
{
	int ret;
	u8 buf[6];

	ret = i2c_smbus_read_i2c_block_data(data->client,
					    MMA8452_OUT_X_MSB,
					    sizeof(buf),
					    buf);
	if (ret < 0)
		return ret;
	if (ret != sizeof(buf))
		return -EIO;

	*x = mma8452_convert_12bit(buf[0], buf[1]);
	*y = mma8452_convert_12bit(buf[2], buf[3]);
	*z = mma8452_convert_12bit(buf[4], buf[5]);

	return 0;
}

static int mma8452_read_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long mask)
{
	struct mma8452_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->lock);
		ret = mma8452_read_axis(data, chan->address, val);
		mutex_unlock(&data->lock);

		if (ret < 0)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = mma8452_scale_nano[data->fs_bits];
		return IIO_VAL_INT_PLUS_NANO;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = mma8452_odr_millihz[data->odr_bits] / 1000;
		*val2 = (mma8452_odr_millihz[data->odr_bits] % 1000) * 1000;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}
}

static int mma8452_write_raw(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan,
			     int val, int val2, long mask)
{
	struct mma8452_data *data = iio_priv(indio_dev);
	int target_millihz;
	int i;
	int ret = -EINVAL;

	mutex_lock(&data->lock);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (val != 0)
			break;

		for (i = 0; i < ARRAY_SIZE(mma8452_scale_nano); i++) {
			if (val2 == mma8452_scale_nano[i]) {
				ret = mma8452_set_range(data, i);
				break;
			}
		}
		break;

	case IIO_CHAN_INFO_SAMP_FREQ:
		target_millihz = val * 1000 + val2 / 1000;

		for (i = 0; i < ARRAY_SIZE(mma8452_odr_millihz); i++) {
			if (target_millihz == mma8452_odr_millihz[i]) {
				ret = mma8452_set_odr(data, i);
				break;
			}
		}
		break;

	default:
		break;
	}

	mutex_unlock(&data->lock);

	return ret;
}

static int mma8452_read_avail(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	static const int scale_avail[] = {
		0, 976562,
		0, 1953125,
		0, 3906250,
	};

	static const int samp_freq_avail[] = {
		800, 0,
		400, 0,
		200, 0,
		100, 0,
		50, 0,
		12, 500000,
		6, 250000,
		1, 560000,
	};

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*vals = scale_avail;
		*type = IIO_VAL_INT_PLUS_NANO;
		*length = ARRAY_SIZE(scale_avail);
		return IIO_AVAIL_LIST;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = samp_freq_avail;
		*type = IIO_VAL_INT_PLUS_MICRO;
		*length = ARRAY_SIZE(samp_freq_avail);
		return IIO_AVAIL_LIST;

	default:
		return -EINVAL;
	}
}

static int mma8452_write_raw_get_fmt(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;

	case IIO_CHAN_INFO_SAMP_FREQ:
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}
}

static irqreturn_t mma8452_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct mma8452_data *data = iio_priv(indio_dev);
	struct mma8452_scan scan;
	int ret;

	memset(&scan, 0, sizeof(scan));

	mutex_lock(&data->lock);
	ret = mma8452_read_xyz(data,
			       &scan.channels[0],
			       &scan.channels[1],
			       &scan.channels[2]);
	mutex_unlock(&data->lock);

	if (ret == 0)
		iio_push_to_buffers_with_timestamp(indio_dev,
						   &scan,
						   pf->timestamp);

	iio_trigger_notify_done(data->trig);

	return IRQ_HANDLED;
}

static const struct iio_info mma8452_iio_info = {
	.read_raw = mma8452_read_raw,
	.write_raw = mma8452_write_raw,
	.write_raw_get_fmt = mma8452_write_raw_get_fmt,
	.read_avail = mma8452_read_avail,
};

static const struct iio_trigger_ops mma8452_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};

#define MMA8452_ACCEL_CHAN(_axis, _addr)				\
	{								\
		.type = IIO_ACCEL,					\
		.modified = 1,						\
		.channel2 = IIO_MOD_##_axis,				\
		.address = _addr,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type =				\
			BIT(IIO_CHAN_INFO_SCALE) |			\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
		.info_mask_shared_by_type_available =			\
			BIT(IIO_CHAN_INFO_SCALE) |			\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
		.scan_index = _addr,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 12,					\
			.storagebits = 16,				\
			.shift = 0,					\
			.endianness = IIO_CPU,				\
		},							\
	}

static const unsigned long mma8452_scan_masks[] = {
	BIT(0) | BIT(1) | BIT(2),
	0
};

static const struct iio_chan_spec mma8452_channels[] = {
	MMA8452_ACCEL_CHAN(X, 0),
	MMA8452_ACCEL_CHAN(Y, 1),
	MMA8452_ACCEL_CHAN(Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static int mma8452_chip_init(struct mma8452_data *data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(data->client, MMA8452_CTRL_REG1, 0x00);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(data->client, MMA8452_XYZ_DATA_CFG, 0x00);
	if (ret < 0)mma8452_scan_masks
		return ret;

	ret = i2c_smbus_write_byte_data(data->client,
					MMA8452_CTRL_REG4,
					MMA8452_INT_DRDY);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(data->client,
					MMA8452_CTRL_REG5,
					MMA8452_INT_DRDY);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(data->client,
					MMA8452_CTRL_REG1,
					(3 << MMA8452_CTRL_REG1_DR_SHIFT) |
					MMA8452_CTRL_REG1_ACTIVE);
	if (ret < 0)
		return ret;

	data->fs_bits = 0;
	data->odr_bits = 3;

	return 0;
}

static irqreturn_t mma8452_irq_thread(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct mma8452_data *data = iio_priv(indio_dev);
	int source;

	mutex_lock(&data->lock);
	source = i2c_smbus_read_byte_data(data->client, MMA8452_INT_SOURCE);
	mutex_unlock(&data->lock);

	if (source < 0)
		return IRQ_HANDLED;

	if ((source & MMA8452_INT_DRDY) && data->trig)
		iio_trigger_poll(data->trig);

	return IRQ_HANDLED;
}

static int mma8452_probe(struct i2c_client *client)
{
	struct iio_dev *indio_dev;
	struct mma8452_data *data;
	int ret;

	ret = i2c_smbus_read_byte_data(client, MMA8452_WHO_AM_I);
	if (ret < 0) {
		dev_err(&client->dev, "WHO_AM_I read failed: %d\n", ret);
		return ret;
	}

	if (ret != MMA8452_WHO_AM_I_VAL) {
		dev_err(&client->dev, "unexpected WHO_AM_I = 0x%02x\n", ret);
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->lock);

	i2c_set_clientdata(client, indio_dev);

	indio_dev->name = "mma8452q";
	indio_dev->info = &mma8452_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
	indio_dev->channels = mma8452_channels;
	indio_dev->num_channels = ARRAY_SIZE(mma8452_channels);
	indio_dev->available_scan_masks = mma8452_scan_masks;

	ret = mma8452_chip_init(data);
	if (ret < 0) {
		dev_err(&client->dev, "chip init failed: %d\n", ret);
		return ret;
	}

	data->trig = devm_iio_trigger_alloc(&client->dev,
					    "%s-trigger",
					    indio_dev->name);
	if (!data->trig)
		return -ENOMEM;

	data->trig->ops = &mma8452_trigger_ops;
	iio_trigger_set_drvdata(data->trig, indio_dev);

	ret = devm_iio_trigger_register(&client->dev, data->trig);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register trigger: %d\n", ret);
		return ret;
	}

	indio_dev->trig = iio_trigger_get(data->trig);

	ret = devm_iio_triggered_buffer_setup(&client->dev,
					      indio_dev,
					      iio_pollfunc_store_time,
					      mma8452_trigger_handler,
					      NULL);
	if (ret < 0) {
		iio_trigger_put(indio_dev->trig);
		indio_dev->trig = NULL;
		dev_err(&client->dev, "triggered buffer setup failed: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "client irq = %d\n", client->irq);

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(&client->dev,
						client->irq,
						NULL,
						mma8452_irq_thread,
						IRQF_ONESHOT,
						"mma8452q",
						indio_dev);
		if (ret < 0) {
			iio_trigger_put(indio_dev->trig);
			indio_dev->trig = NULL;
			dev_err(&client->dev, "failed to request irq: %d\n", ret);
			return ret;
		}

		dev_info(&client->dev, "irq registered: %d\n", client->irq);
	} else {
		dev_info(&client->dev, "no irq configured\n");
	}

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0) {
		iio_trigger_put(indio_dev->trig);
		indio_dev->trig = NULL;
		dev_err(&client->dev, "iio registration failed: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev,
		 "MMA8452Q IIO registered with interrupt trigger and buffer\n");

	return 0;
}

static void mma8452_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct mma8452_data *data;

	if (!indio_dev)
		return;

	data = iio_priv(indio_dev);

	mma8452_set_active(data, false);

	if (indio_dev->trig) {
		iio_trigger_put(indio_dev->trig);
		indio_dev->trig = NULL;
	}

	dev_info(&client->dev, "removed\n");
}

static const struct of_device_id mma8452_of_match[] = {
	{ .compatible = "nxp,mma8452q" },
	{ }
};
MODULE_DEVICE_TABLE(of, mma8452_of_match);

static const struct i2c_device_id mma8452_id[] = {
	{ "mma8452q", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mma8452_id);

static struct i2c_driver mma8452_driver = {
	.driver = {
		.name = "mma8452q",
		.of_match_table = mma8452_of_match,
	},
	.probe = mma8452_probe,
	.remove = mma8452_remove,
	.id_table = mma8452_id,	
};

module_i2c_driver(mma8452_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andronie Vasile-Laurentiu");
MODULE_DESCRIPTION("MMA8452Q IIO accelerometer driver with interrupt trigger and buffer");