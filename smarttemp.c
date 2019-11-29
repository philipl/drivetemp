// SPDX-License-Identifier: GPL-2.0
/*
 * Hwmon client for S.M.A.R.T. hard disk drives with temperature sensors
 * (c) 2019 Guenter Roeck
 *
 * Derived from:
 *    Hwmon client for S.M.A.R.T. hard disk drives with temperature sensors.
 *    (C) 2018 Linus Walleij
 *
 *    hwmon: Driver for SCSI/ATA temperature sensors
 *    by Constantin Baranov <const@mimas.ru>, submitted September 2009
 *
 * There are three SMART attributes reporting drive temperatures.
 * Those are defined as follows (from
 * http://www.cropel.com/library/smart-attribute-list.aspx).
 *
 * 190	Temperature	Temperature, monitored by a sensor somewhere inside
 * 			the drive. Raw value typicaly holds the actual
 * 			temperature (hexadecimal) in its rightmost two digits.
 *
 * 194	Temperature	Temperature, monitored by a sensor somewhere inside
 * 			the drive. Raw value typicaly holds the actual
 * 			temperature (hexadecimal) in its rightmost two digits.
 *
 * 231	Temperature	Temperature, monitored by a sensor somewhere inside
 * 			the drive. Raw value typicaly holds the actual
 * 			temperature (hexadecimal) in its rightmost two digits.
 *
 * Wikipedia defines attributes a bit differently.
 *
 * 190	Temperature	Value is equal to (100-temp. °C), allowing manufacturer
 *	Difference or	to set a minimum threshold which corresponds to a
 *	Airflow		maximum temperature. This also follows the convention of
 *	Temperature	100 being a best-case value and lower values being
 *			undesirable. However, some older drives may instead
 *			report raw Temperature (identical to 0xC2) or
 *			Temperature minus 50 here.
 * 194	Temperature or	Indicates the device temperature, if the appropriate
 *	Temperature	sensor is fitted. Lowest byte of the raw value contains
 *	Celsius		the exact temperature value (Celsius degrees).
 * 231	Life Left	Indicates the approximate SSD life left, in terms of
 *	(SSDs) or	program/erase cycles or available reserved blocks.
 *	Temperature	A normalized value of 100 represents a new drive, with
 *			a threshold value at 10 indicating a need for
 *			replacement. A value of 0 may mean that the drive is
 *			operating in read-only mode to allow data recovery.
 *			Previously (pre-2010) occasionally used for Drive
 *			Temperature (more typically reported at 0xC2).
 *
 * Common denominator is that the first raw byte reports the temperature
 * in degrees C on almost all drives. Some drives may report a fractional
 * temperature in the second raw byte.
 *
 * Known exceptions (from libatasmart):
 * - SAMSUNG SV0412H and SAMSUNG SV1204H) report the temperature in 10th
 *   degrees C in the first two raw bytes.
 * - A few Maxtor drives report an unknown or bad value in attribute 194.
 * - Certain Apple SSD drives report an unknown value in attribute 190.
 *   Only certain firmware versions are affected.
 *
 * Those exceptions affect older ATA drives and are currently ignored.
 * Also, the second raw byte (possibly reporting the fractional temperature)
 * is currently ignored.
 *
 * Many drives also report temperature limits in additional raw bytes.
 * The format of those is not well defined and varies widely. The driver
 * does not currently attempt to report those limits.
 *
 * According to data in smartmontools, attribute 231 is rarely used to report
 * drive temperatures. At the same time, several drives report SSD life left
 * in attribute 231, but do not support temperature sensors. For this reason,
 * attribute 231 is currently ignored.
 *
 * Following above definitions, temperatures are reported as follows.
 * - If attribute 194 is supported, it is used to read the temperature.
 * - If attribute 194 is not supported, attribute 190 is used to read the
 *   temperature if it is supported.
 */

#include <linux/ata.h>
#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_driver.h>
#include <scsi/scsi_proto.h>

struct smarttemp_data {
	struct list_head list;		/* list of instantiated devices */
	struct scsi_device *sdev;	/* SCSI device */
	struct device *dev;		/* instantiating device */
	struct device *hwdev;		/* hardware monitoring device */
	u8 smartdata[ATA_SECT_SIZE];	/* local buffer */
};

static LIST_HEAD(smarttemp_devlist);

#define ATA_MAX_SMART_ATTRS	30
#define SMART_TEMP_PROP_190	190
#define SMART_TEMP_PROP_194	194

static int smarttemp_identify_ata(struct scsi_device *sdev)
{
	/* Use cached SCSI inquiry response to identify ATA devices */
	if (!sdev->inquiry || sdev->inquiry_len < 16)
		return -ENODEV;

	/* libata reports the SCSI Vendor ID as "ATA" */
	if (strncmp(&sdev->inquiry[8], "ATA     ", 8))
		return -ENODEV;

	return 0;
}

static int smarttemp_read_temp(struct smarttemp_data *st, long *temp)
{
	static u8 scsi_cmd[MAX_COMMAND_SIZE];
	u8 *buf = st->smartdata;
	bool have_temp = false;
	int resid, err;
	int nattrs, i;
	u8 temp_raw;
	u8 csum;

	/* ATA command to read SMART values */
	memset(scsi_cmd, 0, sizeof(scsi_cmd));
	scsi_cmd[0] = ATA_16;
	scsi_cmd[1] = (4 << 1); /* PIO Data-in */
	/*
	 * No off.line or cc, read from dev, block count in sector count
	 * field.
	 */
	scsi_cmd[2] = 0x0e;
	scsi_cmd[4] = ATA_SMART_READ_VALUES;
	scsi_cmd[6] = 1; /* 1 sector */
	scsi_cmd[8] = 0; /* args[1]; */
	scsi_cmd[10] = ATA_SMART_LBAM_PASS;
	scsi_cmd[12] = ATA_SMART_LBAH_PASS;
	scsi_cmd[14] = ATA_CMD_SMART;

	err = scsi_execute_req(st->sdev, scsi_cmd, DMA_FROM_DEVICE,
			       buf, ATA_SECT_SIZE, NULL, HZ, 5, &resid);
	if (err)
		return err;

	/* Checksum the read value table */
	csum = 0;
	for (i = 0; i < ATA_SECT_SIZE; i++)
		csum += buf[i];
	if (csum) {
		dev_dbg(&st->sdev->sdev_gendev, "checksum error reading SMART values\n");
		return -EIO;
	}

	nattrs = min_t(int, ATA_MAX_SMART_ATTRS, (ATA_SECT_SIZE - resid) / 12);
	for (i = 0; i < nattrs; i++) {
		u8 *attr = buf + i * 12;
		int id = attr[2];

		if (!id)
			continue;

		if (id == SMART_TEMP_PROP_190) {
			temp_raw = attr[7];
			have_temp = true;
		}
		if (id == SMART_TEMP_PROP_194) {
			temp_raw = attr[7];
			have_temp = true;
			break;
		}
	}

	if (have_temp) {
		*temp = temp_raw * 1000;
		return 0;
	}

	return -ENXIO;
}

static int smarttemp_read(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long *val)
{
	struct smarttemp_data *st = dev_get_drvdata(dev);
	int err = 0;

	if (type != hwmon_temp)
		return -EINVAL;

	switch (attr) {
	case hwmon_temp_input:
		err = smarttemp_read_temp(st, val);
		break;
	case hwmon_temp_max:
	case hwmon_temp_min:
	default:
		err = -EINVAL;
		break;
	}
	return err;
}

static umode_t smarttemp_is_visible(const void *data,
				    enum hwmon_sensor_types type,
				    u32 attr, int channel)
{
	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			return 0444;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return 0;
}

static const struct hwmon_channel_info *smarttemp_info[] = {
	HWMON_CHANNEL_INFO(chip,
			   HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT),
	NULL
};

static const struct hwmon_ops smarttemp_ops = {
	.is_visible = smarttemp_is_visible,
	.read = smarttemp_read,
};

static const struct hwmon_chip_info smarttemp_chip_info = {
	.ops = &smarttemp_ops,
	.info = smarttemp_info,
};

/*
 * The device argument points to sdev->sdev_dev. Its parent is
 * sdev->sdev_gendev, which we can use to get the scsi_device pointer.
 */
static int smarttemp_add(struct device *dev, struct class_interface *intf)
{
	struct scsi_device *sdev = to_scsi_device(dev->parent);
	struct smarttemp_data *st;
	long temp;
	int err;

	/* Bail out immediately if this is not an ATA device */
	err = smarttemp_identify_ata(sdev);
	if (err)
		return err;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->sdev = sdev;
	st->dev = dev;

	/* If reading the sensor fails, assume we don't have one and bail out */
	if (smarttemp_read_temp(st, &temp)) {
		err = -ENODEV;
		goto abort;
	}

	st->hwdev = hwmon_device_register_with_info(dev->parent, "smarttemp",
						    st, &smarttemp_chip_info,
						    NULL);
	if (IS_ERR(st->hwdev)) {
		err = PTR_ERR(st->hwdev);
		goto abort;
	}

	list_add(&st->list, &smarttemp_devlist);
	return 0;

abort:
	kfree(st);
	return err;
}

static void smarttemp_remove(struct device *dev, struct class_interface *intf)
{
	struct smarttemp_data *st, *tmp;

	list_for_each_entry_safe(st, tmp, &smarttemp_devlist, list) {
		if (st->dev == dev) {
			list_del(&st->list);
			hwmon_device_unregister(st->hwdev);
			kfree(st);
			break;
		}
	}
}

static struct class_interface smarttemp_interface = {
	.add_dev = smarttemp_add,
	.remove_dev = smarttemp_remove,
};

static int __init smarttemp_init(void)
{
	return scsi_register_interface(&smarttemp_interface);
}

static void __exit smarttemp_exit(void)
{
	scsi_unregister_interface(&smarttemp_interface);
}

module_init(smarttemp_init);
module_exit(smarttemp_exit);

MODULE_AUTHOR("Guenter Roeck <linus@roeck-us.net>");
MODULE_DESCRIPTION("ATA temperature monitor");
MODULE_LICENSE("GPL");
