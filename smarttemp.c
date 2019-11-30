// SPDX-License-Identifier: GPL-2.0
/*
 * Hwmon client for ATA/SATA hard disk drives with temperature sensors
 * (c) 2019 Guenter Roeck
 *
 * Derived from:
 *    Hwmon client for S.M.A.R.T. hard disk drives with temperature sensors.
 *    (C) 2018 Linus Walleij
 *
 *    hwmon: Driver for SCSI/ATA temperature sensors
 *    by Constantin Baranov <const@mimas.ru>, submitted September 2009
 *
 * The primary means to read hard drive temperatures and temperature limits
 * is the SCT Command Transport feature set as specified in ATA8-ACS.
 * It can be used to read the current drive temperature, temperature limits,
 * and historic minimum and maximum temperatures. The SCT Command Transport
 * feature set is documented in "AT Attachment 8 - ATA/ATAPI Command Set
 * (ATA8-ACS)".
 *
 * If the SCT Command Transport feature set is not available, drive temperatures
 * may be readable through SMART attributes. Since SMART attributes are not well
 * defined, this method is only used as fallback mechanism.
 *
 * There are three SMART attributes which may report drive temperatures.
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
#include <linux/bits.h>
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
	bool have_sct_temp;		/* reading temperature w/ SCT status */
	bool have_temp_lowest;		/* lowest temp in SCT status */
	bool have_temp_highest;		/* highest temp in SCT status */
	bool have_temp_min;		/* have min temp */
	bool have_temp_max;		/* have max temp */
	bool have_temp_lcrit;		/* have lower critical limit */
	bool have_temp_crit;		/* have critical limit */
	int temp_min;			/* min temp */
	int temp_max;			/* max temp */
	int temp_lcrit;			/* lower critical limit */
	int temp_crit;			/* critical limit */
};

static LIST_HEAD(smarttemp_devlist);

#define ATA_MAX_SMART_ATTRS	30
#define SMART_TEMP_PROP_190	190
#define SMART_TEMP_PROP_194	194

#define ATA_IDENTIFY_DEVICE	0xec
#define  IDENTIFY_SCT_TRANSPORT		(206 * 2)
#define SCT_STATUS_REQ		0xe0
#define  SMART_READ_LOG		0xd5
#define  SMART_WRITE_LOG	0xd6
#define  SCT_STATUS_VERSION_LOW	0	/* log byte offsets */
#define  SCT_STATUS_VERSION_HIGH	1
#define  SCT_STATUS_TEMP		200
#define  SCT_STATUS_TEMP_LOWEST		201
#define  SCT_STATUS_TEMP_HIGHEST	202

#define INVALID_TEMP			0x80

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

static int smarttemp_scsi_command(struct smarttemp_data *st, u8 ata_command,
				  u8 feature,
				  u8 lba_low, u8 lba_mid, u8 lba_high)
{
	static u8 scsi_cmd[MAX_COMMAND_SIZE];
	int data_dir;

	/* ATA command */
	memset(scsi_cmd, 0, sizeof(scsi_cmd));
	scsi_cmd[0] = ATA_16;
	if (feature == SMART_WRITE_LOG) {
		scsi_cmd[1] = (5 << 1); /* PIO Data-out */
		/*
		 * No off.line or cc, write to dev, block count in sector count
		 * field.
		 */
		scsi_cmd[2] = 0x06;
		data_dir = DMA_TO_DEVICE;
	} else {
		scsi_cmd[1] = (4 << 1); /* PIO Data-in */
		/*
		 * No off.line or cc, read from dev, block count in sector count
		 * field.
		 */
		scsi_cmd[2] = 0x0e;
		data_dir = DMA_FROM_DEVICE;
	}
	scsi_cmd[4] = feature;
	scsi_cmd[6] = 1; /* 1 sector */
	scsi_cmd[8] = lba_low;
	scsi_cmd[10] = lba_mid;
	scsi_cmd[12] = lba_high;
	scsi_cmd[14] = ata_command;

	return scsi_execute_req(st->sdev, scsi_cmd, data_dir,
				st->smartdata, ATA_SECT_SIZE, NULL, HZ, 5,
				NULL);
}

static int smarttemp_ata_command(struct smarttemp_data *st, u8 feature,
				 u8 select)
{
	return smarttemp_scsi_command(st, ATA_CMD_SMART, feature, select,
				      ATA_SMART_LBAM_PASS, ATA_SMART_LBAH_PASS);
}

static int smarttemp_read_smarttemp(struct smarttemp_data *st, long *temp)
{
	u8 *buf = st->smartdata;
	bool have_temp = false;
	int nattrs, i;
	u8 temp_raw;
	u8 csum;
	int err;

	err = smarttemp_ata_command(st, ATA_SMART_READ_VALUES, 0);
	if (err)
		return err;

	/* Checksum the read value table */
	csum = 0;
	for (i = 0; i < ATA_SECT_SIZE; i++)
		csum += buf[i];
	if (csum) {
		dev_dbg(&st->sdev->sdev_gendev,
			"checksum error reading SMART values\n");
		return -EIO;
	}

	nattrs = min_t(int, ATA_MAX_SMART_ATTRS, ATA_SECT_SIZE / 12);
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

static int smarttemp_identify_features(struct smarttemp_data *st)
{
	u8 *buf = st->smartdata;
	bool have_sct_data_table;
	bool have_sct_status;
	u16 version;
	long temp;
	int err;

	err = smarttemp_scsi_command(st, ATA_IDENTIFY_DEVICE, 0, 0, 0, 0);
	if (err)
		goto skip_sct;

	have_sct_data_table = buf[IDENTIFY_SCT_TRANSPORT] & BIT(5);
	have_sct_status = buf[IDENTIFY_SCT_TRANSPORT] & BIT(0);

	if (!have_sct_status)
		goto skip_sct_status;

	err = smarttemp_ata_command(st, SMART_READ_LOG, SCT_STATUS_REQ);
	if (err)
		goto skip_sct_status;

	version = (buf[SCT_STATUS_VERSION_HIGH] << 8) |
		  buf[SCT_STATUS_VERSION_LOW];
	if (version != 2 && version != 3)
		goto skip_sct_status;

	st->have_sct_temp = buf[SCT_STATUS_TEMP] != INVALID_TEMP;
	if (!st->have_sct_temp)
		goto skip_sct_status;

	st->have_temp_lowest = buf[SCT_STATUS_TEMP_LOWEST] != INVALID_TEMP;
	st->have_temp_highest = buf[SCT_STATUS_TEMP_HIGHEST] != INVALID_TEMP;

skip_sct_status:
	if (!have_sct_data_table)
		goto skip_sct;

	/* Request and read temperature history table */
	memset(buf, '\0', sizeof(st->smartdata));
	buf[0] = 5;	/* data table command */
	buf[2] = 1;	/* read table */
	buf[4] = 2;	/* temperature history table */

	err = smarttemp_ata_command(st, SMART_WRITE_LOG, SCT_STATUS_REQ);
	if (err)
		goto skip_sct_data;

	err = smarttemp_ata_command(st, SMART_READ_LOG, 0xe1);
	if (err)
		goto skip_sct_data;

	/* Temperature limits per AT Attachment 8 -
	 * ATA/ATAPI Command Set (ATA8-ACS)
	 */
	st->have_temp_max = buf[6] != INVALID_TEMP;
	st->have_temp_crit = buf[7] != INVALID_TEMP;
	st->have_temp_min = buf[8] != INVALID_TEMP;
	st->have_temp_lcrit = buf[9] != INVALID_TEMP;

	st->temp_max = ((s8)buf[6]) * 1000;
	st->temp_crit = ((s8)buf[7]) * 1000;
	st->temp_min = ((s8)buf[8]) * 1000;
	st->temp_lcrit = ((s8)buf[9]) * 1000;

skip_sct_data:
	if (st->have_sct_temp)
		return 0;
skip_sct:
	return smarttemp_read_smarttemp(st, &temp);
}

static int smarttemp_read_temp(struct smarttemp_data *st, u32 attr,
			       long *val)
{
	u8 *buf = st->smartdata;
	int err;

	if (st->have_sct_temp) {
		err = smarttemp_ata_command(st, SMART_READ_LOG, SCT_STATUS_REQ);
		if (err)
			return err;
		switch (attr) {
		case hwmon_temp_input:
			*val = ((char)buf[SCT_STATUS_TEMP]) * 1000;
			break;
		case hwmon_temp_lowest:
			*val = ((char)buf[SCT_STATUS_TEMP_LOWEST] * 1000);
			break;
		case hwmon_temp_highest:
			*val = ((char)buf[SCT_STATUS_TEMP_HIGHEST]) * 1000;
			break;
		default:
			err = -EINVAL;
			break;
		}
		return err;
	}

	return smarttemp_read_smarttemp(st, val);
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
	case hwmon_temp_lowest:
	case hwmon_temp_highest:
		err = smarttemp_read_temp(st, attr, val);
		break;
	case hwmon_temp_lcrit:
		*val = st->temp_lcrit;
		break;
	case hwmon_temp_min:
		*val = st->temp_min;
		break;
	case hwmon_temp_max:
		*val = st->temp_max;
		break;
	case hwmon_temp_crit:
		*val = st->temp_crit;
		break;
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
	const struct smarttemp_data *st = data;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			return 0444;
		case hwmon_temp_lowest:
			if (st->have_temp_lowest)
				return 0444;
			break;
		case hwmon_temp_highest:
			if (st->have_temp_highest)
				return 0444;
			break;
		case hwmon_temp_min:
			if (st->have_temp_min)
				return 0444;
			break;
		case hwmon_temp_max:
			if (st->have_temp_max)
				return 0444;
			break;
		case hwmon_temp_lcrit:
			if (st->have_temp_lcrit)
				return 0444;
			break;
		case hwmon_temp_crit:
			if (st->have_temp_crit)
				return 0444;
			break;
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
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT |
			   HWMON_T_LOWEST | HWMON_T_HIGHEST |
			   HWMON_T_MIN | HWMON_T_MAX |
			   HWMON_T_LCRIT | HWMON_T_CRIT),
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

	if (smarttemp_identify_features(st)) {
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
