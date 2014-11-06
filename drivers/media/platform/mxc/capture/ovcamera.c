/*
 * Copyright (C) 2014 Au-Zone Technologies Inc.  All Rights Reserved.
 *
 * Video4Linux Slave (Internal) Device Driver for the Texas Instrument TVP5147m1
 * 
 * Implemented using the tvp514x reference driver but with support for Freescale
 * IPU Video Capture (V4L Internal vs. Subdevice).
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#define pr_fmt_dbg(fmt) pr_fmt(fmt)

#if 0
y#include <linux/delay.h>
y#include <linux/device.h>
y#include <linux/fsl_devices.h>
y#include <linux/i2c.h>
y#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/videodev2.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#else
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/sysfs.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"
#endif

// #include "../../tvp514x_regs.h"
#include "mxc_v4l2_capture.h"

// TO DO: to be put in ovcamera_regs.h file:
#define OV5640_CHIP_ID_HIGH_BYTE        0x300A
#define OV5640_CHIP_ID_LOW_BYTE         0x300B
#define OV5640_XCLK_MIN 6000000
#define OV5640_XCLK_MAX 24000000

enum ov_camera_type {
	ov_camera_none,
	ov_camera_5640,
	ov_camera_10633
};

enum ov5640_mode {
    ov5640_mode_MIN = 0,
    ov5640_mode_VGA_640_480 = 0,
    ov5640_mode_QVGA_320_240 = 1,
    ov5640_mode_NTSC_720_480 = 2,
    ov5640_mode_PAL_720_576 = 3,
    ov5640_mode_720P_1280_720 = 4,
    ov5640_mode_1080P_1920_1080 = 5,
    ov5640_mode_QSXGA_2592_1944 = 6,
    ov5640_mode_QCIF_176_144 = 7,
    ov5640_mode_XGA_1024_768 = 8,
    ov5640_mode_MAX = 8
};

// TO DO: end of ovcamera_regs.h file

// HACK ... need to ensure that we are able to either derive this for each instance
static enum ov_camera_type ov_selected_camera = ov_camera_none;
static int pwn_gpio, rst_gpio;

// ** End of HACK

#define VIDEO_SELECT_AUTO       0
#define VIDEO_SELECT_NTSC       1
#define VIDEO_SELECT_PAL        2
#define VIDEO_SELECT_PALM       3
#define VIDEO_SELECT_PALN       4
#define VIDEO_SELECT_NTSC_443   5
#define VIDEO_SELECT_SECAM      6
#define VIDEO_SELECT_PAL60      7

static ssize_t attr_show(struct kobject*, struct kobj_attribute*, char*);
static ssize_t attr_store(struct kobject*, struct kobj_attribute*, const char*, size_t);

static int ovcamera_write_reg(struct i2c_client*, u16 reg, u8 val);
static int ovcamera_read_reg(struct i2c_client*, u16 reg, u8 *val);

static struct v4l2_queryctrl ovcamera_qctrl[] = {
    {
        .id = V4L2_CID_BRIGHTNESS,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "Brightness",
        .minimum = 0,
        .maximum = 255,
        .step = 1,
        .default_value = 127,
        .flags = 0,
    },
    {
        .id = V4L2_CID_CONTRAST,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "Contrast",
        .minimum = 0,
        .maximum = 255,
        .step = 1,
        .default_value = 127,
        .flags = 0,
    },
    {
        .id = V4L2_CID_SATURATION,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "Saturation",
        .minimum = 0,
        .maximum = 255,
        .step = 1,
        .default_value = 127,
        .flags = 0,
    },
    {
        .id = V4L2_CID_HUE,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "Hue",
        .minimum = 0,
        .maximum = 255,
        .step = 1,
        .default_value = 127,
        .flags = 0,
    },
};

/* Base Functionality */
static struct kobj_attribute attr_info
    = __ATTR(info, 0444, attr_show, NULL);
static struct kobj_attribute attr_status
    = __ATTR(status, 0444, attr_show, NULL);
static struct kobj_attribute attr_signal
    = __ATTR(signal, 0444, attr_show, NULL);
static struct kobj_attribute attr_sleep
    = __ATTR(sleep, 0666, attr_show, attr_store);
static struct kobj_attribute attr_brightness
    = __ATTR(brightness, 0666, attr_show, attr_store);
static struct kobj_attribute attr_contrast
    = __ATTR(contrast, 0666, attr_show, attr_store);
static struct kobj_attribute attr_saturation
    = __ATTR(saturation, 0666, attr_show, attr_store);
static struct kobj_attribute attr_hue
    = __ATTR(hue, 0666, attr_show, attr_store);

static struct attribute *base_attributes[] = {
    (struct attribute*) &attr_info,
    (struct attribute*) &attr_status,
    (struct attribute*) &attr_signal,
    (struct attribute*) &attr_sleep,
    (struct attribute*) &attr_brightness,
    (struct attribute*) &attr_contrast,
    (struct attribute*) &attr_saturation,
    (struct attribute*) &attr_hue,
    NULL,
};


/* Syncs */
static struct kobj_attribute attr_avid_vblk_active
    = __ATTR(avid_vblk_active, 0666, attr_show, attr_store);
static struct kobj_attribute attr_avid_start
    = __ATTR(avid_start, 0666, attr_show, attr_store);
static struct kobj_attribute attr_avid_stop
    = __ATTR(avid_stop, 0666, attr_show, attr_store);
static struct kobj_attribute attr_hsync_start
    = __ATTR(hsync_start, 0666, attr_show, attr_store);
static struct kobj_attribute attr_hsync_stop
    = __ATTR(hsync_stop, 0666, attr_show, attr_store);
static struct kobj_attribute attr_vsync_start
    = __ATTR(vsync_start, 0666, attr_show, attr_store);
static struct kobj_attribute attr_vsync_stop
    = __ATTR(vsync_stop, 0666, attr_show, attr_store);
static struct kobj_attribute attr_vblk_start
    = __ATTR(vblk_start, 0666, attr_show, attr_store);
static struct kobj_attribute attr_vblk_stop
    = __ATTR(vblk_stop, 0666, attr_show, attr_store);

static struct attribute *sync_attributes[] = {
    (struct attribute*) &attr_avid_vblk_active,
    (struct attribute*) &attr_avid_start,
    (struct attribute*) &attr_avid_stop,
    (struct attribute*) &attr_hsync_start,
    (struct attribute*) &attr_hsync_stop,
    (struct attribute*) &attr_vsync_start,
    (struct attribute*) &attr_vsync_stop,
    (struct attribute*) &attr_vblk_start,
    (struct attribute*) &attr_vblk_stop,
    NULL,
};


/* General Registers */
static struct kobj_attribute attr_reg_inputsel
    = __ATTR(inputsel, 0666, attr_show, attr_store);
static struct kobj_attribute attr_reg_videostd
    = __ATTR(videostd, 0666, attr_show, attr_store);

static struct attribute *reg_attributes[] = {
    (struct attribute*) &attr_reg_inputsel,
    (struct attribute*) &attr_reg_videostd,
    NULL,
};


static struct attribute_group base_attribute_group = {
    .attrs = base_attributes,
};

static struct attribute_group sync_attribute_group = {
    .name = "sync",
    .attrs = sync_attributes,
};

static struct attribute_group reg_attribute_group = {
    .name = "registers",
    .attrs = reg_attributes,
};

static ssize_t attr_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    uint8_t val = 0;
    uint16_t wval = 0;
    struct i2c_client *i2c_client = to_i2c_client((struct device*) kobj);


    /* Base Group */
    if (strcmp(attr->attr.name, "info") == 0) {
        return sprintf(buf,
                       "I2C Client:    %lx\n"
                       "I2C Address:   %x\n"
                       "IPU Interface: %d\n",
                       (unsigned long) i2c_client,
                       i2c_client->addr,
                       -1);
    } else if (strcmp(attr->attr.name, "status") == 0) {
       //  ovcamera_read_reg(i2c_client, REG_STATUS1, &val);

        return sprintf(buf,
                       "Peak White:   %s\n"
                       "Line Status:  %s\n"
                       "Field Rate:   %dHz\n"
                       "Lost Lock:    %s\n"
                       "Colour Lock:  %s\n"
                       "VSync Status: %s\n"
                       "HSync Status: %s\n"
                       "Device Type:  %s\n",
                       val & 0x80 ? "detected" : "not detected",
                       val & 0x40 ? "line-alternating" : "non line-alternating",
                       val & 0x20 ? 50 : 60,
                       val & 0x10 ? "lost lock" : "no lost lock",
                       val & 0x08 ? "locked" : "unlocked",
                       val & 0x04 ? "locked" : "unlocked",
                       val & 0x02 ? "locked" : "unlocked",
                       val & 0x01 ? "VCR" : "TV");
    } else if (strcmp(attr->attr.name, "signal") == 0) {
        // ovcamera_read_reg(i2c_client, REG_STATUS2, &val);

        switch ((val >> 6) & 3) {
        case 0:
            return sprintf(buf, "no signal\n");
        case 1:
        case 3:
            return sprintf(buf, "weak signal\n");
        case 2:
            return sprintf(buf, "good signal\n");
        }
    } else if (strcmp(attr->attr.name, "sleep") == 0) {
        // ovcamera_read_reg(i2c_client, REG_OPERATION_MODE, &val);
        return sprintf(buf, "%d\n", val & 1);
    } else if (strcmp(attr->attr.name, "brightness") == 0) {
        // ovcamera_read_reg(i2c_client, REG_BRIGHTNESS, &val);
        return sprintf(buf, "%d\n", val);
    } else if (strcmp(attr->attr.name, "contrast") == 0) {
       //  ovcamera_read_reg(i2c_client, REG_CONTRAST, &val);
        return sprintf(buf, "%d\n", val);
    } else if (strcmp(attr->attr.name, "saturation") == 0) {
        // ovcamera_read_reg(i2c_client, REG_SATURATION, &val);
        return sprintf(buf, "%d\n", val);
    } else if (strcmp(attr->attr.name, "hue") == 0) {
       // ovcamera_read_reg(i2c_client, REG_HUE, &val);
        return sprintf(buf, "%d\n", val);
    }

    /* Sync Group */
    if (strcmp(attr->attr.name, "avid_vblk_active") == 0) {
        // ovcamera_read_reg(i2c_client, REG_AVID_START_PIXEL_MSB, &val);
        return sprintf(buf, "%d\n", val & 0x40 ? 1 : 0);
    } else if (strcmp(attr->attr.name, "avid_start") == 0) {
        // ovcamera_read_reg(i2c_client, REG_AVID_START_PIXEL_MSB, &val);
        wval = (val & 3) << 8;
        // ovcamera_read_reg(i2c_client, REG_AVID_START_PIXEL_LSB, &val);
        wval |= val;
        return sprintf(buf, "%d\n", wval);
    } else if (strcmp(attr->attr.name, "avid_stop") == 0) {
       //  ovcamera_read_reg(i2c_client, REG_AVID_STOP_PIXEL_MSB, &val);
        wval = (val & 3) << 8;
        // ovcamera_read_reg(i2c_client, REG_AVID_STOP_PIXEL_LSB, &val);
        wval |= val;
        return sprintf(buf, "%d\n", wval);
    } else if (strcmp(attr->attr.name, "hsync_start") == 0) {
       // ovcamera_read_reg(i2c_client, REG_HSYNC_START_PIXEL_MSB, &val);
        wval = (val & 3) << 8;
        // ovcamera_read_reg(i2c_client, REG_HSYNC_START_PIXEL_LSB, &val);
        wval |= val;
        return sprintf(buf, "%d\n", wval);
    } else if (strcmp(attr->attr.name, "hsync_stop") == 0) {
        // ovcamera_read_reg(i2c_client, REG_HSYNC_STOP_PIXEL_MSB, &val);
        wval = (val & 3) << 8;
        // ovcamera_read_reg(i2c_client, REG_HSYNC_STOP_PIXEL_LSB, &val);
        wval |= val;
        return sprintf(buf, "%d\n", wval);
    } else if (strcmp(attr->attr.name, "vsync_start") == 0) {
        // ovcamera_read_reg(i2c_client, REG_VSYNC_START_LINE_MSB, &val);
        wval = (val & 3) << 8;
        // ovcamera_read_reg(i2c_client, REG_VSYNC_START_LINE_LSB, &val);
        wval |= val;
        return sprintf(buf, "%d\n", wval);
    } else if (strcmp(attr->attr.name, "vsync_stop") == 0) {
       // ovcamera_read_reg(i2c_client, REG_VSYNC_STOP_LINE_MSB, &val);
        wval = (val & 3) << 8;
        // ovcamera_read_reg(i2c_client, REG_VSYNC_STOP_LINE_LSB, &val);
        wval |= val;
        return sprintf(buf, "%d\n", wval);
    } else if (strcmp(attr->attr.name, "vblk_start") == 0) {
        // ovcamera_read_reg(i2c_client, REG_VBLK_START_LINE_MSB, &val);
        wval = (val & 3) << 8;
        // ovcamera_read_reg(i2c_client, REG_VBLK_START_LINE_LSB, &val);
        wval |= val;
        return sprintf(buf, "%d\n", wval);
    } else if (strcmp(attr->attr.name, "vblk_stop") == 0) {
        // ovcamera_read_reg(i2c_client, REG_VBLK_STOP_LINE_MSB, &val);
        wval = (val & 3) << 8;
        // ovcamera_read_reg(i2c_client, REG_VBLK_STOP_LINE_LSB, &val);
        wval |= val;
        return sprintf(buf, "%d\n", wval);
    }

    /* Register Group */
    if (strcmp(attr->attr.name, "inputsel") == 0) {
		// ovcamera_read_reg(i2c_client, REG_INPUT_SEL, &val);
        return sprintf(buf, "0x%.2x\n", val);
    } else if (strcmp(attr->attr.name, "videostd") == 0) {
        // ovcamera_read_reg(i2c_client, REG_VIDEO_STD, &val);

        switch (val) {
        case VIDEO_SELECT_AUTO:
            return sprintf(buf, "%d: Auto-Select\n", val);
        case VIDEO_SELECT_NTSC:
            return sprintf(buf, "%d: NTSC\n", val);
        case VIDEO_SELECT_PAL:
            return sprintf(buf, "%d: PAL\n", val);
        case VIDEO_SELECT_PALM:
            return sprintf(buf, "%d: PAL (M)\n", val);
        case VIDEO_SELECT_PALN:
            return sprintf(buf, "%d: PAL (Combination-N)\n", val);
        case VIDEO_SELECT_NTSC_443:
            return sprintf(buf, "%d: NTSC 4.43\n", val);
        case VIDEO_SELECT_SECAM:
            return sprintf(buf, "%d: SECAM\n", val);
        case VIDEO_SELECT_PAL60:
            return sprintf(buf, "%d: PAL 60\n", val);
        default:
            return sprintf(buf, "%d: Unknown Standard\n", val);
        }
    }

    return -EINVAL;
}

static ssize_t attr_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    uint8_t val = 0;
    uint16_t wval = 0;
    struct i2c_client *i2c_client = to_i2c_client((struct device*) kobj);

    /* Base Group */
    if (strcmp(attr->attr.name, "sleep") == 0) {
        int sleep = simple_strtoul(buf, NULL, 0);
		/*
        ovcamera_read_reg(i2c_client, REG_OPERATION_MODE, &val);
        
        if (sleep) {
            val |= 1;
        } else {
            val ^= ~1;
        }

        ovcamera_write_reg(i2c_client, REG_OPERATION_MODE, val);
		*/
        return count;
    } else if (strcmp(attr->attr.name, "brightness") == 0) {
        val = simple_strtoul(buf, NULL, 0);
       //  ovcamera_write_reg(i2c_client, REG_BRIGHTNESS, val);
        return count;
    } else if (strcmp(attr->attr.name, "contrast") == 0) {
        val = simple_strtoul(buf, NULL, 0);
        // ovcamera_write_reg(i2c_client, REG_CONTRAST, val);
        return count;
    } else if (strcmp(attr->attr.name, "saturation") == 0) {
        val = simple_strtoul(buf, NULL, 0);
       //  ovcamera_write_reg(i2c_client, REG_SATURATION, val);
        return count;
    } else if (strcmp(attr->attr.name, "hue") == 0) {
        val = simple_strtoul(buf, NULL, 0);
       // ovcamera_write_reg(i2c_client, REG_HUE, val);
        return count;
    }

    /* Sync Group */
    if (strcmp(attr->attr.name, "avid_vblk_active") == 0) {
        int active = simple_strtoul(buf, NULL, 0);
		/*
        ovcamera_read_reg(i2c_client, REG_AVID_START_PIXEL_MSB, &val);

        if (active) {
            val |= 0x40;
        } else {
            val ^= ~0x40;
        }

        ovcamera_write_reg(i2c_client, REG_AVID_START_PIXEL_MSB, val);
		*/
        return count;
    } else if (strcmp(attr->attr.name, "avid_start") == 0) {
		/*
        ovcamera_read_reg(i2c_client, REG_AVID_START_PIXEL_MSB, &val);
        wval = simple_strtoul(buf, NULL, 0);
        val |= (wval >> 8) & 3;
        ovcamera_write_reg(i2c_client, REG_AVID_START_PIXEL_LSB, wval);
        ovcamera_write_reg(i2c_client, REG_AVID_START_PIXEL_MSB, val);
		*/
        return count;
    } else if (strcmp(attr->attr.name, "avid_stop") == 0) {
		/*
        ovcamera_read_reg(i2c_client, REG_AVID_STOP_PIXEL_MSB, &val);
        wval = simple_strtoul(buf, NULL, 0);
        val |= (wval >> 8) & 3;
        ovcamera_write_reg(i2c_client, REG_AVID_STOP_PIXEL_LSB, wval);
        ovcamera_write_reg(i2c_client, REG_AVID_STOP_PIXEL_MSB, val);
		*/
        return count;
    } else if (strcmp(attr->attr.name, "hsync_start") == 0) {
		/*
        ovcamera_read_reg(i2c_client, REG_HSYNC_START_PIXEL_MSB, &val);
        wval = simple_strtoul(buf, NULL, 0);
        val |= (wval >> 8) & 3;
        ovcamera_write_reg(i2c_client, REG_HSYNC_START_PIXEL_LSB, wval);
        ovcamera_write_reg(i2c_client, REG_HSYNC_START_PIXEL_MSB, val);
		*/
        return count;
    } else if (strcmp(attr->attr.name, "hsync_stop") == 0) {
		/*
        ovcamera_read_reg(i2c_client, REG_HSYNC_STOP_PIXEL_MSB, &val);
        wval = simple_strtoul(buf, NULL, 0);
        val |= (wval >> 8) & 3;
        ovcamera_write_reg(i2c_client, REG_HSYNC_STOP_PIXEL_LSB, wval);
        ovcamera_write_reg(i2c_client, REG_HSYNC_STOP_PIXEL_MSB, val);
		*/
        return count;
    } else if (strcmp(attr->attr.name, "vsync_start") == 0) {
		/*
        ovcamera_read_reg(i2c_client, REG_VSYNC_START_LINE_MSB, &val);
        wval = simple_strtoul(buf, NULL, 0);
        val |= (wval >> 8) & 3;
        ovcamera_write_reg(i2c_client, REG_VSYNC_START_LINE_LSB, wval);
        ovcamera_write_reg(i2c_client, REG_VSYNC_START_LINE_MSB, val);
		*/
        return count;
    } else if (strcmp(attr->attr.name, "vsync_stop") == 0) {
		/*
        ovcamera_read_reg(i2c_client, REG_VSYNC_STOP_LINE_MSB, &val);
        wval = simple_strtoul(buf, NULL, 0);
        val |= (wval >> 8) & 3;
        ovcamera_write_reg(i2c_client, REG_VSYNC_STOP_LINE_LSB, wval);
        ovcamera_write_reg(i2c_client, REG_VSYNC_STOP_LINE_MSB, val);
		*/
        return count;
    } else if (strcmp(attr->attr.name, "vblk_start") == 0) {
		/*
        ovcamera_read_reg(i2c_client, REG_VBLK_START_LINE_MSB, &val);
        wval = simple_strtoul(buf, NULL, 0);
        val |= (wval >> 8) & 3;
        ovcamera_write_reg(i2c_client, REG_VBLK_START_LINE_LSB, wval);
        ovcamera_write_reg(i2c_client, REG_VBLK_START_LINE_MSB, val);
		*/
        return count;
    } else if (strcmp(attr->attr.name, "vblk_stop") == 0) {
		/*
        ovcamera_read_reg(i2c_client, REG_VBLK_STOP_LINE_MSB, &val);
        wval = simple_strtoul(buf, NULL, 0);
        val |= (wval >> 8) & 3;
        ovcamera_write_reg(i2c_client, REG_VBLK_STOP_LINE_LSB, wval);
        ovcamera_write_reg(i2c_client, REG_VBLK_STOP_LINE_MSB, val);
		*/
        return count;
    }

    /* Register Group */
    if (strcmp(attr->attr.name, "inputsel") == 0) {
		/*
        val = simple_strtoul(buf, NULL, 16);
        ovcamera_write_reg(i2c_client, REG_INPUT_SEL, val);
		*/
        return count;
    } else if (strcmp(attr->attr.name, "videostd") == 0) {
		/*
        val = simple_strtoul(buf, NULL, 0);

        if (val > 7) {
            pr_warning("invalid video standard selected: %d\n", val);
            return -EINVAL;
        }

        ovcamera_write_reg(i2c_client, REG_VIDEO_STD, val);
		*/
        return count;
    }

    return -EINVAL;
}


static int ovcamera_write_reg(struct i2c_client* i2c_client, u16 reg, u8 val)
{
    u8 au8Buf[3] = {0};

    au8Buf[0] = reg >> 8;
    au8Buf[1] = reg & 0xff;
    au8Buf[2] = val;

    if (i2c_master_send(i2c_client, au8Buf, 3) < 0) {
        pr_err("%s:write reg error:reg=%x,val=%x\n",
            __func__, reg, val);
        return -1;
    }

   //  pr_info("ovcamera_5640: write_reg %4x value %2x \n", reg, val );

    return 0;
}


static int ovcamera_read_reg(struct i2c_client* i2c_client, u16 reg, u8 *val)
{

    u8 au8RegBuf[2] = {0};
    u8 u8RdVal = 0;

    au8RegBuf[0] = reg >> 8;
    au8RegBuf[1] = reg & 0xff;


    if (2 != i2c_master_send(i2c_client, au8RegBuf, 2)) {
        pr_err("%s:write reg error:reg=%x\n",
                __func__, reg);
        return -1;
    }

    if (1 != i2c_master_recv(i2c_client, &u8RdVal, 1)) {
        pr_err("%s:read reg error:reg=%x,val=%x\n",
                __func__, reg, u8RdVal);
        return -1;
    }

    *val = u8RdVal;

    return u8RdVal;
}

static int
ovcamera_ioctl_g_ifparam(struct v4l2_int_device *idev, struct v4l2_ifparm *param)
{

    struct sensor_data *sensor = idev->priv;

    pr_info("%s \n", __func__ );
    if (idev == NULL) {
        pr_err("   ERROR!! no slave device set!\n");
        return -1;
    }
    if ( ov_selected_camera == ov_camera_5640 )
    {
        memset(param, 0, sizeof(*param));
        param->u.bt656.clock_curr = sensor->mclk;
        pr_info("   clock_curr=mclk=%d\n", sensor->mclk);
        param->if_type = V4L2_IF_TYPE_BT656;
        param->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
        param->u.bt656.clock_min = OV5640_XCLK_MIN;
        param->u.bt656.clock_max = OV5640_XCLK_MAX;
        param->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */
    }
    else if ( ov_selected_camera == ov_camera_10633 )
    {
        // same for now (clocks are the same for both chips)

        memset(param, 0, sizeof(*param));

        param->u.bt656.clock_curr = sensor->mclk;
        pr_info("   clock_curr=mclk=%d\n", sensor->mclk);
        param->if_type = V4L2_IF_TYPE_BT656;
        param->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT; //V4L2_IF_TYPE_BT656_MODE_NOBT_10BIT;  //
        param->u.bt656.clock_min = OV5640_XCLK_MIN;
        param->u.bt656.clock_max = OV5640_XCLK_MAX;
        param->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */
    }

	return 0;
}

static int
ovcamera_ioctl_s_power(struct v4l2_int_device *idev, int on)
{
	struct sensor_data *sensor = idev->priv;
	struct fsl_mxc_camera_platform_data *platform = sensor->i2c_client->dev.platform_data;

    pr_info("%s \n", __func__ );


    if (on && !sensor->on) {
        pr_info("%s setting pwn_gpio to %d (on)\n", __func__ , 0);
        /* Make sure power on */
        gpio_set_value(pwn_gpio, 0 );

        msleep(2);
    } else if (!on && sensor->on) {
     pr_info("%s setting pwn_gpio to %d (off) \n", __func__ , 1 );
        gpio_set_value(pwn_gpio, 1 );

        msleep(2);
    }
    sensor->on = on;


	return 0;
}

static int
ovcamera_ioctl_g_param(struct v4l2_int_device *idev, struct v4l2_streamparm *param)
{
	struct sensor_data *sensor = idev->priv;
	struct v4l2_captureparm *cparam = &param->parm.capture;

    pr_info("%s \n", __func__ );

	switch (param->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        pr_info("%s: streamcap 0x%x\n", __func__ , sensor->streamcap.capability);
		memset(param, 0, sizeof(*param));
		param->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparam->capability = sensor->streamcap.capability;
		cparam->timeperframe = sensor->streamcap.timeperframe;
		cparam->capturemode = sensor->streamcap.capturemode;
		return 0;
    default:
        pr_info("%s: capture operation %d unsupported.\n", __func__, param->type);
        break;
    }

    return -EINVAL;
}

static int
ovcamera_ioctl_s_param(struct v4l2_int_device *idev, struct v4l2_streamparm *param)
{
	struct sensor_data *sensor = idev->priv;

    pr_info("%s \n", __func__ );

	switch (param->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		sensor->streamcap.capturemode = param->parm.capture.capturemode;
		sensor->streamcap.timeperframe = sensor->streamcap.timeperframe;
		//sensor->streamcap.timeperframe = param->parm.capture.timeperframe;

        pr_info("%s: capturemode %d timeperframe %d/%d\n", __func__,
                 sensor->streamcap.capturemode,
                 sensor->streamcap.timeperframe.numerator,
                 sensor->streamcap.timeperframe.denominator);
		return 0;
    default:
        pr_info("%s: invalid stream type: %d\n", __func__, param->type);
        break;
    }

    return -EINVAL;
}

static int
ovcamera_ioctl_enum_fmt_cap(struct v4l2_int_device *idev, struct v4l2_fmtdesc *format)
{
	struct sensor_data *sensor = idev->priv;

    pr_info("%s \n", __func__ );

	if (format->index > 0) return -EINVAL;
	format->pixelformat = sensor->pix.pixelformat;
	return 0;
}

static int
ovcamera_ioctl_g_fmt_cap(struct v4l2_int_device *idev, struct v4l2_format *format)
{
	struct sensor_data *sensor = idev->priv;

    // as per the 5640 driver

    pr_info("%s \n", __func__ );

    format->fmt.pix = sensor->pix;
    format->fmt.pix.pixelformat = sensor->pix.pixelformat;

    return 0;
}

static int
ovcamera_ioctl_queryctrl(struct v4l2_int_device *idev, struct v4l2_queryctrl *query)
{
    int index;

    pr_info("%s \n", __func__ );

    for (index = 0; index < ARRAY_SIZE(ovcamera_qctrl); index++) {
		pr_debug("%s query id %d\n", __func__, query->id);
        if (query->id && query->id == ovcamera_qctrl[index].id) {
            memcpy(query, &ovcamera_qctrl[index], sizeof(*query));
            return 0;
        }
    }

	return -EINVAL;
}

static int
ovcamera_ioctl_g_ctrl(struct v4l2_int_device *idev, struct v4l2_control *control)
{
	struct sensor_data *sensor = idev->priv;
    uint8_t val;

    pr_info("%s \n", __func__ );

	switch (control->id) {
	case V4L2_CID_BRIGHTNESS:
       //  ovcamera_read_reg(sensor->i2c_client, REG_BRIGHTNESS, &val);
        control->value = val;
        return 0;
    case V4L2_CID_CONTRAST:
       //  ovcamera_read_reg(sensor->i2c_client, REG_CONTRAST, &val);
        control->value = val;
        return 0;
    case V4L2_CID_SATURATION:
        // ovcamera_read_reg(sensor->i2c_client, REG_SATURATION, &val);
        control->value = val;
        return 0;
    case V4L2_CID_HUE:
        // ovcamera_read_reg(sensor->i2c_client, REG_HUE, &val);
        control->value = val;
        return 0;
	}

    return -EINVAL;
}

static int
ovcamera_ioctl_s_ctrl(struct v4l2_int_device *idev, struct v4l2_control *control)
{
	struct sensor_data *sensor = idev->priv;

    pr_info("%s \n", __func__ );

	switch (control->id) {
	case V4L2_CID_BRIGHTNESS:
       //  ovcamera_write_reg(sensor->i2c_client, REG_BRIGHTNESS, control->value);
		return 0;
	case V4L2_CID_CONTRAST:
        // ovcamera_write_reg(sensor->i2c_client, REG_CONTRAST, control->value);
		return 0;
	case V4L2_CID_SATURATION:
        // ovcamera_write_reg(sensor->i2c_client, REG_SATURATION, control->value);
		return 0;
	case V4L2_CID_HUE:
        // ovcamera_write_reg(sensor->i2c_client, REG_HUE, control->value);
		return 0;
	}

    return -EPERM;
}

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
                     struct v4l2_frmivalenum *fival)
{
    int i, j, count;

    pr_info("%s \n",__func__ );

/*  FIX just return an error for now
 *
    if (fival->index < 0 || fival->index > ov5640_mode_MAX)
        return -EINVAL;

    if (fival->width == 0 || fival->height == 0 ||
        fival->pixel_format == 0) {
        pr_warning("Please assign pixelformat, width and height.\n");
        return -EINVAL;
    }

    fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
    fival->discrete.numerator = 1;

    count = 0;
    for (i = 0; i < ARRAY_SIZE(ov5640_mode_info_data); i++) {
        for (j = 0; j < (ov5640_mode_MAX + 1); j++) {
            if (fival->pixel_format == ov5640_data.pix.pixelformat
             && fival->width == ov5640_mode_info_data[i][j].width
             && fival->height == ov5640_mode_info_data[i][j].height
             && ov5640_mode_info_data[i][j].init_data_ptr != NULL) {
                count++;
            }
            if (fival->index == (count - 1)) {
                fival->discrete.denominator =
                        ov5640_framerates[i];
                return 0;
            }
        }
    }
*/
    return -EINVAL;
}

static int
ovcamera_ioctl_enum_framesizes(struct v4l2_int_device *idev, struct v4l2_frmsizeenum *fsize)
{
	struct sensor_data *sensor = idev->priv;

    pr_info("%s \n", __func__ );
//HACK need to fix for different format sizes ala 5640 ...

	fsize->pixel_format = sensor->pix.pixelformat;
    fsize->discrete.width = 1280;
    fsize->discrete.height = 720;
	
	return 0;
}

static int
ovcamera_ioctl_g_chip_ident(struct v4l2_int_device *idev, int *id)
{
    struct v4l2_dbg_chip_ident *chip_id = (struct v4l2_dbg_chip_ident*) id;

    pr_info("%s \n", __func__ );

    pr_debug("identify chip.\n");
    chip_id->match.type = V4L2_CHIP_MATCH_I2C_DRIVER;
    // ?? not set in 5460 driver ... should likely be
	// chip_id->ident = V4L2_IDENT_TVP5147;
	strcpy(chip_id->match.name, "ov10633_camera");

    return 0;
}

static int
ovcamera_ioctl_init(struct v4l2_int_device *idev)
{
    pr_debug("%s\n", __func__);
	return 0;
}

static int
ovcamera_ioctl_dev_init(struct v4l2_int_device *idev)
{
	struct sensor_data *sensor = idev->priv;
    struct i2c_client *i2c_client = sensor->i2c_client;

    pr_info("%s \n", __func__ );
    ovcamera_write_reg(i2c_client, 0x0103, 0x01); // soft reset (OVCAM)
    msleep(2);

    ovcamera_write_reg(i2c_client,0x301D, 0xFF);
    ovcamera_write_reg(i2c_client,0x301E, 0xFF);
    ovcamera_write_reg(i2c_client,0x3040, 0xFF);
    ovcamera_write_reg(i2c_client,0x3041, 0xFF);
    ovcamera_write_reg(i2c_client,0x3042, 0xFF);

    ovcamera_write_reg(i2c_client,0x301B, 0xFF);
    ovcamera_write_reg(i2c_client,0x301C, 0xFF);
    ovcamera_write_reg(i2c_client, 0x301A, 0xFF);

    msleep(2);

    // bump up to 2X output drive (save bottom 5 bits as they are reserved)
    ovcamera_write_reg(i2c_client, 0x3011, 0x02 );
    //*** Switch from PAD Clock to all clock ?
    ovcamera_write_reg(i2c_client, 0x3023, 0x10 );

    ovcamera_write_reg(i2c_client, 0x3024, 0x05 );  // PLCLK = System Clk and Short
    // set system clock to 82.66 MHz to match our default settings for now
    ovcamera_write_reg(i2c_client,0x3003, 0x1f);
    ovcamera_write_reg(i2c_client,0x3004, 0x12);

    ovcamera_write_reg(i2c_client,0x3800, 0x00); // Timing X Start Addr (cropping x start)
    ovcamera_write_reg(i2c_client,0x3801, 0x00);
    ovcamera_write_reg(i2c_client,0x3802, 0x00); // (OVCAM) programatically
    ovcamera_write_reg(i2c_client,0x3803, 0x2E); // programatically


    ovcamera_write_reg(i2c_client,0x3804, 0x05);  //  Timing X End Addr  def 0x05
    ovcamera_write_reg(i2c_client,0x3805, 0x1f);  //

    ovcamera_write_reg(i2c_client,0x3806, 0x03);  // (OVCAM) programatically
    ovcamera_write_reg(i2c_client,0x3807, 0x01);  // programatically

    ovcamera_write_reg(i2c_client,0x3808, 0x05);  // (OVCAM) programatically
    ovcamera_write_reg(i2c_client,0x3809, 0x00);  // (OVCAM+1) programatically
    ovcamera_write_reg(i2c_client,0x380a, 0x02);  // (OVCAM) programatically
    ovcamera_write_reg(i2c_client,0x380b, 0xd0);  // (OVCAM+1) programatically
    ovcamera_write_reg(i2c_client,0x380C, 0x06); // (OVCAM) programatically
    ovcamera_write_reg(i2c_client,0x380D, 0xFB); // (OVCAM+1) programatically
    ovcamera_write_reg(i2c_client,0x380E, 0x03);  // (OVCAM) programatically
    ovcamera_write_reg(i2c_client,0x380F, 0x03);  // (OVCAM+1) pro
    ovcamera_write_reg(i2c_client,0x3621, 0x63); // (OVCAM) programatically
    ovcamera_write_reg(i2c_client,0x3702, 0x1C); // (OVCAM) programatically
    ovcamera_write_reg(i2c_client,0x3703, 0x3E); // (OVCAM) programatically
    ovcamera_write_reg(i2c_client,0x3704, 0x2B); // (OVCAM) programatically
    ovcamera_write_reg(i2c_client, 0x4606, 0x0D);  // programatically
    ovcamera_write_reg(i2c_client,0x4607, 0xF6);  // programatically
    ovcamera_write_reg(i2c_client,0x460a, 0x03);  // programatically
    ovcamera_write_reg(i2c_client, 0x460b, 0xF6);  // programatically
    ovcamera_write_reg(i2c_client, 0xc488, 0x2F); // programatically
    ovcamera_write_reg(i2c_client,0xc489, 0xB0);  // programatically
    ovcamera_write_reg(i2c_client, 0xc48a, 0x2F);  // programatically
    ovcamera_write_reg(i2c_client, 0xc48b, 0xB0);  // programatically
    ovcamera_write_reg(i2c_client, 0xc4cc, 0x0e);  // programatically
    ovcamera_write_reg(i2c_client, 0xc4cd, 0x7e);  // programatically
    ovcamera_write_reg(i2c_client, 0xc4ce, 0x0e);  // programatically
    ovcamera_write_reg(i2c_client, 0xc4cf, 0x7e);  // programatically
    ovcamera_write_reg(i2c_client,0xc512, 0xe7); // programatically
    ovcamera_write_reg(i2c_client,0xc513, 0xe8); // programatically

    // Horizontal sub sample
    ovcamera_write_reg(i2c_client,0xc518, 0x03);  // programatically
    ovcamera_write_reg(i2c_client, 0xc519, 0x03);  // programatically
    ovcamera_write_reg(i2c_client,0xc51a, 0x06);   // programatically
    ovcamera_write_reg(i2c_client, 0xc51b, 0xfb);  // programatically

    ovcamera_write_reg(i2c_client,0X4300, 0x38);  // (OVCAM) Set the format to YUV (3) and YUYV (8

    ovcamera_write_reg(i2c_client,0X5003, 0x14);  // (OVCAM)  ? Set for YUV44 to YUV422 drop AND no-VSYNC latch AND AEC/simple awb/tonemap/combine done

    ovcamera_write_reg(i2c_client,0x4605, 0x08);


    ovcamera_write_reg( i2c_client,0x0100, 0x01 ); // ensure the video streaming bit is set

    /* delay at least 1 ms */
    msleep(1);
    ovcamera_write_reg(i2c_client,0x3042, 0xF9);
    msleep(30);
    ovcamera_write_reg(i2c_client,0x301D, 0xB4);
    ovcamera_write_reg(i2c_client,0x301E, 0xF0);
    ovcamera_write_reg(i2c_client,0x3040, 0xF0);
    ovcamera_write_reg(i2c_client,0x3041, 0xF0);
    ovcamera_write_reg(i2c_client,0x301B, 0xF0);
    ovcamera_write_reg(i2c_client,0x301C, 0xF0);
    ovcamera_write_reg(i2c_client,0x301A, 0xF0);
    
    /* this controls the UV sign bit somehow?? */
    //ovcamera_write_reg(i2c_client,0x6901, 0x01);

    msleep(30);

	return 0;
}

static int
ovcamera_ioctl_dev_exit(struct v4l2_int_device *idev)
{
    pr_debug("%s\n", __func__);
	return 0;
}

static struct v4l2_int_ioctl_desc ovcamera_ioctl_desc[] = {
	{ vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_g_ifparam },
	{ vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_s_power },
	{ vidioc_int_g_parm_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_g_param },
	{ vidioc_int_s_parm_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_s_param },
	{ vidioc_int_enum_fmt_cap_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_enum_fmt_cap },
	{ vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_g_fmt_cap },
	// was in the TVP514X driver from Sebatien
	{ vidioc_int_queryctrl_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_queryctrl },
	// was in 5640 driver
	{ vidioc_int_enum_frameintervals_num, (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
	{ vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_g_ctrl },
	{ vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_s_ctrl },
	{ vidioc_int_enum_framesizes_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_enum_framesizes },
    { vidioc_int_g_chip_ident_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_g_chip_ident },
	{ vidioc_int_init_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_init },
	{ vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ovcamera_ioctl_dev_init },
	{ vidioc_int_dev_exit_num, ovcamera_ioctl_dev_exit },
};

static inline void ov10633_reset(void)
{

    pr_info("%s \n", __func__ );

	// initialization sequence unique to 10633 to allow for reading of i2c bus
	// get this to High
	gpio_set_value(pwn_gpio, 1);
	msleep(10);
	// pull the reset high
	gpio_set_value(rst_gpio, 1);
	msleep(10);
	// pull the reset low
	gpio_set_value(rst_gpio, 0);
	msleep(60);
	// set the PWN low
	gpio_set_value(pwn_gpio, 0);
	// 200 microseconds ... should be long enough to do 1 millisecond then
	msleep(20);  // perhaps the documentation is incorrect and they meant 200 ms

	// pull the reset high
	gpio_set_value(rst_gpio, 1);
	// this might be more ... need to calculate
	// So assuming a 400kHz ic2 clock then this calcuates out to about 5 milliseconds delay before we read ... 6 for safety
	msleep(120);
}

static int
ovcamera_probe(struct i2c_client *i2c_client, const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
    struct device *dev = &i2c_client->dev;
    int retval;
    struct regmap *gpr;

	uint8_t chip_id_hi, chip_id_lo;
    char sysfs_link_name[16] = { 0 };
	struct fsl_mxc_camera_platform_data *platform = i2c_client->dev.platform_data;
    struct kobject *sys_devices;
	struct sensor_data *sensor;
    struct v4l2_int_device *device;
    struct v4l2_int_slave *slave;


    pr_info("%s \n", __func__ );

	if (strcmp(i2c_client->name, "ov10633") == 0) {

		ov_selected_camera = ov_camera_10633;
        pr_info("%s : i2c_client name 10633 \n",__func__ );
	}
	else if (strcmp(i2c_client->name, "ov5640") == 0) {
        pr_info("%s: i2c_client name ov5640 \n", __func__);
	}
	else {
		pr_err("%s: Error Unknown client type %s \n", __func__, i2c_client->name);
		return -ENODEV;
	}

	/* ov10633 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "%s: setup pinctrl failed\n", __func__);
		return PTR_ERR(pinctrl);
	}

	/* request power down pin */
	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwn_gpio)) {
		dev_err(dev, "%s: no sensor pwdn pin available\n", __func__);
		return -ENODEV;
	}
	retval = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH, "ov5640_pwdn");
	if (retval < 0)	return retval;

	/* request reset pin */
	rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(rst_gpio)) {
		dev_err(dev, "%s: no sensor reset pin available\n", __func__);
		return -EINVAL;
	}
	retval = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_HIGH,	"ov5640_reset");
	if (retval < 0)	return retval;

	// Memory allocation
	// sensor info
    sensor = kmalloc(sizeof(struct sensor_data), GFP_KERNEL);
    if (sensor == NULL) return -ENOMEM;
	memset(sensor, 0, sizeof(struct sensor_data));

	// device info
    device = kmalloc(sizeof(struct v4l2_int_device), GFP_KERNEL);
    if (device == NULL) return -ENOMEM;
    memset(device, 0, sizeof(struct v4l2_int_device));
	// slave info
    slave = kmalloc(sizeof(struct v4l2_int_slave), GFP_KERNEL);
    if (slave == NULL) return -ENOMEM;
    memset(slave, 0, sizeof(struct v4l2_int_slave));

#if 0
    sensor->mclk = platform->mclk;
	sensor->mclk_source = platform->mclk_source;
	sensor->csi = platform->csi;
#else
    sensor->sensor_clk= devm_clk_get(dev, "csi_mclk");
    if (IS_ERR(sensor->sensor_clk)) {
        dev_err(dev, "%s: get mclk failed\n",__func__);
        return PTR_ERR(sensor->sensor_clk);
    }

    retval = of_property_read_u32(dev->of_node, "mclk", &sensor->mclk);
    if (retval) {
        dev_err(dev, "%s : mclk frequency is invalid\n", __func__);
        return retval;
    }

    retval = of_property_read_u32(dev->of_node, "mclk_source", (u32 *) &(sensor->mclk_source));
    if (retval) {
        dev_err(dev, "%s : mclk_source invalid\n", __func__);
        return retval;
    }

    retval = of_property_read_u32(dev->of_node, "ipu_id",  &sensor->ipu_id);
    if (retval) {
        dev_err(dev, "%s : ipu_id missing or invalid\n", __func__ );
        return retval;
    }

    retval = of_property_read_u32(dev->of_node, "csi_id",  &(sensor->csi));
    if (retval) {
        dev_err(dev, "%s : csi_id invalid\n", __func__ );
        return retval;
    }

    clk_prepare_enable(sensor->sensor_clk);
#endif

    slave->ioctls = ovcamera_ioctl_desc;
    slave->num_ioctls = ARRAY_SIZE(ovcamera_ioctl_desc);

    device->module = THIS_MODULE;
    strcpy(device->name, "ovcamera");
    // not in 5640 driver ?
    device->type = v4l2_int_type_slave;
    device->u.slave = slave;

    device->priv = sensor;
    sensor->i2c_client = i2c_client;

    sensor->on = true;


	// ??  different than in 5640 ... where is this io_init defined?
	// NOTE for time being am setting this dependent on the chip we think we are talking to
	// 	sensor->io_init = platform->io_init;
	if (ov_camera_10633 == ov_selected_camera) {
		sensor->io_init = ov10633_reset;
	}
	else if (ov_camera_5640 == ov_selected_camera) {
	}

	// changed to work for 10633 for now
	sensor->pix.pixelformat = V4L2_PIX_FMT_YUYV;
	sensor->pix.width = 1280;
	sensor->pix.height = 720;
	// not defined in 5640
	// ... this would be incorrect anyway sensor->pix.field = V4L2_FIELD_INTERLACED_BT;
	// not in 5640 driver
	// sensor->pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
	// not in 5640 driver
	// sensor->pix.priv = 1; /* TV in flag */

	// changed to 720p capture mode
	sensor->streamcap.capturemode = 4;
	sensor->streamcap.capability = V4L2_MODE_HIGHQUALITY | V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.timeperframe.denominator = 30;
	sensor->streamcap.timeperframe.numerator = 1;

    if (sysfs_create_group(&i2c_client->dev.kobj, &base_attribute_group)) {
        return -ENOMEM;
    }

    if (sysfs_create_group(&i2c_client->dev.kobj, &sync_attribute_group)) {
        return -ENOMEM;
    }

    if (sysfs_create_group(&i2c_client->dev.kobj, &reg_attribute_group)) {
        return -ENOMEM;
    }

    /* Ugly hack to get to /sys/devices why is this not a base symbol??? */
    sys_devices = i2c_client->dev.kobj.parent->parent->parent->parent;
    snprintf(sysfs_link_name, sizeof(sysfs_link_name), "ovcamera.%s", i2c_client->dev.kobj.name);
    if (sysfs_create_link(sys_devices, &i2c_client->dev.kobj, sysfs_link_name)) {
		pr_warning("%s: failed to create link /sys/devices/%s\n", __func__, sysfs_link_name);
    }
	
	if (ov_camera_10633 == ov_selected_camera) {
        pr_info("%s: OK should be ready to reset it \n", __func__);
         ov10633_reset();
         /*
		// initialization sequence unique to 10633 to allow for reading of i2c bus
		// get this to High
		gpio_set_value(pwn_gpio, 1);
		msleep(10);
		// pull the reset high
		gpio_set_value(rst_gpio, 1);
		msleep(10);
		// pull the reset low
		gpio_set_value(rst_gpio, 0);
		msleep(60);
		// set the PWN low
		gpio_set_value(pwn_gpio, 0);
		// 200 microseconds ... should be long enough to do 1 millisecond then
		msleep(20);  // perhaps the documentation is incorrect and they meant 200 ms

		// pull the reset high
		gpio_set_value(rst_gpio, 1);
		// this might be more ... need to calculate
		// So assuming a 400kHz ic2 clock then this calcuates out to about 5 milliseconds delay before we read ... 6 for safety
		msleep(120);
        */
	}
	else if (ov_camera_5640 == ov_selected_camera) {
		// initialization sequence unique to 5640 ... 
	}
	// both cameras use the same register for the chip id
	ovcamera_read_reg(i2c_client, OV5640_CHIP_ID_HIGH_BYTE, &chip_id_hi);
	ovcamera_read_reg(i2c_client, OV5640_CHIP_ID_LOW_BYTE, &chip_id_lo);


	pr_info("%s: %X chip id 0x%.2x%.2x\n", __func__, i2c_client->addr, chip_id_hi, chip_id_lo);

	if ((chip_id_hi) == 0xA6 && (chip_id_lo == 0x30)) {
		pr_info("%s: camera ov10633 IS detected\n", __func__);
		// *** was in 5640 ... seems to be associating with some imx register sets?
		// *** ??
		gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
		if (!IS_ERR(gpr)) {
			if (of_machine_is_compatible("fsl,imx6q")) {
				int mask = sensor->csi ? (1 << 20) : (1 << 19);

				if (sensor->csi != sensor->ipu_id) {
					pr_warning("%s: csi_id != ipu_id\n", __func__);
					return -ENODEV;
				}
				regmap_update_bits(gpr, IOMUXC_GPR1, mask, mask);
			}
			else if (of_machine_is_compatible("fsl,imx6dl")) {
				int mask = sensor->csi ? (7 << 3) : (7 << 0);
				int val = sensor->csi ? (4 << 3) : (4 << 0);

				regmap_update_bits(gpr, IOMUXC_GPR13, mask, val);
			}
		}
		else {
			pr_err("%s: failed to find fsl,imx6q-iomux-gpr regmap\n", __func__);
		}

		return v4l2_int_device_register(device);
	}

	return -ENODEV;
}

static int
ovcamera_remove(struct i2c_client *client)
{
    pr_debug("%s\n", __func__);
	return 0;
}

static const struct i2c_device_id ovcamera_id[] = {
    { "ov10633", 0 },
    // { "ov5640", 0 },  // add later?
	{},
};

MODULE_DEVICE_TABLE(i2c, ovcamera_id);

static struct i2c_driver ovcamera_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ovcamera",
	},
	.probe = ovcamera_probe,
	.remove = ovcamera_remove,
	.id_table = ovcamera_id,
};

static __init int ovcamera_init(void)
{

    pr_info("%s \n", __func__ );

	return i2c_add_driver(&ovcamera_i2c_driver);
}

static void __exit ovcamera_exit(void)
{

    pr_info("%s \n", __func__ );

	i2c_del_driver(&ovcamera_i2c_driver);
}

module_init(ovcamera_init);
module_exit(ovcamera_exit);

MODULE_AUTHOR("Sébastien Taylor <sebastien@au-zone.com>");
MODULE_DESCRIPTION("Omni Vision Camera driver (5640 and 10633)");
MODULE_LICENSE("GPL");

