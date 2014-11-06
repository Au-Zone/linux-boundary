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

// 5640 and 10633 both use the same registers and clock specifications
#define OV5640_CHIP_ID_HIGH_BYTE        0x300A
#define OV5640_CHIP_ID_LOW_BYTE         0x300B
#define OV5640_XCLK_MIN 6000000
#define OV5640_XCLK_MAX 24000000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

// specific to 10633
#define OV1063X_SENSOR_WIDTH		1312
#define OV1063X_SENSOR_HEIGHT		814

#define OV1063X_MAX_WIDTH		1280
#define OV1063X_MAX_HEIGHT		800

//  max / min for 1st mid PLL clock speeds
#define OV10633_MIDPLLCLK_MAX   27000000
#define OV10633_MIDPLLCLK_MIN   3000000
// max / min for the 2nd mid PLL clock speeds
#define OV10633_MIDPLLCLK_2_MAX 500000000
#define OV10633_MIDPLLCLK_2_MIN 200000000
// max system clock speed allowed
#define OV10633_SYSCLK_MAX      96000000


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

//  HACK
// NOTE: this structure an the subsequent data structure that it is used with should be changed
//   in the 5640 there were 2 tables one for 15FPS and the other for 30 FPS
//   in the 10633 we are calculating so we should just calculate for both of these chips eventually
// then we have a single table with the MODES and the FPS and clock speed (pixel clock) are calculated out
//  or some such thing
enum ov5640_frame_rate {
    ov5640_15_fps,
    ov5640_30_fps
};

static int ov5640_framerates[] = {
    [ov5640_15_fps] = 15,
    [ov5640_30_fps] = 30,
};


struct ov5640_mode_info {
    enum ov5640_mode mode;
    u32 width;
    u32 height;
    struct reg_value *init_data_ptr;
    u32 init_data_size;
    int				xvclk;
    int				fps_numerator;
    int				fps_denominator;
};

static struct ov5640_mode_info ov5640_mode_info_data[2][ov5640_mode_MAX + 1] = {
    {
        {ov5640_mode_VGA_640_480,      640,  480,
        NULL, // ov5640_setting_15fps_VGA_640_480,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_15fps_VGA_640_480)},
        {ov5640_mode_QVGA_320_240,     320,  240,
        NULL, // ov5640_setting_15fps_QVGA_320_240,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_15fps_QVGA_320_240)},
        {ov5640_mode_NTSC_720_480,     720,  480,
        NULL, // ov5640_setting_15fps_NTSC_720_480,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_15fps_NTSC_720_480)},
        {ov5640_mode_PAL_720_576,      720,  576,
        NULL, // ov5640_setting_15fps_PAL_720_576,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_15fps_PAL_720_576)},
        {ov5640_mode_720P_1280_720,   1280,  720,
        NULL, // ov5640_setting_15fps_720P_1280_720,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_15fps_720P_1280_720)},
        {ov5640_mode_1080P_1920_1080, 1920, 1080,
        NULL, // ov5640_setting_15fps_1080P_1920_1080,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_15fps_1080P_1920_1080)},
        {ov5640_mode_QSXGA_2592_1944, 2592, 1944,
        NULL, // ov5640_setting_15fps_QSXGA_2592_1944,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_15fps_QSXGA_2592_1944)},
        {ov5640_mode_QCIF_176_144,     176,  144,
        NULL, // ov5640_setting_15fps_QCIF_176_144,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_15fps_QCIF_176_144)},
        {ov5640_mode_XGA_1024_768,    1024,  768,
        NULL, // ov5640_setting_15fps_XGA_1024_768,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_15fps_XGA_1024_768)},
    },
    {
        {ov5640_mode_VGA_640_480,      640,  480,
        NULL, // ov5640_setting_30fps_VGA_640_480,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_30fps_VGA_640_480)},
        {ov5640_mode_QVGA_320_240,     320,  240,
        NULL, // ov5640_setting_30fps_QVGA_320_240,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_30fps_QVGA_320_240)},
        {ov5640_mode_NTSC_720_480,     720,  480,
        NULL, // ov5640_setting_30fps_NTSC_720_480,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_30fps_NTSC_720_480)},
        {ov5640_mode_PAL_720_576,      720,  576,
        NULL, // ov5640_setting_30fps_PAL_720_576,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_30fps_PAL_720_576)},
        {ov5640_mode_720P_1280_720,   1280,  720,
        NULL, // ov5640_setting_30fps_720P_1280_720,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_30fps_720P_1280_720)},
        {ov5640_mode_1080P_1920_1080, 0, 0, NULL, 0, 0, 0, 0},
        {ov5640_mode_QSXGA_2592_1944, 0, 0, NULL, 0, 0, 0, 0},
        {ov5640_mode_QCIF_176_144,     176,  144,
        NULL, // ov5640_setting_30fps_QCIF_176_144,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_30fps_QCIF_176_144)},
        {ov5640_mode_XGA_1024_768,    1024,  768,
        NULL, // ov5640_setting_30fps_XGA_1024_768,
        0, 0, 0, 0}, // ARRAY_SIZE(ov5640_setting_30fps_XGA_1024_768)},
    },
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

static s32 ovcamera_write_reg16(struct i2c_client*, u16 reg, u16 val);


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


// ** 10633 Specific functions

// used to calculate the pixel clock
// NOTE: This is only for the YUV422 Pixel Clock calculation at the moment
//
static int ov10633_get_pclk(int xvclk, int *htsmin, int *vtsmin, int fps_numerator, int fps_denominator, u8 *r3003, u8 *r3004)
{
        int pre_divs[] = { 2, 3, 4, 6, 8, 10, 12, 14 };
        int pclk;
        int best_pclk = INT_MAX;
        int best_hts = 0;
        int i, j, k;
        int best_i = 0, best_j = 0, best_k = 0;
        int clk1, clk2;
        int hts;

        pr_info("%s: xvclk %d fps_numerator %d fps_denominator %d \n", __func__, xvclk, fps_numerator, fps_denominator );
        /* Pre-div, reg 0x3004, bits 6:4 */
        for (i = 0; i < 8; i++) {
            clk1 = (xvclk / pre_divs[i]) * 2;

            if ((clk1 < OV10633_MIDPLLCLK_MIN) || (clk1 > OV10633_MIDPLLCLK_MAX))
                continue;

            /* Mult = reg 0x3003, bits 5:0 */

            for (j = 1; j < 32; j++) {
                clk2 = (clk1 * j);

                if ((clk2 < OV10633_MIDPLLCLK_2_MIN) || (clk2 > OV10633_MIDPLLCLK_2_MAX))
                    continue;

                /* Post-div, reg 0x3004, bits 2:0 */
                for (k = 0; k < 8; k++) {
                    pclk = clk2 / (2 * (k + 1));
                    if (pclk > OV10633_SYSCLK_MAX)
                        continue;
                    hts = *htsmin + 200 + pclk / 300000;  // ? need to figure this out ?

                    /* 2 clock cycles for every YUV422 pixel */

                    if (pclk < (((hts * *vtsmin) / fps_denominator)
                        * fps_numerator * 2))
                        continue;

                    if (pclk < best_pclk) {
                        best_pclk = pclk;
                        best_hts = hts;
                        best_i = i;
                        best_j = j;
                        best_k = k;
                    }
                }
            }
        }

        /* register contents  for clock */
        * r3003 = (u8)best_j;
        * r3004 = ((u8)best_i << 4) | (u8)best_k;

        /* Did we get a valid PCLK? */
        if (best_pclk == INT_MAX)
            return -1;

        *htsmin = best_hts;

        /* Adjust vts to get as close to the desired frame rate as we can */
        *vtsmin = best_pclk / ((best_hts / fps_denominator) * fps_numerator * 2);

        return best_pclk;

}

/* Setup registers according to resolution and color encoding */
static int ov10633_change_mode( struct i2c_client* i2c_client, enum ov5640_frame_rate frame_rate,  enum ov5640_mode mode)
{
    struct ov5640_mode_info *priv;
    int ret;
    ret = -EINVAL;
    int pclk;
    int hts, vts;
    u8 r3003, r3004;
    int tmp;
    u32 height_pre_subsample;
    u32 width_pre_subsample;
    u8 horiz_crop_mode;
    int i;
    int nr_isp_pixels;
    int vert_sub_sample = 0;
    int horiz_sub_sample = 0;
    int sensor_width;

    // init priv to something here

    //    if ((*width > OV1063X_MAX_WIDTH) || (*height > OV1063X_MAX_HEIGHT))	{
    //            return ret;
    //    }
    pr_info("%s: frame_rate %d mode %d \n",__func__, frame_rate, mode );

    /* select format */
    priv =  &ov5640_mode_info_data[frame_rate][mode];
   // priv->cfmt = NULL;
    priv->xvclk = OV5640_XCLK_MAX;
    priv->fps_numerator = 1;
    priv->fps_denominator = ov5640_framerates[frame_rate];

   //  priv->width = *width;
   //  priv->height = *height;

    /* Vertical sub-sampling? */
    height_pre_subsample = priv->height;

    if (priv->height <= 400) {
        vert_sub_sample = 1;
        height_pre_subsample <<= 1;
    }

    /* Horizontal sub-sampling? */
    width_pre_subsample = priv->width;

    if (priv->width <= 640) {
        horiz_sub_sample = 1;
        width_pre_subsample <<= 1;
    }

    /* Horizontal cropping */
    if (width_pre_subsample > 768) {
        sensor_width = OV1063X_SENSOR_WIDTH;
        horiz_crop_mode = 0x63;
    }
    else if (width_pre_subsample > 656) {
        sensor_width = 768;
        horiz_crop_mode = 0x6b;
    }
    else {
        sensor_width = 656;
        horiz_crop_mode = 0x73;
    }

    /* minimum values for hts and vts */
    hts = sensor_width;
    vts = height_pre_subsample + 50;

    pr_info("%s: fps=(%d/%d), hts=%d, vts=%d\n", __func__,	priv->fps_numerator, priv->fps_denominator, hts, vts);

    /* Get the best PCLK & adjust hts,vts accordingly */

    // swapped denom and num as the function want 30/1 versus 1/30 (which didn't work)
    pclk = ov10633_get_pclk(priv->xvclk, &hts, &vts, priv->fps_denominator, priv->fps_numerator, &r3003, &r3004);

    if (pclk < 0) {
        return ret;
    }

    pr_info( "%s: pclk=%d, hts=%d, vts=%d\n", __func__, pclk, hts, vts);
    pr_info( "%s: r3003=0x%X r3004=0x%X\n", __func__, r3003, r3004);

    /* Set to 1280x720 */
    // this should get set later ... may want to remove
    ret = ovcamera_write_reg( i2c_client,0x380f, 0x80);
    if (ret) {
        return ret;
    }

    /* Set PLL */

    ret = ovcamera_write_reg(i2c_client,0x3003, r3003);

    if (ret) {
        return ret;
    }
    ret = ovcamera_write_reg( i2c_client,0x3004, r3004);
    if (ret) {
        return ret;
    }
    pr_info("%s:     register 0x3003 = %02X \n", __func__, r3003);
    pr_info("%s     register 0x3004 = %02X \n", __func__, r3004);

    /* Set HSYNC */
    ret = ovcamera_write_reg(i2c_client, 0x4700, 0x00);
    if (ret) {
        return ret;
    }
    pr_info("%s:  SETTING HSYNC \n", __func__);
    pr_info("%s:     register 0x4700 = %02X \n", __func__, 0x00);

    /* Set format to UYVY */

    ret = ovcamera_write_reg(i2c_client, 0x4300, 0x38); // Set the format to YUV (3) and YUYV (8
    ret = ovcamera_write_reg(i2c_client, 0x5003, 0x14); //Set for YUV44 to YUV422 drop AND no-VSYNC latch AND AEC/simple awb/tonemap/combine done
    if (ret) {
        return ret;
    }
    pr_info("%s:  SETTING YUYV \n", __func__);
    pr_info("%s:     register 0x4300 = %02X and 0x5003 = %2X \n", __func__, 0x38, 0x14);

    /* Set output to 8-bit yuv */

    ret = ovcamera_write_reg(i2c_client, 0x4605, 0x08);
    if (ret) {
        return ret;
    }
    pr_info("%s:  SETTING 8-bit yuv \n", __func__);
    pr_info("%s:      register 0x4605 = %02X \n", __func__, 0x08);

    /* Horizontal cropping */

    ret = ovcamera_write_reg(i2c_client,0x3621, horiz_crop_mode);
    if (ret) {
        return ret;
    }
    ret = ovcamera_write_reg( i2c_client,0x3702, (pclk + 1500000) / 3000000);
    if (ret) {
        return ret;
    }
    ret = ovcamera_write_reg(i2c_client, 0x3703, (pclk + 666666) / 1333333);
    if (ret) {
        return ret;
    }
    ret = ovcamera_write_reg(i2c_client, 0x3704, (pclk + 961500) / 1923000);
    if (ret) {
        return ret;
    }

    pr_info("%s:  Horizontal cropping\n", __func__);
    pr_info("%s:    register 0x3621 = %02X \n", __func__, horiz_crop_mode);
    pr_info("%s:    register 0x3702 = %02X \n", __func__, (pclk + 1500000) / 3000000);
    pr_info("%s:    register 0x3703 = %02X \n", __func__, (pclk + 666666) / 1333333);
    pr_info("%s:    register 0x3704 = %02X \n", __func__, (pclk + 961500) / 1923000);

    /* Vertical cropping */
    tmp = ((OV1063X_SENSOR_HEIGHT - height_pre_subsample) / 2) & ~0x1;
    ret = ovcamera_write_reg( i2c_client,0x3802, tmp);
    if (ret) {
        return ret;
    }
    tmp = tmp + height_pre_subsample + 3;
    ret = ovcamera_write_reg( i2c_client,0x3806, tmp);
    if (ret) {
        return ret;
    }
    pr_info("%s:  Vertical cropping\n", __func__);
    tmp = ((OV1063X_SENSOR_HEIGHT - height_pre_subsample) / 2) & ~0x1;
    pr_info("%s:     register 0x3802 / 03 = %04X \n", __func__, tmp);
    tmp = tmp + height_pre_subsample + 3;
    pr_info("%s:     register 0x3806 / 07 = %04X \n", __func__, tmp);

    /* Output size */

    ret = ovcamera_write_reg16(i2c_client, 0x3808, priv->width);
    if (ret) {
        return ret;
    }
    ret = ovcamera_write_reg16(i2c_client,0x380a, priv->height);
    if (ret) {
        return ret;
    }
    ret = ovcamera_write_reg16( i2c_client,0x380c, hts);
    if (ret) {
        return ret;
    }
    ret = ovcamera_write_reg16(i2c_client, 0x380e, vts);
    if (ret) {
        return ret;
    }

    if (vert_sub_sample) {
            pr_info("%s:  ***   FIX VERT_SUB_SAMPLE!!! \n", __func__);
        /*
        ret = ov1063x_reg_rmw(client, OV1063X_VFLIP,
                              OV1063X_VFLIP_SUBSAMPLE, 0);
        if (ret) {
            return ret;
        }
        ret = ov1063x_set_regs(client, ov1063x_regs_vert_sub_sample,
                               ARRAY_SIZE(ov1063x_regs_vert_sub_sample));
        if (ret) {
            return ret;
        }
        */
    }

    ret = ovcamera_write_reg16(i2c_client,0x4606, 2 * hts);
    if (ret) {
        return ret;
    }
    ret = ovcamera_write_reg16( i2c_client,0x460a, 2 * (hts - width_pre_subsample));
    if (ret) {
        return ret;
    }
    tmp = (vts - 8) * 16;
    ret = ovcamera_write_reg16(i2c_client, 0xc488, tmp);
    if (ret) {
        return ret;
    }
    ret = ovcamera_write_reg16(i2c_client, 0xc48a, tmp);
    if (ret) {
        return ret;
    }
    nr_isp_pixels = sensor_width * (priv->height + 4);
    ret = ovcamera_write_reg16(i2c_client, 0xc4cc, nr_isp_pixels / 256);
    if (ret) {
        return ret;
    }
    ret = ovcamera_write_reg16(i2c_client, 0xc4ce, nr_isp_pixels / 256);
    if (ret) {
        return ret;
    }
    ret = ovcamera_write_reg16(i2c_client, 0xc512, nr_isp_pixels / 16);
    if (ret) {
        return ret;
    }


    pr_info("%s:  Output Size \n", __func__);
    pr_info("%s:     register 0x3808 / 09 = %04X \n", __func__, priv->width);
    pr_info("%s:     register 0x380a / 0b = %04X \n", __func__, priv->height);
    pr_info("%s:     register 0x380c / 0c = %04X \n", __func__, hts);
    pr_info("%s:     register 0x380e / 0f = %04X \n", __func__, vts);


    if (vert_sub_sample) {
                    pr_info("%s:  ***   FIX VERT_SUB_SAMPLE!!! Number 2 \n", __func__);
        /*
        ret = ov1063x_reg_rmw(client, OV1063X_VFLIP, OV1063X_VFLIP_SUBSAMPLE, 0);
        if (ret)
            return ret;
        ret = ov1063x_set_regs(client, ov1063x_regs_vert_sub_sample, ARRAY_SIZE(ov1063x_regs_vert_sub_sample));
        if (ret)
            return ret;
            */
    }


    pr_info("%s:  Vert sub sample \n", __func__);
    pr_info("%s:     register 0x4606 / 07 = %04X \n", __func__, 2 * hts);
    pr_info("%s:    register 0x460a / 0b = %04X \n", __func__, 2 * (hts - width_pre_subsample));

    tmp = (vts - 8) * 16;
    pr_info("%s:     register 0xC488 / 89 = %04X \n", __func__, tmp);
    pr_info("%s:     register 0xc48a / 8b = %04X \n", __func__, tmp);

    nr_isp_pixels = sensor_width * (priv->height + 4);
    pr_info("%s:     register 0xc4cc / cd = %04X \n", __func__, nr_isp_pixels / 256);
    pr_info("%s:     register 0xc4ce / cf = %04X \n", __func__, nr_isp_pixels / 256);
    pr_info("%s:     register 0xc512 / 13 = %04X \n", __func__, nr_isp_pixels / 16);

                /* Horizontal sub-sampling */

    if (horiz_sub_sample) {
        ret = ovcamera_write_reg(i2c_client, 0x5005, 0x9);
        if (ret) {
            return ret;
        }
        ret = ovcamera_write_reg(i2c_client, 0x3007, 0x2);
        if (ret) {
            return ret;
        }
    }
    ret = ovcamera_write_reg16( i2c_client,0xc518, vts);
    if (ret) {
        return ret;
    }
    ret = ovcamera_write_reg16(i2c_client,0xc51a, hts);
    if (ret) {
        return ret;
    }


    if (horiz_sub_sample) {
        ret = ovcamera_write_reg(i2c_client,0x5005, 0x9);
        if (ret) {
            return ret;
        }
        ret = ovcamera_write_reg( i2c_client,0x3007, 0x2);
        if (ret) {
            return ret;
        }
    }


    pr_info("%s: Horizontal sub sample \n", __func__);
    pr_info("%s:     register 0xc518 / 19 = %04X \n", __func__, vts);
    pr_info("%s:     register 0xc51a / 1b = %04X \n", __func__, hts);
}


// call this prior to changing any registry settings
static void ov10633_suspend_pipeline(  struct i2c_client* i2c_client )
{

    pr_info("%s: ov10633_suspend_pipeline \n",__func__);

    ovcamera_write_reg(i2c_client,0x301D, 0xFF);
    ovcamera_write_reg(i2c_client,0x301E, 0xFF);
    ovcamera_write_reg(i2c_client,0x3040, 0xFF);
    ovcamera_write_reg(i2c_client,0x3041, 0xFF);
    ovcamera_write_reg(i2c_client,0x3042, 0xFF);

    // clock reset 1
    ovcamera_write_reg(i2c_client, 0x301B, 0xFF);

    // clock reset 2
    ovcamera_write_reg(i2c_client, 0x301C, 0xFF);

    // clock reset 0
    ovcamera_write_reg(i2c_client, 0x301A, 0xFF);


    /* delay at least 10 ms */
    msleep(2);
}

static void ov10633_enable_pipeline( struct i2c_client* i2c_client )
{

    pr_info("%s: ov10633_enable_pipeline \n",__func__ );

    ovcamera_write_reg( i2c_client,0x0100, 0x01 ); // ensure the video streaming bit is set

    /* delay at least 1 ms */
    msleep(1);

    // clock reset 7
    ovcamera_write_reg(i2c_client, 0x3042, 0xF9);  // some abiguity here ... should this be F0 or F9
    msleep(30);
    ovcamera_write_reg(i2c_client,0x301D, 0xB4);
    ovcamera_write_reg(i2c_client,0x301E, 0xF0);
    ovcamera_write_reg(i2c_client,0x3040, 0xF0);
    ovcamera_write_reg(i2c_client,0x3041, 0xF0);
    // clock reset 1
    ovcamera_write_reg(i2c_client, 0x301B, 0xF0);
    // clock reset 2
    ovcamera_write_reg(i2c_client, 0x301C, 0xF0);
    // clock reset 0
    ovcamera_write_reg(i2c_client, 0x301A, 0xF0);

    /* delay at least 10 ms */
    msleep(30);
}


//** end of 10633 Specific functions

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


static s32 ovcamera_write_reg16(struct i2c_client* i2c_client, u16 reg, u16 val )
{
    int retval;

    retval = ovcamera_write_reg( i2c_client, reg, val >> 8);

    if (retval) {
            return retval;
    }

    retval = ovcamera_write_reg(i2c_client, reg + 1, val & 0xff);
    if (retval) {
         return retval;
    }

    return 0;
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
    struct i2c_client *i2c_client = sensor->i2c_client;
    struct v4l2_fract *timeperframe = &param->parm.capture.timeperframe;
    u32 tgt_fps;	/* target frames per secound */
    enum ov5640_frame_rate frame_rate;
    int ret = 0;

    pr_info("%s \n", __func__ );

	switch (param->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:

        /* Check that the new frame rate is allowed. */
        if ((timeperframe->numerator == 0) ||
            (timeperframe->denominator == 0)) {
            timeperframe->denominator = DEFAULT_FPS;
            timeperframe->numerator = 1;
        }

        tgt_fps = timeperframe->denominator /
              timeperframe->numerator;

        if (tgt_fps > MAX_FPS) {
            timeperframe->denominator = MAX_FPS;
            timeperframe->numerator = 1;
        } else if (tgt_fps < MIN_FPS) {
            timeperframe->denominator = MIN_FPS;
            timeperframe->numerator = 1;
        }

        /* Actual frame rate we use */
        tgt_fps = timeperframe->denominator /
              timeperframe->numerator;

        if (tgt_fps == 15)
            frame_rate = ov5640_15_fps;
        else if (tgt_fps == 30)
            frame_rate = ov5640_30_fps;
        else {
            pr_err(" The camera frame rate is not supported!\n");
            return -EINVAL;
        }

        if ( ov_selected_camera == ov_camera_5640 )
        {
            ret = 0; // ov5640_change_mode(frame_rate, param->parm.capture.capturemode);
        }
        else if ( ov_selected_camera == ov_camera_10633 )
        {
            /*
             * The image size sent over is stable and changes between mode 0 and mode 4
             * However, the image is always dark.
             * I am attributing this to an incorrect suspend or enable sequence
             *
             * Turning off until we get the start up sequence figured out
             *
             *
            ov10633_suspend_pipeline(i2c_client);// need to disable the pipeline while setting the values
            sensor->pix.width = ov5640_mode_info_data[frame_rate][param->parm.capture.capturemode].width;
            sensor->pix.height = ov5640_mode_info_data[frame_rate][param->parm.capture.capturemode].height;
            sensor->streamcap.capturemode = param->parm.capture.capturemode;
            ret = ov10633_change_mode(i2c_client, frame_rate, param->parm.capture.capturemode);
            ov10633_enable_pipeline(i2c_client); // need to reenable the pipeline
            msleep( 500 );
            */
        }

        if (ret < 0)
        {
            pr_err(" ovcamera: The camera  change mode failed!\n");
            return ret;
        }

        sensor->streamcap.timeperframe = *timeperframe;
        sensor->streamcap.capturemode = param->parm.capture.capturemode;

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
    // fsize->discrete.width = 1280;
    // fsize->discrete.height = 720;

    fsize->discrete.width =
            max(ov5640_mode_info_data[0][fsize->index].width,
                ov5640_mode_info_data[1][fsize->index].width);
    fsize->discrete.height =
            max(ov5640_mode_info_data[0][fsize->index].height,
                ov5640_mode_info_data[1][fsize->index].height);

    pr_info("%s: fsize->index %d  format %x width %d height %d \n", __func__,fsize->index ,fsize->pixel_format,fsize->discrete.width,
            fsize->discrete.height );


	
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

     // soft reset
    ovcamera_write_reg(i2c_client, 0x0103, 0x01);
    msleep(2);
    // Control register setup to allow sensor configuration
    ovcamera_write_reg(i2c_client,0x301D, 0xFF);
    ovcamera_write_reg(i2c_client,0x301E, 0xFF);
    ovcamera_write_reg(i2c_client,0x3040, 0xFF);
    ovcamera_write_reg(i2c_client,0x3041, 0xFF);
    ovcamera_write_reg(i2c_client,0x3042, 0xFF);

    ovcamera_write_reg(i2c_client,0x301B, 0xFF);
    ovcamera_write_reg(i2c_client,0x301C, 0xFF);
    ovcamera_write_reg(i2c_client, 0x301A, 0xFF);

    msleep(2);

    // output drive
    ovcamera_write_reg(i2c_client, 0x3011, 0x02 );
    //*** Switch from PAD Clock to all clock ?
    ovcamera_write_reg(i2c_client, 0x3023, 0x10 );


    // PLCLK from System Clock and Short YUV mode
    ovcamera_write_reg(i2c_client, 0x3024, 0x05 );

    // System Clock
    // 82.666 MHz from calculation for Pixel Clock requirement
    // set system clock to 82.66 MHz to match our default settings for now
    ovcamera_write_reg(i2c_client,0x3003, 0x1f);
    ovcamera_write_reg(i2c_client,0x3004, 0x12);

    // Set up the cropping and window regions
    ovcamera_write_reg(i2c_client,0x3800, 0x00);
    ovcamera_write_reg(i2c_client,0x3801, 0x00);
    ovcamera_write_reg(i2c_client,0x3802, 0x00);
    ovcamera_write_reg(i2c_client,0x3803, 0x2E);


    ovcamera_write_reg(i2c_client,0x3804, 0x05);
    ovcamera_write_reg(i2c_client,0x3805, 0x1f);

    ovcamera_write_reg(i2c_client,0x3806, 0x03);
    ovcamera_write_reg(i2c_client,0x3807, 0x01);

    ovcamera_write_reg(i2c_client,0x3808, 0x05);
    ovcamera_write_reg(i2c_client,0x3809, 0x00);
    ovcamera_write_reg(i2c_client,0x380a, 0x02);
    ovcamera_write_reg(i2c_client,0x380b, 0xd0);
    ovcamera_write_reg(i2c_client,0x380C, 0x06);
    ovcamera_write_reg(i2c_client,0x380D, 0xFB);
    ovcamera_write_reg(i2c_client,0x380E, 0x03);
    ovcamera_write_reg(i2c_client,0x380F, 0x03);

     //** cropping set up
    ovcamera_write_reg(i2c_client,0x3621, 0x63);
    ovcamera_write_reg(i2c_client,0x3702, 0x1C);
    ovcamera_write_reg(i2c_client,0x3703, 0x3E);
    ovcamera_write_reg(i2c_client,0x3704, 0x2B);
    ovcamera_write_reg(i2c_client, 0x4606, 0x0D);
    // Vertical sub sample
    ovcamera_write_reg(i2c_client,0x4607, 0xF6);
    ovcamera_write_reg(i2c_client,0x460a, 0x03);
    ovcamera_write_reg(i2c_client, 0x460b, 0xF6);
    ovcamera_write_reg(i2c_client, 0xc488, 0x2F); 
    ovcamera_write_reg(i2c_client,0xc489, 0xB0);
    ovcamera_write_reg(i2c_client, 0xc48a, 0x2F);
    ovcamera_write_reg(i2c_client, 0xc48b, 0xB0);
    ovcamera_write_reg(i2c_client, 0xc4cc, 0x0e);
    ovcamera_write_reg(i2c_client, 0xc4cd, 0x7e);
    ovcamera_write_reg(i2c_client, 0xc4ce, 0x0e);
    ovcamera_write_reg(i2c_client, 0xc4cf, 0x7e);
    ovcamera_write_reg(i2c_client,0xc512, 0xe7);
    ovcamera_write_reg(i2c_client,0xc513, 0xe8);

    // Horizontal sub sample
    ovcamera_write_reg(i2c_client,0xc518, 0x03);
    ovcamera_write_reg(i2c_client, 0xc519, 0x03);
    ovcamera_write_reg(i2c_client,0xc51a, 0x06);
    ovcamera_write_reg(i2c_client, 0xc51b, 0xfb);

    // SET_YUYV_FORMAT
    ovcamera_write_reg(i2c_client,0X4300, 0x38);
    // Set for YUV44 to YUV422 drop AND no-VSYNC latch AND AEC/simple awb/tonemap/combine done
    ovcamera_write_reg(i2c_client,0X5003, 0x14);
    // YUV_8BIT
    ovcamera_write_reg(i2c_client,0x4605, 0x08);
    // Set Video Streaming setting
    ovcamera_write_reg( i2c_client,0x0100, 0x01 );

    // Re-enable chip
    msleep(1);
    ovcamera_write_reg(i2c_client,0x3042, 0xF9);
    msleep(30);  // this was a long delay in the sample (they did many writes to 3042 instead of sleeping)
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
    sensor->streamcap.capturemode = ov5640_mode_720P_1280_720;  // mode 4 default
	sensor->streamcap.capability = V4L2_MODE_HIGHQUALITY | V4L2_CAP_TIMEPERFRAME;
    sensor->streamcap.timeperframe.denominator = DEFAULT_FPS;
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

