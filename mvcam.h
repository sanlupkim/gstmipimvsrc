/* mipimvsrc - a GStreamer source for MIPI machine vision cameras
 *
 * Copyright (C) 2026 Lukasz Sanocki
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If not,
 * see <https://www.gnu.org/licenses/>.
 */

/* Control of Veye/MV MIPI machine vision cameras over I2C.
 *
 * These cameras are not configured through V4L2 controls. Everything - exposure, gain, frame rate,
 * black level, ROI and the sensor's own pixel format - lives in a flat register space reached over
 * I2C, which the vendor's mv_mipi_i2c.sh pokes with two small helper binaries. This is the same
 * protocol reimplemented directly, so the element does not have to shell out to the scripts and does
 * not depend on their location.
 *
 * Protocol: 16 bit register address, 32 bit big-endian value.
 *   write  one 6 byte transfer: reg_hi, reg_lo, then the value, most significant byte first
 *   read   an I2C_RDWR pair: a 2 byte write of the address, then a 4 byte read
 *
 * Scaling is not uniform and the vendor scripts hide it, so it is stated explicitly for every
 * accessor below. Getting it wrong is silent: the camera clamps and carries on.
 */

#ifndef MVCAM_H
#define MVCAM_H

#include <stdint.h>
#include <string>

/* Default I2C address of the camera's control block. */
#define MVCAM_I2C_ADDR 0x3b

/* Return values of detect_bus() that are not a bus number. */
#define MVCAM_DETECT_NONE      (-1)
#define MVCAM_DETECT_AMBIGUOUS (-2)

/* Register map, from the vendor's mv_mipi_i2c_new.sh. Only what this element needs. */
enum MvReg {
    MV_MANUFACTURER      = 0x0000,
    MV_MODEL             = 0x0004,
    MV_PRODUCT_INFO      = 0x000c,   /* serial number */
    MV_DEVICE_VERSION    = 0x0010,
    MV_PARAM_SAVE        = 0x0018,
    MV_ERROR_CODE        = 0x0024,
    MV_FORMAT_CAP        = 0x0028,   /* bitmask of supported pixel formats */
    MV_TEMP_K            = 0x0058,   /* Kelvin x 100 */

    MV_IMAGE_ACQUISITION = 0x0400,   /* 0 stop, 1 start */
    MV_TRIGGER_MODE      = 0x0404,
    MV_SENSOR_FRAMECOUNT = 0x0464,   /* frames the sensor produced */
    MV_OUT_FRAMECOUNT    = 0x0468,   /* frames the camera sent. The gap is what it dropped itself */

    MV_TEST_IMAGE        = 0x0800,
    MV_PIXEL_FORMAT      = 0x0804,   /* 0 raw8, 1 raw10, 2 raw12, 3 raw14, 4 UYVY */
    MV_SENSOR_WIDTH      = 0x0808,
    MV_SENSOR_HEIGHT     = 0x080c,
    MV_MAX_FRAMERATE     = 0x0810,   /* fps x 100 */
    MV_FRAMERATE         = 0x0814,   /* fps x 100 */
    MV_ROI_WIDTH         = 0x0818,
    MV_ROI_HEIGHT        = 0x081c,
    MV_ROI_OFFSET_X      = 0x0820,
    MV_ROI_OFFSET_Y      = 0x0824,
    MV_IMAGE_DIRECTION   = 0x0828,   /* 0 normal, 1 mirror, 2 flip, 3 both */
    MV_BLACK_LEVEL       = 0x0830,
    MV_BLC_MODE          = 0x0834,
    MV_MIN_ROI_WIDTH     = 0x0844,
    MV_MIN_ROI_HEIGHT    = 0x0848,
    MV_MIN_FRAMERATE     = 0x084c,   /* fps x 100 */

    /* One register holds the ISP enables as bits, so changing one is a read-modify-write. */
    MV_ISP_MODULE_CTRL   = 0x0c00,
    MV_EXPOSURE_MODE     = 0x0c04,   /* 0 manual, 1 auto once, 2 auto continuous */
    MV_AE_TARGET         = 0x0c08,   /* target mean brightness the auto loop aims at, 1..255 */
    MV_ME_TIME           = 0x0c10,   /* microseconds, no scaling */
    MV_AE_MAX_TIME       = 0x0c14,   /* ceiling the auto loop may raise exposure to, microseconds */
    MV_EXP_TIME          = 0x0c18,   /* read only, microseconds */
    MV_GAIN_MODE         = 0x0c1c,   /* 0 manual, 1 auto once, 2 auto continuous */
    MV_MANUAL_GAIN       = 0x0c20,   /* dB x 10 */
    MV_AG_MAX_GAIN       = 0x0c24,   /* dB x 10 */
    MV_CUR_GAIN          = 0x0c28,   /* read only, dB x 10 */
    MV_AAROI_OFFSET_X    = 0x0c2c,   /* window the auto loop measures, ignored unless AAROI_ENABLE */
    MV_AAROI_OFFSET_Y    = 0x0c30,
    MV_AAROI_WIDTH       = 0x0c34,
    MV_AAROI_HEIGHT      = 0x0c38,
    MV_GAMMA             = 0x0c60,   /* gamma x 100 */
    MV_LUT_COUNT         = 0x0c74,   /* read only: how many entries the camera wants, 1024 or 4096 */
    MV_LUT_START         = 0x0c78,   /* write 1 to begin a transfer */
    MV_LUT_DONE          = 0x0c7c,   /* write 1 to commit it */
    MV_LUT_DATA          = 0x1800,   /* 0x1800-0x37ff, two 16 bit entries per 32 bit register */
    /* Named DPC_Count in the vendor's driver header, but absent from their published register map and
       read by nothing they ship, so what it actually counts is unverified. */
    MV_DPC_COUNT         = 0x0c6c,
    MV_AAROI_ENABLE      = 0x0c80,   /* 0 whole frame, 1 use the AAROI window */
    MV_MAX_EXP_TIME      = 0x0c8c,   /* read only, microseconds */
    MV_MIN_EXP_TIME      = 0x0c90,   /* read only, units of 0.1 microseconds */
};

/* Bits inside MV_ISP_MODULE_CTRL. */
#define MV_ISP_BIT_DPC   4
#define MV_ISP_BIT_GAMMA 5
#define MV_ISP_BIT_LUT   8

/* Values for MV_PIXEL_FORMAT. */
enum MvPixelFormat {
    MV_FMT_RAW8  = 0,
    MV_FMT_RAW10 = 1,
    MV_FMT_RAW12 = 2,
    MV_FMT_RAW14 = 3,
    MV_FMT_UYVY  = 4,
};

class MvCam {
public:
    MvCam() : fd_(-1), addr_(MVCAM_I2C_ADDR) {}
    ~MvCam() { close(); }

    /* bus is the number in /dev/i2c-N. Returns false and fills error() on failure. */
    bool open(int bus, int addr = MVCAM_I2C_ADDR);

    /* Finds the bus the camera is on by looking for the driver's own sysfs entry, the same way the
       vendor's mv_probe.sh does: /sys/bus/i2c/devices/i2c-N/N-003b/veye_mvcam. Saves having to know
       whether this board puts the camera on bus 4, 6 or 10, which differs between Pi models and
       between the two CSI connectors.

       Returns the bus number, MVCAM_DETECT_NONE if no camera is bound, or MVCAM_DETECT_AMBIGUOUS if
       more than one is. Two cameras on one board is the case that matters: there is nothing here to
       tie a bus to a particular /dev/video node, so guessing would silently configure the wrong
       camera. The caller must ask the user to choose. n_found, if given, gets the count. */
    static int detect_bus(int addr = MVCAM_I2C_ADDR, int *n_found = NULL);

    /* The precise version: works out which camera feeds a particular capture node, so it stays right
       on a board with two of them. video_dev is "/dev/video8" or a bare index.

       It walks sysfs to find the media device that owns the node, then asks that media device for its
       entity names. The sensor's entity is named after its I2C address - "mvcam 4-003b" - so the bus
       is in the name. Returns the bus number or MVCAM_DETECT_NONE. */
    static int detect_bus_for_video(const char *video_dev, int addr = MVCAM_I2C_ADDR);
    void close();
    bool is_open() const { return fd_ >= 0; }

    const std::string &error() const { return error_; }

    bool read32(uint16_t reg, uint32_t *out);
    bool write32(uint16_t reg, uint32_t val);

    /* Identification. These are NOT strings, whatever the register names suggest: the manufacturer
       happens to read as ASCII ("VEYE" is 0x56455945) but the model is a numeric code, the serial is an
       opaque number and the version packs four bytes as C and L pairs. Decoding them as text produces
       convincing nonsense, which is how this was found. */
    bool manufacturer(std::string *out);
    bool model(std::string *out, uint32_t *code);
    bool serial(uint32_t *out);
    bool version(std::string *out);

    /* Several registers are simply absent on some firmware and read back as all ones. Anything built on
       top of a raw read has to treat that as "no answer" rather than as a value. */
    static bool implemented(uint32_t v) { return v != 0xffffffffu; }
    bool temperature_c(double *out);
    bool gamma_value(double *out);
    bool sensor_size(int *w, int *h);
    bool framerate_range(double *min_fps, double *max_fps);
    bool exposure_range_us(double *min_us, double *max_us);
    bool min_roi(int *w, int *h);
    bool format_cap(uint32_t *mask);
    bool dpc_count(uint32_t *n);
    bool error_code(uint32_t *code);
    bool blc_mode(uint32_t *mode);

    /* Frames the sensor produced against frames it actually sent. A growing gap means the camera is
       dropping them before the host ever sees them, which no amount of host-side buffering can fix and
       which otherwise looks exactly like a host problem. */
    bool frame_counts(uint32_t *sensor, uint32_t *out);

    /* NOT provided on purpose: pixel format and ROI.
       The veye_mvcam driver writes Pixel_Format and the ROI registers itself from the media-ctl
       format, so writing either here would leave its cached state disagreeing with the hardware.
       Set them with media_setting_rpi5.sh. get_roi below is read-only, for checking.

       set_framerate IS provided, even though the driver has a working frame_rate control. Writing the
       register directly leaves only the driver's cur_fps cache stale, and both its getter and its ROI
       path re-read the register, so nothing downstream is affected. In exchange the register takes
       hundredths of a frame, where the driver's control is whole frames only.

       Setters. Each reads the register back and reports what the camera actually took, because it
       clamps out-of-range values silently rather than failing the write. */
    bool get_roi(int *x, int *y, int *w, int *h);
    bool set_framerate(double fps, double *actual);
    bool set_exposure_us(int us, int *actual);
    bool set_gain_db(double db, double *actual);
    bool set_black_level(int level, int *actual);
    bool set_gamma(double gamma, double *actual);
    bool set_exposure_mode(int mode);
    bool set_gain_mode(int mode);

    /* Auto exposure. The mode registers above only say who decides; these say what it decides towards,
       and without them "auto" means the vendor's defaults, which are tuned for machine vision rather
       than for a sky that spans full sun to overcast.

       ae_target is a mean output level, 1..255, in the camera's 8 bit terms whatever the output format.
       ae_max_time is the ceiling in microseconds and matters most: left at a default of tens of
       milliseconds the loop will happily reach for a long exposure, which at 10 s cadence is harmless
       but blurs cloud and, more to the point, makes the loop slow to come back down. ag_max_gain caps
       the gain the same way, which is what keeps a dark frame noisy-but-honest rather than smeared. */
    bool set_ae_target(int level, int *actual);
    bool set_ae_max_time_us(int us, int *actual);
    bool set_ag_max_gain_db(double db, double *actual);
    bool set_aa_roi(int x, int y, int w, int h);
    bool set_aa_roi_enable(int on);
    bool get_aa_roi(int *x, int *y, int *w, int *h);

    /* Both live in MV_ISP_MODULE_CTRL, so these are read-modify-write on a shared register. */
    bool set_gamma_enable(int on);
    bool set_dpc_enable(int on);
    bool set_lut_enable(int on);
    bool get_isp_bit(int bit, int *on);
    bool set_blc_mode(int mode, int *actual);

    /* The user LUT. The camera reports the size it wants in MV_LUT_COUNT and it is read-only, so ask
       before building anything. Measured on an IMX178M: 4096 entries in both Mono12 and Mono8, 1024 in
       Mono10 - so the table works on the sensor's 12 bit values, before the logic unit rounds them down
       to the output format. That is what makes it useful: the curve can be applied here and the camera
       asked for Mono8, with nothing left for the host to do.

       values are 0..count-1. Transfer is START, then the data, then DONE, following the vendor's
       lut_rw.c exactly. */
    bool lut_count(uint32_t *count);
    bool upload_lut(const uint16_t *values, int count);
    bool set_image_direction(int dir);
    bool set_test_image(int which);
    bool set_acquisition(int on);
    bool save_params();

    /* Read-only current state, for logging. */
    /* What the camera is actually doing, from MV_EXP_TIME and MV_CUR_GAIN. These are the ones to read
       with the auto loop running: MV_ME_TIME and MV_MANUAL_GAIN still hold whatever was written
       manually, so reading those back reports the request rather than the state. */
    bool get_exposure_us(int *us);
    bool get_gain_db(double *db);

private:
    int fd_;
    int addr_;
    std::string error_;
};

#endif /* MVCAM_H */
