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

/* See mvcam.h for the protocol and for why this is reimplemented rather than shelling out. */

#include "mvcam.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <limits.h>
#include <linux/media.h>
#include <sys/stat.h>

int MvCam::detect_bus(int addr, int *n_found)
{
    if (n_found)
        *n_found = 0;

    DIR *d = opendir("/sys/bus/i2c/devices");
    if (!d)
        return MVCAM_DETECT_NONE;

    int first = MVCAM_DETECT_NONE;
    int count = 0;

    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        int bus = -1;
        if (sscanf(e->d_name, "i2c-%d", &bus) != 1)
            continue;

        /* The driver registers itself as <bus>-<addr in 4 hex digits>, e.g. 10-003b. */
        char path[256];
        snprintf(path, sizeof(path), "/sys/bus/i2c/devices/i2c-%d/%d-%04x/veye_mvcam",
                 bus, bus, addr);

        struct stat st;
        if (stat(path, &st) == 0 && S_ISDIR(st.st_mode)) {
            if (count == 0)
                first = bus;
            count++;
        }
    }

    closedir(d);

    if (n_found)
        *n_found = count;

    /* Scanning the whole directory rather than stopping at the first hit is the point: a second
       camera has to be noticed, not ignored. */
    return (count > 1) ? MVCAM_DETECT_AMBIGUOUS : first;
}

/* Resolves /sys/class/<cls>/<name>/device to the physical device it hangs off, so two class entries
   belonging to the same hardware can be compared. */
static bool parent_device(const char *cls, const char *name, char *out, size_t outlen)
{
    char link[320];
    snprintf(link, sizeof(link), "/sys/class/%s/%s/device", cls, name);
    char *r = realpath(link, NULL);
    if (!r)
        return false;
    snprintf(out, outlen, "%s", r);
    free(r);
    return true;
}

int MvCam::detect_bus_for_video(const char *video_dev, int addr)
{
    if (!video_dev)
        return MVCAM_DETECT_NONE;

    /* Accept "/dev/video8" or a bare "8". */
    const char *node = video_dev;
    char tmp[32];
    bool numeric = *video_dev != '\0';
    for (const char *c = video_dev; *c && numeric; c++)
        if (*c < '0' || *c > '9')
            numeric = false;
    if (numeric) {
        snprintf(tmp, sizeof(tmp), "video%s", video_dev);
        node = tmp;
    } else {
        const char *slash = strrchr(video_dev, '/');
        node = slash ? slash + 1 : video_dev;
    }

    char want[PATH_MAX];
    if (!parent_device("video4linux", node, want, sizeof(want)))
        return MVCAM_DETECT_NONE;

    /* Find the media device hanging off the same hardware as the capture node. */
    DIR *d = opendir("/sys/class/media");
    if (!d)
        return MVCAM_DETECT_NONE;

    int bus = MVCAM_DETECT_NONE;
    struct dirent *e;
    while (bus == MVCAM_DETECT_NONE && (e = readdir(d)) != NULL) {
        if (strncmp(e->d_name, "media", 5) != 0)
            continue;

        char have[PATH_MAX];
        if (!parent_device("media", e->d_name, have, sizeof(have)))
            continue;
        if (strcmp(have, want) != 0)
            continue;

        char path[5 + sizeof(e->d_name) + 1];
        snprintf(path, sizeof(path), "/dev/%s", e->d_name);
        int fd = ::open(path, O_RDWR);
        if (fd < 0)
            continue;

        /* Entity names are all we need, and ENUM_ENTITIES reports them whatever the driver fills in
           for the device-node union. */
        struct media_entity_desc ent;
        __u32 id = 0;
        while (bus == MVCAM_DETECT_NONE) {
            memset(&ent, 0, sizeof(ent));
            ent.id = id | MEDIA_ENT_ID_FLAG_NEXT;
            if (ioctl(fd, MEDIA_IOC_ENUM_ENTITIES, &ent) < 0)
                break;
            id = ent.id;

            /* "<driver> <bus>-<addr in 4 hex digits>", e.g. "mvcam 4-003b". */
            int b = -1;
            unsigned a = 0;
            const char *sp = strrchr(ent.name, ' ');
            if (sp && sscanf(sp + 1, "%d-%4x", &b, &a) == 2
                && a == (unsigned)addr && b >= 0)
                bus = b;
        }
        ::close(fd);
    }

    closedir(d);
    return bus;
}

bool MvCam::open(int bus, int addr)
{
    close();

    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", bus);

    fd_ = ::open(path, O_RDWR);
    if (fd_ < 0) {
        error_ = std::string("cannot open ") + path + ": " + strerror(errno);
        return false;
    }

    /* FORCE because the kernel driver already holds this address. Talking to the control block while
       the driver is bound is how the vendor tools work too. */
    if (ioctl(fd_, I2C_SLAVE_FORCE, addr) < 0) {
        error_ = std::string("cannot set I2C address on ") + path + ": " + strerror(errno);
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    addr_ = addr;
    error_.clear();
    return true;
}

void MvCam::close()
{
    if (fd_ >= 0)
        ::close(fd_);
    fd_ = -1;
}

bool MvCam::write32(uint16_t reg, uint32_t val)
{
    if (fd_ < 0) {
        error_ = "not open";
        return false;
    }

    /* One transfer: address high, address low, then the value most significant byte first. */
    uint8_t msg[6] = {
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff),
        (uint8_t)(val >> 24), (uint8_t)(val >> 16), (uint8_t)(val >> 8), (uint8_t)(val & 0xff)
    };

    if (::write(fd_, msg, sizeof(msg)) != (ssize_t)sizeof(msg)) {
        char buf[96];
        snprintf(buf, sizeof(buf), "write of register 0x%04x failed: %s", reg, strerror(errno));
        error_ = buf;
        return false;
    }
    return true;
}

bool MvCam::read32(uint16_t reg, uint32_t *out)
{
    if (fd_ < 0) {
        error_ = "not open";
        return false;
    }

    uint8_t a[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff) };
    uint8_t d[4] = { 0, 0, 0, 0 };

    struct i2c_msg msgs[2];
    msgs[0].addr = addr_; msgs[0].flags = 0;        msgs[0].len = 2; msgs[0].buf = a;
    msgs[1].addr = addr_; msgs[1].flags = I2C_M_RD; msgs[1].len = 4; msgs[1].buf = d;

    struct i2c_rdwr_ioctl_data set;
    set.msgs = msgs;
    set.nmsgs = 2;

    if (ioctl(fd_, I2C_RDWR, &set) != 2) {
        char buf[96];
        snprintf(buf, sizeof(buf), "read of register 0x%04x failed: %s", reg, strerror(errno));
        error_ = buf;
        return false;
    }

    *out = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3];
    return true;
}

std::string MvCam::id_string(uint16_t reg)
{
    uint32_t v;
    if (!read32(reg, &v))
        return std::string();

    /* Four packed ASCII bytes, most significant first. */
    char s[5] = { (char)(v >> 24), (char)(v >> 16), (char)(v >> 8), (char)v, 0 };
    for (int i = 0; i < 4; i++)
        if (s[i] < 0x20 || s[i] > 0x7e)
            s[i] = ' ';
    return std::string(s);
}

bool MvCam::temperature_c(double *out)
{
    uint32_t v;
    if (!read32(MV_TEMP_K, &v))
        return false;
    *out = v/100.0 - 273.15;
    return true;
}

bool MvCam::sensor_size(int *w, int *h)
{
    uint32_t a, b;
    if (!read32(MV_SENSOR_WIDTH, &a) || !read32(MV_SENSOR_HEIGHT, &b))
        return false;
    *w = (int)a;
    *h = (int)b;
    return true;
}

bool MvCam::framerate_range(double *min_fps, double *max_fps)
{
    uint32_t a, b;
    if (!read32(MV_MIN_FRAMERATE, &a) || !read32(MV_MAX_FRAMERATE, &b))
        return false;
    *min_fps = a/100.0;
    *max_fps = b/100.0;
    return true;
}

bool MvCam::exposure_range_us(double *min_us, double *max_us)
{
    uint32_t a, b;
    if (!read32(MV_MIN_EXP_TIME, &a) || !read32(MV_MAX_EXP_TIME, &b))
        return false;
    *min_us = a/10.0;      /* this one is in units of 0.1 us, unlike every other exposure register */
    *max_us = (double)b;
    return true;
}

bool MvCam::min_roi(int *w, int *h)
{
    uint32_t a, b;
    if (!read32(MV_MIN_ROI_WIDTH, &a) || !read32(MV_MIN_ROI_HEIGHT, &b))
        return false;
    *w = (int)a;
    *h = (int)b;
    return true;
}

bool MvCam::format_cap(uint32_t *mask)
{
    return read32(MV_FORMAT_CAP, mask);
}

bool MvCam::dpc_count(uint32_t *n)
{
    return read32(MV_DPC_COUNT, n);
}



bool MvCam::get_roi(int *x, int *y, int *w, int *h)
{
    uint32_t a, b, c, d;
    if (!read32(MV_ROI_WIDTH, &a) || !read32(MV_ROI_HEIGHT, &b)
        || !read32(MV_ROI_OFFSET_X, &c) || !read32(MV_ROI_OFFSET_Y, &d))
        return false;
    *w = (int)a; *h = (int)b; *x = (int)c; *y = (int)d;
    return true;
}


bool MvCam::set_framerate(double fps, double *actual)
{
    if (!write32(MV_FRAMERATE, (uint32_t)(fps*100.0 + 0.5)))
        return false;
    uint32_t v;
    if (!read32(MV_FRAMERATE, &v))
        return false;
    *actual = v/100.0;
    return true;
}

bool MvCam::set_exposure_us(int us, int *actual)
{
    if (!write32(MV_ME_TIME, (uint32_t)us))
        return false;
    uint32_t v;
    if (!read32(MV_ME_TIME, &v))
        return false;
    *actual = (int)v;
    return true;
}

bool MvCam::set_gain_db(double db, double *actual)
{
    if (!write32(MV_MANUAL_GAIN, (uint32_t)(db*10.0 + 0.5)))
        return false;
    uint32_t v;
    if (!read32(MV_MANUAL_GAIN, &v))
        return false;
    *actual = v/10.0;
    return true;
}

bool MvCam::set_black_level(int level, int *actual)
{
    if (!write32(MV_BLACK_LEVEL, (uint32_t)level))
        return false;
    uint32_t v;
    if (!read32(MV_BLACK_LEVEL, &v))
        return false;
    *actual = (int)v;
    return true;
}

bool MvCam::set_gamma(double gamma, double *actual)
{
    if (!write32(MV_GAMMA, (uint32_t)(gamma*100.0 + 0.5)))
        return false;
    uint32_t v;
    if (!read32(MV_GAMMA, &v))
        return false;
    *actual = v/100.0;
    return true;
}

bool MvCam::set_exposure_mode(int mode) { return write32(MV_EXPOSURE_MODE, (uint32_t)mode); }
bool MvCam::set_gain_mode(int mode)     { return write32(MV_GAIN_MODE, (uint32_t)mode); }
bool MvCam::set_image_direction(int d)  { return write32(MV_IMAGE_DIRECTION, (uint32_t)d); }
bool MvCam::set_test_image(int w)       { return write32(MV_TEST_IMAGE, (uint32_t)w); }
bool MvCam::set_acquisition(int on)     { return write32(MV_IMAGE_ACQUISITION, (uint32_t)(on ? 1 : 0)); }
bool MvCam::save_params()               { return write32(MV_PARAM_SAVE, 1); }

/* The ISP enables share one register, so flipping a bit means reading the others back first. */
static bool isp_set_bit(MvCam *cam, int bit, int on)
{
    uint32_t v;
    if (!cam->read32(MV_ISP_MODULE_CTRL, &v))
        return false;
    uint32_t mask = 1u << bit;
    uint32_t nv = on ? (v | mask) : (v & ~mask);
    if (nv == v)
        return true;
    return cam->write32(MV_ISP_MODULE_CTRL, nv);
}

bool MvCam::set_gamma_enable(int on) { return isp_set_bit(this, MV_ISP_BIT_GAMMA, on); }
bool MvCam::set_dpc_enable(int on)   { return isp_set_bit(this, MV_ISP_BIT_DPC, on); }

bool MvCam::get_isp_bit(int bit, int *on)
{
    uint32_t v;
    if (!read32(MV_ISP_MODULE_CTRL, &v))
        return false;
    *on = (v >> bit) & 1;
    return true;
}

bool MvCam::get_exposure_us(int *us)
{
    uint32_t v;
    if (!read32(MV_EXP_TIME, &v))
        return false;
    *us = (int)v;
    return true;
}

bool MvCam::get_gain_db(double *db)
{
    uint32_t v;
    if (!read32(MV_CUR_GAIN, &v))
        return false;
    *db = v/10.0;
    return true;
}
