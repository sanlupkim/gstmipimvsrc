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

#ifndef PACKAGE
#define PACKAGE "mipimvsrc"
#endif

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

/* OpenCV's V4L2 backend will not go above this (MAX_V4L_BUFFERS in its cap_v4l.cpp; its own default
   is 4). Asking for more releases the buffers it had and then refuses, leaving capture dead. */
#define OPENCV_MAX_V4L_BUFFERS 10

#include "mvcam.h"
using namespace cv;
using namespace std;

/* ======================== STRUCT ======================== */

typedef enum {
    FORMAT_GRAY8,
    FORMAT_Y10P,
    FORMAT_Y12P
} MvFormat;

typedef enum {
    OUT_GRAY8,
    OUT_GRAY16_LE
} MvOut;

typedef enum {
    STAMP_DRIVER,
    STAMP_CLOCK,
    STAMP_FRAMES
} MvStamp;

typedef enum {
    TRANS_LIN,
    TRANS_LSB,
    TRANS_GHS,
    TRANS_GAMMA
} MvTrans;

typedef struct _GstMipiMvSrc {
    GstPushSrc parent;

    cv::VideoCapture *cap;
    cv::Mat frame;
    unsigned char *lut;
    int lut_size;

    gchar *device;
    gint width;
    gint height;
    gint fps_n;
    gint fps_d;
    MvFormat format;
    MvTrans trans;
    MvOut out_format;

    /* Set when the pipeline wants create() to give up. cap->read() cannot be interrupted, so the most
       this can do is stop another one being started, and return as soon as the current one lands. */
    volatile gboolean unlocked;

    /* GHS shape. Sensible values depend on the bit depth, so they are only filled in from the
       per-format defaults for the ones the user did not set explicitly. */
    gfloat ghs_b;
    gfloat ghs_d;
    gfloat ghs_sp;
    gboolean ghs_b_set;
    gboolean ghs_d_set;
    gboolean ghs_sp_set;

    /* Exponent of the gamma transform. RMS inverts exactly this value through config.gamma, which is
       what keeps photometry usable, so it is a plain power law and nothing else. */
    gfloat gamma;

    /* Black level in the camera's own units, subtracted before any stretch. The sensor's offset is
       most of every pixel value and does not scale with gain, so a curve applied on top of it spends
       its steepest part lifting the offset instead of the signal. */
    gint black_level;

    /* Where timestamps come from. The driver stamps a frame in the kernel as it is captured, which is
       what v4l2src uses and the only source immune to how long this element waits to be scheduled. */
    MvStamp stamps;
    double first_driver_ms;

    /* Set when the transform happens to be "keep the top 8 bits", which has a much cheaper path. */
    gboolean high_byte_only;

    gint n_threads;

    /* How many buffers the driver keeps queued. Four covers about 160 ms at 25 fps, so a longer stall
       downstream costs frames the driver had nowhere to put. Going deeper needs V4L2 directly. */
    gint n_buffers;

    guint64 duration;
    guint64 frame_count;

    /* ---- camera control over I2C ----
       These cameras expose nothing through V4L2 controls: exposure, gain, frame rate, black level,
       ROI and the sensor's own pixel format all live in an I2C register space. Driving it from here
       means one pipeline description configures the camera as well, instead of needing set_params.sh
       to have been run with matching values beforehand.

       i2c_bus < 0 leaves the camera completely alone, which is the default. */
    MvCam *i2c;
    gint i2c_bus;
    gint i2c_addr;
    gboolean probe;

    /* Only what nothing else owns. The veye_mvcam driver already drives the sensor's pixel format and
       ROI from the media-ctl format, and its frame_rate / roi_x / roi_y controls work, so writing any
       of those over I2C would desynchronise the driver's cached state from the hardware. Exposure and
       gain are a different case: the driver's V4L2_CID_EXPOSURE and V4L2_CID_ANALOGUE_GAIN only store
       the value ("just save the value" in its own comment) and never reach the camera, which is why
       the vendor's set_params.sh writes them over I2C too.

       -1 means leave alone. */
    gdouble cam_fps;        /* -1 keep, 0 take it from the framerate property */
    gint cam_exposure_us;
    gdouble cam_gain_db;
    gint cam_black_level;
    gdouble cam_gamma;
    gint cam_dpc;
    gint cam_imgdir;
    gint cam_testimg;
    gint cam_auto_exposure;
    gint cam_auto_gain;
    gint cam_lut;
    gint cam_blc;
    gboolean cam_curve;
    gboolean curve_in_camera;   /* set once the upload succeeded */

    /* Baseline of the camera's own frame counters, taken at start. The difference at stop says whether
       the camera dropped frames before the host ever saw them, which looks identical to a host problem
       from the outside. */
    guint32 fc_sensor0;
    guint32 fc_out0;
    gboolean fc_valid;

} GstMipiMvSrc;

typedef struct _GstMipiMvSrcClass {
    GstPushSrcClass parent_class;
} GstMipiMvSrcClass;

G_DEFINE_TYPE(GstMipiMvSrc, gst_mipimv_src, GST_TYPE_PUSH_SRC)

/* ======================== PROPERTIES ======================== */

enum {
    PROP_0,
    PROP_DEVICE,
    PROP_WIDTH,
    PROP_HEIGHT,
    PROP_FRAMERATE,
    PROP_FORMAT,
    PROP_TRANS,
    PROP_GHS_B,
    PROP_GHS_D,
    PROP_GHS_SP,
    PROP_GAMMA,
    PROP_BLACK_LEVEL,
    PROP_OUT_FORMAT,
    PROP_STAMPS,
    PROP_BUFFERS,
    PROP_THREADS,
    PROP_I2C_BUS,
    PROP_I2C_ADDR,
    PROP_PROBE,
    PROP_CAM_FPS,
    PROP_EXPOSURE,
    PROP_GAIN,
    PROP_CAM_BLACK_LEVEL,
    PROP_CAM_GAMMA,
    PROP_CAM_DPC,
    PROP_CAM_IMGDIR,
    PROP_CAM_TESTIMG,
    PROP_CAM_AUTO_EXPOSURE,
    PROP_CAM_AUTO_GAIN,
    PROP_CAM_LUT,
    PROP_CAM_BLC,
    PROP_CAM_CURVE,
    PROP_TEMPERATURE
};

/* Pushes one live control to the camera if the I2C side is up, so a property set while the pipeline
   is already running takes effect instead of being silently stored for next time. Reads back and logs
   what the camera actually took: it clamps out-of-range values rather than refusing them. */
static void mv_apply_live(GstMipiMvSrc *src, guint prop_id);

static void gst_mipimv_src_set_property(GObject *object, guint prop_id,
                                    const GValue *value, GParamSpec *pspec)
{
    GstMipiMvSrc *src = (GstMipiMvSrc *)object;

    switch (prop_id) {
        case PROP_DEVICE:
            g_free(src->device);
            src->device = g_value_dup_string(value);
            break;
        case PROP_WIDTH:
            src->width = g_value_get_int(value);
            break;
        case PROP_HEIGHT:
            src->height = g_value_get_int(value);
            break;
        case PROP_FRAMERATE: {
            src->fps_n = gst_value_get_fraction_numerator(value);
            src->fps_d = gst_value_get_fraction_denominator(value);
            break;
        }
        case PROP_FORMAT: {
            const char *fmt = g_value_get_string(value);
            if (g_strcmp0(fmt, "GRAY8") == 0)
                src->format = FORMAT_GRAY8;
            else if (g_strcmp0(fmt, "Y10P") == 0)
                src->format = FORMAT_Y10P;
            else if (g_strcmp0(fmt, "Y12P") == 0)
                src->format = FORMAT_Y12P;
            else
                g_warning("mipimvsrc: unknown format '%s', keeping the previous one", fmt);
            break;
        }
        case PROP_TRANS: {
            const char *trans = g_value_get_string(value);
            if (g_strcmp0(trans, "lin") == 0)
                src->trans = TRANS_LIN;
            else if (g_strcmp0(trans, "lsb") == 0)
                src->trans = TRANS_LSB;
            else if (g_strcmp0(trans, "ghs") == 0)
                src->trans = TRANS_GHS;
            else if (g_strcmp0(trans, "gamma") == 0)
                src->trans = TRANS_GAMMA;
            else
                g_warning("mipimvsrc: unknown trans '%s', keeping the previous one", trans);
            break;
        }
        case PROP_OUT_FORMAT: {
            const char *of = g_value_get_string(value);
            if (g_strcmp0(of, "GRAY8") == 0)
                src->out_format = OUT_GRAY8;
            else if (g_strcmp0(of, "GRAY16_LE") == 0)
                src->out_format = OUT_GRAY16_LE;
            else
                g_warning("mipimvsrc: unknown out-format '%s', keeping the previous one", of);
            break;
        }
        case PROP_GHS_B:
            src->ghs_b = g_value_get_float(value);
            src->ghs_b_set = TRUE;
            break;
        case PROP_GHS_D:
            src->ghs_d = g_value_get_float(value);
            src->ghs_d_set = TRUE;
            break;
        case PROP_GHS_SP:
            src->ghs_sp = g_value_get_float(value);
            src->ghs_sp_set = TRUE;
            break;
        case PROP_GAMMA:
            src->gamma = g_value_get_float(value);
            break;
        case PROP_BLACK_LEVEL:
            src->black_level = g_value_get_int(value);
            break;
        case PROP_STAMPS: {
            const char *st = g_value_get_string(value);
            if (g_strcmp0(st, "driver") == 0)
                src->stamps = STAMP_DRIVER;
            else if (g_strcmp0(st, "clock") == 0)
                src->stamps = STAMP_CLOCK;
            else if (g_strcmp0(st, "frames") == 0)
                src->stamps = STAMP_FRAMES;
            else
                g_warning("mipimvsrc: unknown stamps '%s', keeping the previous one", st);
            break;
        }
        case PROP_THREADS:
            src->n_threads = g_value_get_int(value);
            break;
        case PROP_BUFFERS:
            src->n_buffers = g_value_get_int(value);
            break;

        /* ---- camera control ---- */
        case PROP_I2C_BUS:
            src->i2c_bus = g_value_get_int(value);
            break;
        case PROP_I2C_ADDR:
            src->i2c_addr = g_value_get_int(value);
            break;
        case PROP_PROBE:
            src->probe = g_value_get_boolean(value);
            break;
        case PROP_CAM_FPS:
            src->cam_fps = g_value_get_double(value);
            break;
        case PROP_EXPOSURE:
            src->cam_exposure_us = g_value_get_int(value);
            mv_apply_live(src, prop_id);
            break;
        case PROP_GAIN:
            src->cam_gain_db = g_value_get_double(value);
            mv_apply_live(src, prop_id);
            break;
        case PROP_CAM_BLACK_LEVEL:
            src->cam_black_level = g_value_get_int(value);
            mv_apply_live(src, prop_id);
            break;
        case PROP_CAM_GAMMA:
            src->cam_gamma = g_value_get_double(value);
            mv_apply_live(src, prop_id);
            break;
        case PROP_CAM_DPC:
            src->cam_dpc = g_value_get_int(value);
            mv_apply_live(src, prop_id);
            break;
        case PROP_CAM_IMGDIR:
            src->cam_imgdir = g_value_get_int(value);
            mv_apply_live(src, prop_id);
            break;
        case PROP_CAM_TESTIMG:
            src->cam_testimg = g_value_get_int(value);
            mv_apply_live(src, prop_id);
            break;
        case PROP_CAM_AUTO_EXPOSURE:
            src->cam_auto_exposure = g_value_get_int(value);
            mv_apply_live(src, prop_id);
            break;
        case PROP_CAM_AUTO_GAIN:
            src->cam_auto_gain = g_value_get_int(value);
            mv_apply_live(src, prop_id);
            break;
        case PROP_CAM_LUT:
            src->cam_lut = g_value_get_int(value);
            mv_apply_live(src, prop_id);
            break;
        case PROP_CAM_BLC:
            src->cam_blc = g_value_get_int(value);
            mv_apply_live(src, prop_id);
            break;
        case PROP_CAM_CURVE:
            src->cam_curve = g_value_get_boolean(value);
            break;

        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    }
}

static void gst_mipimv_src_get_property(GObject *object, guint prop_id,
                                    GValue *value, GParamSpec *pspec)
{
    GstMipiMvSrc *src = (GstMipiMvSrc *)object;

    switch (prop_id) {
        case PROP_DEVICE:
            g_value_set_string(value, src->device);
            break;
        case PROP_WIDTH:
            g_value_set_int(value, src->width);
            break;
        case PROP_HEIGHT:
            g_value_set_int(value, src->height);
            break;
        case PROP_FRAMERATE:
            gst_value_set_fraction(value, src->fps_n, src->fps_d);
            break;
        case PROP_FORMAT: {
            const char *fmt_str = "GRAY8";
            if (src->format == FORMAT_Y10P) fmt_str = "Y10P";
            else if (src->format == FORMAT_Y12P) fmt_str = "Y12P";
            g_value_set_string(value, fmt_str);
            break;
        }
        case PROP_TRANS: {
            const char *trans_str = "lin";
            if (src->trans == TRANS_LSB) trans_str = "lsb";
            else if (src->trans == TRANS_GHS) trans_str = "ghs";
            else if (src->trans == TRANS_GAMMA) trans_str = "gamma";
            g_value_set_string(value, trans_str);
            break;
        }
        case PROP_OUT_FORMAT:
            g_value_set_string(value, src->out_format == OUT_GRAY16_LE ? "GRAY16_LE" : "GRAY8");
            break;
        case PROP_GHS_B:
            g_value_set_float(value, src->ghs_b);
            break;
        case PROP_GHS_D:
            g_value_set_float(value, src->ghs_d);
            break;
        case PROP_GHS_SP:
            g_value_set_float(value, src->ghs_sp);
            break;
        case PROP_GAMMA:
            g_value_set_float(value, src->gamma);
            break;
        case PROP_BLACK_LEVEL:
            g_value_set_int(value, src->black_level);
            break;
        case PROP_STAMPS: {
            const char *st = "driver";
            if (src->stamps == STAMP_CLOCK) st = "clock";
            else if (src->stamps == STAMP_FRAMES) st = "frames";
            g_value_set_string(value, st);
            break;
        }
        case PROP_THREADS:
            g_value_set_int(value, src->n_threads);
            break;
        case PROP_BUFFERS:
            g_value_set_int(value, src->n_buffers);
            break;

        /* ---- camera control ---- */
        case PROP_I2C_BUS:
            g_value_set_int(value, src->i2c_bus);
            break;
        case PROP_I2C_ADDR:
            g_value_set_int(value, src->i2c_addr);
            break;
        case PROP_PROBE:
            g_value_set_boolean(value, src->probe);
            break;
        case PROP_CAM_FPS:
            g_value_set_double(value, src->cam_fps);
            break;
        case PROP_EXPOSURE:
            g_value_set_int(value, src->cam_exposure_us);
            break;
        case PROP_GAIN:
            g_value_set_double(value, src->cam_gain_db);
            break;
        case PROP_CAM_BLACK_LEVEL:
            g_value_set_int(value, src->cam_black_level);
            break;
        case PROP_CAM_GAMMA:
            g_value_set_double(value, src->cam_gamma);
            break;
        case PROP_CAM_DPC:
            g_value_set_int(value, src->cam_dpc);
            break;
        case PROP_CAM_IMGDIR:
            g_value_set_int(value, src->cam_imgdir);
            break;
        case PROP_CAM_TESTIMG:
            g_value_set_int(value, src->cam_testimg);
            break;
        case PROP_CAM_AUTO_EXPOSURE:
            g_value_set_int(value, src->cam_auto_exposure);
            break;
        case PROP_CAM_AUTO_GAIN:
            g_value_set_int(value, src->cam_auto_gain);
            break;
        case PROP_CAM_LUT:
            g_value_set_int(value, src->cam_lut);
            break;
        case PROP_CAM_BLC:
            g_value_set_int(value, src->cam_blc);
            break;
        case PROP_CAM_CURVE:
            g_value_set_boolean(value, src->cam_curve);
            break;
        case PROP_TEMPERATURE: {
            /* Read on demand, so a long run can be correlated with sensor temperature. Dark current
               moves with it, and so does the amplitude of the hot pixel pattern. */
            double t = -1000.0;
            if (src->i2c && src->i2c->is_open())
                src->i2c->temperature_c(&t);
            g_value_set_double(value, t);
            break;
        }

        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    }
}

/* ======================== CAPS ======================== */

static GstCaps* gst_mipimv_src_get_caps(GstBaseSrc *basesrc, GstCaps *filter)
{
    GstMipiMvSrc *src = (GstMipiMvSrc *)basesrc;

    GstCaps *caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, (src->out_format == OUT_GRAY16_LE) ? "GRAY16_LE" : "GRAY8",
        "width", G_TYPE_INT, src->width,
        "height", G_TYPE_INT, src->height,
        "framerate", GST_TYPE_FRACTION, src->fps_n, src->fps_d,
        NULL
    );

    // Honour the filter, otherwise negotiation with a downstream capsfilter can fail
    if (filter) {
        GstCaps *intersection = gst_caps_intersect_full(filter, caps, GST_CAPS_INTERSECT_FIRST);
        gst_caps_unref(caps);
        caps = intersection;
    }

    return caps;
}

/* ================== Helpers =========================== */

static gboolean is_number(const char *str)
{
    if (!str || !*str) return FALSE;

    for (const char *p = str; *p; ++p) {
        if (!g_ascii_isdigit(*p))
            return FALSE;
    }
    return TRUE;
}


/* Maps a raw code onto 0..1, with the black level moved to zero so the stretch only acts on signal. */
static float normalise_input(int i, int lut_size, int black)
{
    if (black >= lut_size - 1)
        black = 0;

    float x = (float)(i - black)/(float)(lut_size - 1 - black);
    return CLAMP(x, 0.0f, 1.0f);
}


/* Generalized Hyperbolic Stretch, as used by Siril.

   Only the two branches around the symmetry point are needed: the outer ones apply below LP and above
   HP, which are fixed at 0 and 1 here, so they never cover any input. */
/* Fills out[0..lut_size-1] with the curve as normalised 0..1 values. The 8 bit table this element
   applies and the 12 bit table uploaded into the camera are both scalings of this one array, so there
   is no second copy of the formula to drift out of step. */
static void build_ghs_curve(float *out, int lut_size, float B, float D, float SP, int black)
{
    const float LP = 0.0f, HP = 1.0f;

    B = -B;

    // B == 1 puts a zero in every denominator below, and a flat curve carries no information anyway
    if (fabsf(B - 1.0f) < 1e-6f || D <= 0.0f) {
        g_warning("mipimvsrc: GHS parameters degenerate (b=%.3f d=%.3f), falling back to linear", -B, D);
        int div = lut_size/256;
        for (int i = 0; i < lut_size; i++)
            out[i] = (float)(i/(div > 0 ? div : 1))/255.0f;
        return;
    }

    float qlp = (1.0f - powf(1.0f + D*B*(SP - LP), (B - 1.0f)/B))/(B - 1.0f);
    float q0  = qlp - D*LP*powf(1.0f + D*B*(SP - LP), -1.0f/B);
    float qwp = (powf(1.0f + D*B*(HP - SP), (B - 1.0f)/B) - 1.0f)/(B - 1.0f);
    float q1  = qwp + D*(1.0f - HP)*powf(1.0f + D*B*(HP - SP), -1.0f/B);

    if (fabsf(q1 - q0) < 1e-9f) {
        g_warning("mipimvsrc: GHS curve is degenerate, falling back to linear");
        int div = lut_size/256;
        for (int i = 0; i < lut_size; i++)
            out[i] = (float)(i/(div > 0 ? div : 1))/255.0f;
        return;
    }

    float q = 1.0f/(q1 - q0);

    float a2 = (1.0f/(B - 1.0f) - q0)*q;
    float b2 = -q/(B - 1.0f);
    float c2 = 1.0f + D*B*SP;
    float d2 = -D*B;
    float e2 = (B - 1.0f)/B;

    float a3 = (-1.0f/(B - 1.0f) - q0)*q;
    float b3 = q/(B - 1.0f);
    float c3 = 1.0f - D*B*SP;
    float d3 = D*B;
    float e3 = (B - 1.0f)/B;

    for (int i = 0; i < lut_size; i++) {
        float x = normalise_input(i, lut_size, black);
        float y;

        if (x < SP)
            y = a2 + b2*powf(c2 + d2*x, e2);
        else
            y = a3 + b3*powf(c3 + d3*x, e3);

        if (!std::isfinite(y))
            y = 0.0f;

        out[i] = CLAMP(y, 0.0f, 1.0f);
    }
}


/* The 8 bit table this element applies while mapping down to GRAY8. */
static void build_ghs_lut(unsigned char *lut, int lut_size, float B, float D, float SP, int black)
{
    float *curve = g_new(float, lut_size);
    build_ghs_curve(curve, lut_size, B, D, SP, black);

    for (int i = 0; i < lut_size; i++)
        lut[i] = (unsigned char)lrintf(255.0f*curve[i]);

    g_free(curve);
}


/* ======================== START ======================== */
// gst-launch-1.0 mipimvsrc format=Y12P framerate=25/1 width=1920 height=1080 ! videoconvert ! filesink location=/tmp/mp2.raw
//
/* ======================== CAMERA CONTROL ======================== */

static void mv_apply_live(GstMipiMvSrc *src, guint prop_id)
{
    if (!src->i2c || !src->i2c->is_open())
        return;

    switch (prop_id) {
        case PROP_EXPOSURE:
            if (src->cam_exposure_us >= 0) {
                int got = 0;
                if (src->i2c->set_exposure_us(src->cam_exposure_us, &got))
                    g_print("camera: exposure %d us -> %d us\n", src->cam_exposure_us, got);
                else
                    g_printerr("camera: exposure failed: %s\n", src->i2c->error().c_str());
            }
            break;
        case PROP_GAIN:
            if (src->cam_gain_db >= 0.0) {
                double got = 0;
                if (src->i2c->set_gain_db(src->cam_gain_db, &got))
                    g_print("camera: gain %.1f dB -> %.1f dB\n", src->cam_gain_db, got);
                else
                    g_printerr("camera: gain failed: %s\n", src->i2c->error().c_str());
            }
            break;
        case PROP_CAM_BLACK_LEVEL:
            if (src->cam_black_level >= 0) {
                int got = 0;
                if (src->i2c->set_black_level(src->cam_black_level, &got))
                    g_print("camera: black level %d -> %d\n", src->cam_black_level, got);
            }
            break;
        case PROP_CAM_GAMMA:
            if (src->cam_gamma >= 0.0) {
                double got = 0;
                /* A gamma of 0 is how you ask for the curve to be bypassed. */
                src->i2c->set_gamma_enable(src->cam_gamma > 0.0 ? 1 : 0);
                if (src->cam_gamma > 0.0 && src->i2c->set_gamma(src->cam_gamma, &got))
                    g_print("camera: gamma %.2f -> %.2f\n", src->cam_gamma, got);
            }
            break;
        case PROP_CAM_DPC:
            if (src->cam_dpc >= 0) {
                src->i2c->set_dpc_enable(src->cam_dpc);
                g_print("camera: defect pixel correction %s\n", src->cam_dpc ? "on" : "off");
            }
            break;
        case PROP_CAM_IMGDIR:
            if (src->cam_imgdir >= 0)
                src->i2c->set_image_direction(src->cam_imgdir);
            break;
        case PROP_CAM_TESTIMG:
            if (src->cam_testimg >= 0)
                src->i2c->set_test_image(src->cam_testimg);
            break;
        case PROP_CAM_AUTO_EXPOSURE:
            if (src->cam_auto_exposure >= 0)
                src->i2c->set_exposure_mode(src->cam_auto_exposure);
            break;
        case PROP_CAM_AUTO_GAIN:
            if (src->cam_auto_gain >= 0)
                src->i2c->set_gain_mode(src->cam_auto_gain);
            break;
        case PROP_CAM_LUT:
            if (src->cam_lut >= 0) {
                src->i2c->set_lut_enable(src->cam_lut);
                g_print("camera: user LUT %s\n", src->cam_lut ? "on" : "off");
            }
            break;
        case PROP_CAM_BLC:
            if (src->cam_blc >= 0) {
                int got = 0;
                if (src->i2c->set_blc_mode(src->cam_blc, &got))
                    g_print("camera: black level correction mode %d -> %d\n", src->cam_blc, got);
            }
            break;
        default:
            break;
    }
}

/* Reads and logs what the camera is and what it can do. Cheap, and the only way to find out the
   sensor's real size and limits without the vendor scripts. */
static void mv_log_probe(MvCam *cam)
{
    g_print("camera: %s %s, sensor %s, serial %s, firmware %s\n",
            cam->id_string(MV_MANUFACTURER).c_str(), cam->id_string(MV_MODEL).c_str(),
            cam->id_string(MV_SENSOR_NAME).c_str(), cam->id_string(MV_PRODUCT_INFO).c_str(),
            cam->id_string(MV_DEVICE_VERSION).c_str());

    int w = 0, h = 0;
    if (cam->sensor_size(&w, &h))
        g_print("camera: sensor %dx%d\n", w, h);

    int rx = 0, ry = 0, rw = 0, rh = 0;
    if (cam->get_roi(&rx, &ry, &rw, &rh))
        g_print("camera: current ROI %dx%d at %d,%d\n", rw, rh, rx, ry);

    double lo = 0, hi = 0;
    if (cam->framerate_range(&lo, &hi))
        g_print("camera: frame rate %.2f..%.2f fps\n", lo, hi);
    if (cam->exposure_range_us(&lo, &hi))
        g_print("camera: exposure %.1f..%.0f us\n", lo, hi);

    if (cam->min_roi(&w, &h))
        g_print("camera: smallest ROI %dx%d\n", w, h);

    uint32_t mask = 0;
    if (cam->format_cap(&mask)) {
        /* The capability word is a bitmask over the same order as MvPixelFormat. */
        g_print("camera: formats supported:%s%s%s%s\n",
                (mask & (1u << MV_FMT_RAW8))  ? " raw8"  : "",
                (mask & (1u << MV_FMT_RAW10)) ? " raw10" : "",
                (mask & (1u << MV_FMT_RAW12)) ? " raw12" : "",
                (mask & (1u << MV_FMT_UYVY))  ? " uyvy"  : "");
    }

    double t = 0;
    if (cam->temperature_c(&t))
        g_print("camera: sensor temperature %.1f C\n", t);

    int bit = 0;
    if (cam->get_isp_bit(MV_ISP_BIT_DPC, &bit))
        g_print("camera: defect pixel correction %s\n", bit ? "on" : "off");

    /* 0xC6C is named DPC_Count in the vendor's driver header but does not appear in their published
       register map and nothing they ship reads it, so what it counts is a guess. Printed as a raw
       register value rather than dressed up as a pixel count. */
    uint32_t dpc = 0;
    if (cam->dpc_count(&dpc))
        g_print("camera: register 0xC6C (DPC_Count in the vendor header, undocumented) = %u\n", dpc);

    /* A user LUT left enabled quietly rewrites every pixel on top of gamma, which is close to
       undiagnosable from the host side, so it is worth saying out loud. */
    if (cam->get_isp_bit(MV_ISP_BIT_LUT, &bit))
        g_print("camera: user LUT %s%s\n", bit ? "ON" : "off",
                bit ? "  <- this alters pixel values on top of gamma" : "");
    if (cam->get_isp_bit(MV_ISP_BIT_GAMMA, &bit))
        g_print("camera: internal gamma %s\n", bit ? "on" : "off");

    uint32_t m = 0;
    if (cam->blc_mode(&m))
        g_print("camera: black level correction mode %u\n", m);

    uint32_t err = 0;
    if (cam->error_code(&err) && err)
        g_printerr("camera: error code 0x%x\n", err);

    int us = 0;
    double db = 0;
    if (cam->get_exposure_us(&us) && cam->get_gain_db(&db))
        g_print("camera: now exposing %d us at %.1f dB\n", us, db);
}

/* Builds this element's transfer curve at the camera's own LUT resolution and uploads it, so the camera
   applies it and the host does not have to.

   Measured on an IMX178M: the LUT is 4096 entries in Mono12 AND in Mono8, 1024 in Mono10, which places
   it before the logic unit that rounds 12 bit sensor values down to the output format. Confirmed on real
   data: with a gamma 0.5 table loaded, a population sitting at 199 came back at 903, and
   4095*(199/4095)^0.5 is 903.

   The point is the combination with Mono8 output: the camera then hands over exactly what convertRows
   would have produced, so the unpacking and the table lookup disappear and the MIPI payload halves. */
static gboolean mv_upload_curve(GstMipiMvSrc *src, float B, float D, float SP)
{
    uint32_t count = 0;
    if (!src->i2c->lut_count(&count) || count < 2 || count > 65536) {
        g_printerr("camera: LUT size came back as %u, not uploading a curve\n", count);
        return FALSE;
    }

    /* The camera reports the size; it is read-only, so the curve is built to fit rather than chosen.

       The top of the range is 255*(count/256), not count-1. The logic unit rounds the table's output
       down to 8 bits by dividing by count/256, and 4095/16 is 255.94, so filling the table to its
       absolute maximum leaves every value a level high. Scaling to 4080 instead makes the camera's
       Mono8 output identical to the byte this element would have produced: measured median difference
       0 against max 1 either way, under both rounding and truncation. */
    float out_max;
    if (count >= 256 && count % 256 == 0)
        out_max = 255.0f*(float)(count/256);
    else
        out_max = (float)(count - 1);   /* not a shape we have seen; keep it sane */
    guint16 *tbl = g_new(guint16, count);

    if (src->trans == TRANS_GHS) {

        float *curve = g_new(float, count);
        build_ghs_curve(curve, (int)count, B, D, SP, src->black_level);
        for (uint32_t i = 0; i < count; i++)
            tbl[i] = (guint16)lrintf(out_max*curve[i]);
        g_free(curve);

    } else if (src->trans == TRANS_GAMMA) {

        for (uint32_t i = 0; i < count; i++) {
            float x = normalise_input((int)i, (int)count, src->black_level);
            tbl[i] = (guint16)lrintf(out_max*powf(x, src->gamma));
        }

    } else if (src->trans == TRANS_LSB) {

        /* Keep the bottom 8 bits at full resolution and clip, scaled into the output range. */
        for (uint32_t i = 0; i < count; i++)
            tbl[i] = (guint16)lrintf(out_max*(float)MIN(i, 255u)/255.0f);

    } else {

        /* Linear is what the camera already does, so there would be nothing to upload. */
        g_printerr("camera: trans=lin is what the camera does anyway, not uploading a curve\n");
        g_free(tbl);
        return FALSE;
    }

    gboolean ok = src->i2c->upload_lut(tbl, (int)count);
    g_free(tbl);

    if (!ok) {
        g_printerr("camera: LUT upload failed: %s\n", src->i2c->error().c_str());
        return FALSE;
    }

    /* Two curves in series would be applied twice, so make sure the camera's own gamma is out of the
       way, then switch the table on. */
    src->i2c->set_gamma_enable(0);
    src->i2c->set_lut_enable(1);

    g_print("camera: uploaded a %u entry %s curve and enabled it\n", count,
            (src->trans == TRANS_GHS) ? "GHS" : (src->trans == TRANS_GAMMA) ? "gamma" : "lsb");

    /* The host must now stop curving, and that is not optional. The uploaded table already folds in
       black-level, so the camera's output has black at zero; applying the host curve on top subtracts
       the black level a second time and everything below it clamps to zero. The first version of this
       only warned when format was not GRAY8, which missed the real problem: even with GRAY8 the host
       table maps everything under black-level to black, so the frame came out completely black. */
    src->curve_in_camera = TRUE;
    return TRUE;
}

/* Everything that has to be in place before the V4L2 device is opened. Returns FALSE only for a
   failure that would leave the stream misconfigured; a control that merely could not be written is
   logged and does not stop the pipeline. */
static gboolean mv_setup_camera(GstMipiMvSrc *src)
{
    /* Cleared on every start: a pipeline that goes READY and back must not inherit a stale TRUE from
       a previous run whose upload happened to succeed. */
    src->curve_in_camera = FALSE;

    if (src->i2c_bus == -1) {

        /* Say so rather than silently dropping them: passing gain=30 and getting no gain change is a
           confusing way to find out that camera control is off by default. */
        GString *ignored = g_string_new(NULL);
        if (src->cam_exposure_us >= 0)   g_string_append(ignored, " exposure");
        if (src->cam_gain_db >= 0.0)     g_string_append(ignored, " gain");
        if (src->cam_fps >= 0.0)         g_string_append(ignored, " cam-fps");
        if (src->cam_black_level >= 0)   g_string_append(ignored, " cam-black-level");
        if (src->cam_gamma >= 0.0)       g_string_append(ignored, " cam-gamma");
        if (src->cam_dpc >= 0)           g_string_append(ignored, " cam-dpc");
        if (src->cam_imgdir >= 0)        g_string_append(ignored, " cam-imgdir");
        if (src->cam_testimg >= 0)       g_string_append(ignored, " cam-testimg");
        if (src->cam_auto_exposure >= 0) g_string_append(ignored, " cam-auto-exposure");
        if (src->cam_auto_gain >= 0)     g_string_append(ignored, " cam-auto-gain");
        if (src->cam_lut >= 0)           g_string_append(ignored, " cam-lut");
        if (src->cam_blc >= 0)           g_string_append(ignored, " cam-blc");
        if (src->cam_curve)              g_string_append(ignored, " cam-curve");
        if (src->probe)                  g_string_append(ignored, " probe");

        if (ignored->len)
            g_printerr("mipimvsrc: i2c-bus is -1, so the camera is left alone and these were "
                       "ignored:%s. Add i2c-bus=-2 to apply them.\n", ignored->str);

        g_string_free(ignored, TRUE);
        return TRUE;
    }

    /* An explicit bus number is a request, so failing to reach it is an error. Autodetection is a
       convenience, so not finding a camera only costs the camera control - the pipeline still runs,
       exactly as it does with i2c-bus=-1. The one case that must not be papered over is two cameras:
       nothing here ties a bus to a /dev/video node, so picking one would configure the wrong camera
       on a two-camera board. */
    gboolean detecting = (src->i2c_bus == -2);
    int bus = src->i2c_bus;

    if (detecting) {
        /* First ask which camera actually feeds this capture node, by walking the media graph. That is
           the only answer that stays right with two cameras on one board. */
        bus = MvCam::detect_bus_for_video(src->device, src->i2c_addr);

        if (bus >= 0) {
            g_print("camera: %s is fed by the camera on bus %d\n", src->device, bus);
        } else {
            /* No media graph to follow, so fall back to looking for a camera anywhere, which can only
               be trusted when there is exactly one. */
            int n = 0;
            bus = MvCam::detect_bus(src->i2c_addr, &n);

            if (bus == MVCAM_DETECT_AMBIGUOUS) {
                g_printerr("camera: %d cameras on this board and the media graph did not say which "
                           "one feeds %s. Give the bus number explicitly.\n", n, src->device);
                return FALSE;
            }
            if (bus == MVCAM_DETECT_NONE) {
                g_print("camera: no veye_mvcam found, continuing without camera control\n");
                return TRUE;
            }
            g_print("camera: detected on bus %d\n", bus);
        }
    }

    src->i2c = new MvCam();
    if (!src->i2c->open(bus, src->i2c_addr)) {
        g_printerr("camera: %s\n", src->i2c->error().c_str());
        delete src->i2c;
        src->i2c = NULL;
        /* Usually a permissions problem: the user needs to be in the i2c group. Worth saying out
           loud, but not worth refusing to capture over when it was only autodetection asking. */
        if (detecting) {
            g_print("camera: continuing without camera control\n");
            return TRUE;
        }
        return FALSE;
    }
    g_print("camera: control on /dev/i2c-%d at 0x%02x\n", bus, src->i2c_addr);

    if (src->probe)
        mv_log_probe(src->i2c);

    /* The ROI the driver has, purely so a mismatch with this element's width and height is visible
       in the log instead of showing up as a sheared image. Not written: the driver owns it. */
    int rx = 0, ry = 0, rw = 0, rh = 0;
    if (src->i2c->get_roi(&rx, &ry, &rw, &rh)) {
        g_print("camera: ROI %dx%d at %d,%d\n", rw, rh, rx, ry);
        if (rw != src->width || rh != src->height)
            g_printerr("camera: ROI is %dx%d but this element is set to %dx%d. Fix the "
                       "media_setting_rpi5.sh call; unpacking a mismatched size shears the image.\n",
                       rw, rh, src->width, src->height);
    }

    /* Frame rate. 0 takes it from the framerate property, so the camera and the negotiated caps
       cannot drift apart. Written here and not live: changing it mid-stream would leave the caps
       claiming a rate the camera no longer delivers. */
    double want_fps = src->cam_fps;
    if (want_fps == 0.0 && src->fps_d > 0)
        want_fps = (double)src->fps_n/src->fps_d;
    if (want_fps > 0.0) {
        double got = 0;
        if (src->i2c->set_framerate(want_fps, &got))
            g_print("camera: frame rate %.2f -> %.2f fps\n", want_fps, got);
        else
            g_printerr("camera: frame rate failed: %s\n", src->i2c->error().c_str());
    }

    /* Modes before values: writing a manual exposure while the camera is in auto mode is accepted and
       then immediately overridden by the auto loop. */
    mv_apply_live(src, PROP_CAM_AUTO_EXPOSURE);
    mv_apply_live(src, PROP_CAM_AUTO_GAIN);
    mv_apply_live(src, PROP_EXPOSURE);
    mv_apply_live(src, PROP_GAIN);
    mv_apply_live(src, PROP_CAM_BLACK_LEVEL);
    mv_apply_live(src, PROP_CAM_GAMMA);
    mv_apply_live(src, PROP_CAM_DPC);
    mv_apply_live(src, PROP_CAM_IMGDIR);
    mv_apply_live(src, PROP_CAM_TESTIMG);
    mv_apply_live(src, PROP_CAM_LUT);
    mv_apply_live(src, PROP_CAM_BLC);

    /* After the other controls: uploading the curve turns the camera's own gamma off, so it must not be
       re-enabled afterwards by cam-gamma. */
    if (src->cam_curve)
        mv_upload_curve(src, src->ghs_b, src->ghs_d, src->ghs_sp);

    src->fc_valid = src->i2c->frame_counts(&src->fc_sensor0, &src->fc_out0);

    return TRUE;
}

static gboolean gst_mipimv_src_start(GstBaseSrc *basesrc)
{
    GstMipiMvSrc *src = (GstMipiMvSrc *)basesrc;

    float B = src->ghs_b, D = src->ghs_d, SP = src->ghs_sp;

    g_free(src->lut);
    src->lut = NULL;

    if (src->format == FORMAT_GRAY8) {
        src->lut_size = 256;
    } else if (src->format == FORMAT_Y10P) {
        src->lut_size = 1024;
        if (!src->ghs_b_set)  B  = -0.99f;
        if (!src->ghs_d_set)  D  = 10.0f;
        if (!src->ghs_sp_set) SP = 0.12f;
    } else if (src->format == FORMAT_Y12P) {
        src->lut_size = 4096;
        if (!src->ghs_b_set)  B  = -0.99f;
        if (!src->ghs_d_set)  D  = 70.0f;
        if (!src->ghs_sp_set) SP = 0.03f;
    }

    src->lut = (unsigned char*)g_malloc(src->lut_size);

    if (src->trans == TRANS_LIN) {

        // Keep the top 8 bits, which costs resolution at the faint end but stays linear
        int div = src->lut_size/256;
        for (int i = 0; i < src->lut_size; i++)
            src->lut[i] = (unsigned char)(i/(div > 0 ? div : 1));

    } else if (src->trans == TRANS_LSB) {

        // Keep the bottom 8 bits at full resolution and let everything brighter clip
        for (int i = 0; i < src->lut_size; i++)
            src->lut[i] = (unsigned char)(i < 256 ? i : 255);

    } else if (src->trans == TRANS_GAMMA) {

        // A plain power law, so RMS can undo it exactly by setting config.gamma to the same exponent
        for (int i = 0; i < src->lut_size; i++) {
            float x = normalise_input(i, src->lut_size, src->black_level);
            src->lut[i] = (unsigned char)lrintf(255.0f*powf(x, src->gamma));
        }

    } else if (src->trans == TRANS_GHS) {

        build_ghs_lut(src->lut, src->lut_size, B, D, SP, src->black_level);
    }

    /* Before the device: the sensor format and ROI decide the byte layout the capture side will see,
       so they cannot be changed once the stream is up. */
    if (!mv_setup_camera(src))
        return FALSE;

    /* The camera is now applying the curve, so the host's job is reduced to taking the top 8 bits -
       which for GRAY8 input is the identity, and for Y10P/Y12P is the plain division the `lin` mode
       does. Anything else would curve the frame twice. Rebuilt here rather than earlier because only
       the upload can say whether it succeeded. */
    if (src->curve_in_camera) {
        int div = src->lut_size/256;
        if (div < 1)
            div = 1;
        for (int i = 0; i < src->lut_size; i++)
            src->lut[i] = (unsigned char)MIN(i/div, 255);

        g_print("camera: curve is in the camera, so this element only takes the top 8 bits%s\n",
                (src->format == FORMAT_GRAY8) ? " (a straight copy)" : "");

        if (src->format != FORMAT_GRAY8)
            g_print("camera: set the camera to Mono8 and format=GRAY8 to also drop the unpacking "
                    "and halve the MIPI payload\n");
    }

    src->cap = new cv::VideoCapture();

    if (is_number(src->device)) {
        int index = atoi(src->device);
        g_print("Opening camera index: %d\n", index);
        src->cap->open(index, cv::CAP_V4L2);
    } else {
        g_print("Opening device path: %s\n", src->device);
        src->cap->open(src->device, cv::CAP_V4L2);
    }

    if (!src->cap->isOpened()) {
        g_printerr("ERROR! Unable to open camera\n");
        delete src->cap;
        src->cap = NULL;
        return FALSE;
    }

    src->cap->set(cv::CAP_PROP_FRAME_WIDTH, src->width);
    src->cap->set(cv::CAP_PROP_FRAME_HEIGHT, src->height);

    src->cap->set(cv::CAP_PROP_FPS, src->fps_n);
    src->cap->set(cv::CAP_PROP_CONVERT_RGB, 0);
    src->cap->set(cv::CAP_PROP_FORMAT, -1);

    int fourcc = 0;
    switch (src->format) {
        case FORMAT_GRAY8: fourcc = VideoWriter::fourcc('G','R','E','Y'); break;
        case FORMAT_Y10P:  fourcc = VideoWriter::fourcc('Y','1','0','P'); break;
        case FORMAT_Y12P:  fourcc = VideoWriter::fourcc('Y','1','2','P'); break;
    }
    src->cap->set(CAP_PROP_FOURCC, fourcc);

    /* Ask for a deeper queue in the driver. OpenCV's V4L2 backend caps this at MAX_V4L_BUFFERS, which
       is 4, and asks for a value above it only after having released the buffers it already had, which
       leaves the device with none and every read returning empty. Hence the hard clamp. */
    if (src->n_buffers > 0) {

        int want = src->n_buffers;

        if (want > OPENCV_MAX_V4L_BUFFERS) {
            g_warning("mipimvsrc: OpenCV allows at most %d driver buffers, asking for %d instead of %d",
                      OPENCV_MAX_V4L_BUFFERS, OPENCV_MAX_V4L_BUFFERS, want);
            want = OPENCV_MAX_V4L_BUFFERS;
        }

        if (!src->cap->set(cv::CAP_PROP_BUFFERSIZE, want))
            g_warning("mipimvsrc: driver kept its own buffer count, %d not applied", want);
    }

    /* With the curve in the camera the host table is the plain top-8-bits ramp, which is exactly what
       the fast path computes without a lookup. */
    src->high_byte_only = ((src->trans == TRANS_LIN) || src->curve_in_camera)
                          && (src->format != FORMAT_GRAY8);

    src->duration = gst_util_uint64_scale_int(GST_SECOND, src->fps_d, src->fps_n);
    src->frame_count = 0;
    src->first_driver_ms = -1.0;

    gst_base_src_set_format(basesrc, GST_FORMAT_TIME);
    return TRUE;
}


/* ======================== STOP ======================== */

static gboolean gst_mipimv_src_stop(GstBaseSrc *basesrc)
{
    GstMipiMvSrc *src = (GstMipiMvSrc *)basesrc;

    if (src->cap) {
        src->cap->release();
        delete src->cap;
        src->cap = NULL;
    }

    g_free(src->lut);
    src->lut = NULL;

    if (src->i2c) {

        /* What the camera itself lost. A gap here is not something host-side buffering can fix, and
           without this it is indistinguishable from the host being too slow. */
        guint32 sns = 0, out = 0;
        if (src->fc_valid && src->i2c->frame_counts(&sns, &out)) {
            guint32 produced = sns - src->fc_sensor0;
            guint32 sent = out - src->fc_out0;
            g_print("camera: produced %u frames, sent %u", produced, sent);
            if (produced > sent)
                g_print(", DROPPED %u inside the camera", produced - sent);
            g_print("\n");
        }

        uint32_t err = 0;
        if (src->i2c->error_code(&err) && err)
            g_printerr("camera: error code 0x%x at stop\n", err);

        delete src->i2c;
        src->i2c = NULL;
    }

    return TRUE;
}


/* GstBaseSrc calls these to get a blocked create() to come back. OpenCV's read() cannot be
   interrupted, so the wait still lasts until the driver hands over the frame it is holding, or until
   its own timeout. What this does guarantee is that no further read is started. */
static gboolean gst_mipimv_src_unlock(GstBaseSrc *basesrc)
{
    ((GstMipiMvSrc *)basesrc)->unlocked = TRUE;
    return TRUE;
}

static gboolean gst_mipimv_src_unlock_stop(GstBaseSrc *basesrc)
{
    ((GstMipiMvSrc *)basesrc)->unlocked = FALSE;
    return TRUE;
}


/* ======================== convertToGray ======================== */

/* Useful (unpadded) bytes a packed row of this width occupies. */
static int packed_line_bytes(int width, MvFormat fmt)
{
    switch (fmt) {
        case FORMAT_Y10P: return width*5/4;   // 4 pixels per 5 bytes
        case FORMAT_Y12P: return width*3/2;   // 2 pixels per 3 bytes
        default:          return width;       // GRAY8
    }
}


/* Unpacks a span of rows and maps them through the LUT, writing straight into the caller's buffer.

   Rows are handled one at a time rather than as one flat run because drivers pad them: a 2400 pixel
   Y10P row is 3000 bytes of data in a 3008 byte stride, and walking straight through would fold those
   8 bytes into the next row and shear the image. */
static void convertRows(const uchar *in, int stride, int line_bytes, int rows, int width,
                        uchar *ou, MvFormat fmt, const unsigned char *lut, gboolean high_byte_only)
{
    for (int r = 0; r < rows; r++) {

        const uchar *p = in + (size_t)r*stride;
        uchar *o = ou + (size_t)r*width;
        int k = 0, t;

        /* Keeping the top 8 bits means the answer is the packed high byte itself, so the bit assembly
           and the table lookup can both be skipped. Measured 2.6x faster than going through the LUT. */
        if (high_byte_only) {

            if (fmt == FORMAT_Y10P) {
                for (int j = 0; j + 4 < line_bytes; j += 5) {
                    o[k+0] = p[j+0];
                    o[k+1] = p[j+1];
                    o[k+2] = p[j+2];
                    o[k+3] = p[j+3];
                    k += 4;
                }
                continue;
            }

            if (fmt == FORMAT_Y12P) {
                for (int j = 0; j + 2 < line_bytes; j += 3) {
                    o[k+0] = p[j+0];
                    o[k+1] = p[j+1];
                    k += 2;
                }
                continue;
            }
        }

        if (fmt == FORMAT_GRAY8) {

            for (int j = 0; j < line_bytes; j++)
                o[j] = lut[p[j]];

        } else if (fmt == FORMAT_Y10P) {

            // 4 pixels packed into 5 bytes: four high bytes, then their low bits gathered in the fifth
            for (int j = 0; j + 4 < line_bytes; j += 5) {
                t = (p[j+0] << 2) | ((p[j+4] & 0b11000000) >> 6);
                o[k+0] = lut[t];
                t = (p[j+1] << 2) | ((p[j+4] & 0b00110000) >> 4);
                o[k+1] = lut[t];
                t = (p[j+2] << 2) | ((p[j+4] & 0b00001100) >> 2);
                o[k+2] = lut[t];
                t = (p[j+3] << 2) | (p[j+4] & 0b00000011);
                o[k+3] = lut[t];
                k += 4;
            }

        } else if (fmt == FORMAT_Y12P) {

            // 2 pixels packed into 3 bytes: two high bytes, then a nibble each in the third
            for (int j = 0; j + 2 < line_bytes; j += 3) {
                t = (p[j+0] << 4) | (p[j+2] >> 4);
                o[k+0] = lut[t];
                t = (p[j+1] << 4) | (p[j+2] & 0b00001111);
                o[k+1] = lut[t];
                k += 2;
            }
        }
    }
}


/* The same, to the camera's own values right aligned in 16 bits, with no curve applied. Nothing is
   lost here, which is the point: this path exists for recording and analysis, not for viewing. */
static void convertRows16(const uchar *in, int stride, int line_bytes, int rows, int width,
                          guint16 *ou, MvFormat fmt)
{
    for (int r = 0; r < rows; r++) {

        const uchar *p = in + (size_t)r*stride;
        guint16 *o = ou + (size_t)r*width;
        int k = 0;

        if (fmt == FORMAT_GRAY8) {
            for (int j = 0; j < line_bytes; j++)
                o[j] = p[j];

        } else if (fmt == FORMAT_Y10P) {
            for (int j = 0; j + 4 < line_bytes; j += 5) {
                o[k+0] = (guint16)((p[j+0] << 2) | ((p[j+4] & 0b11000000) >> 6));
                o[k+1] = (guint16)((p[j+1] << 2) | ((p[j+4] & 0b00110000) >> 4));
                o[k+2] = (guint16)((p[j+2] << 2) | ((p[j+4] & 0b00001100) >> 2));
                o[k+3] = (guint16)((p[j+3] << 2) | (p[j+4] & 0b00000011));
                k += 4;
            }

        } else if (fmt == FORMAT_Y12P) {
            for (int j = 0; j + 2 < line_bytes; j += 3) {
                o[k+0] = (guint16)((p[j+0] << 4) | (p[j+2] >> 4));
                o[k+1] = (guint16)((p[j+1] << 4) | (p[j+2] & 0b00001111));
                k += 2;
            }
        }
    }
}


/* Splits the frame across threads by rows, which keeps each one on its own whole lines and so needs
   no locking. Measured a little over 2x on two cores, which is worth more than anything left in the
   inner loop. */
static void convertToGray(const Mat& inp, uchar *ou, MvFormat fmt, const unsigned char *lut,
                          gboolean high_byte_only, int n_threads, MvOut out_format,
                          int width, int height)
{
    cv::Mat input = inp.isContinuous() ? inp : inp.clone();

    const uchar *in = input.ptr<uchar>(0);
    const int line_bytes = packed_line_bytes(width, fmt);

    /* The driver's row stride, which is not always the packed row length: Y10P at 2400 pixels is
       3000 bytes of data inside a 3008 byte row. Derived from the frame, since OpenCV hands the whole
       buffer over as one flat run and the padding would otherwise be read as pixels. */
    const int total = (int)(input.total()*input.elemSize());
    const int stride = (height > 0) ? total/height : line_bytes;

    if (n_threads > height)
        n_threads = height;

    if (n_threads < 2) {
        if (out_format == OUT_GRAY16_LE)
            convertRows16(in, stride, line_bytes, height, width, (guint16*)ou, fmt);
        else
            convertRows(in, stride, line_bytes, height, width, ou, fmt, lut, high_byte_only);
        return;
    }

    const int per_thread = height/n_threads;

    std::vector<std::thread> workers;
    workers.reserve(n_threads - 1);

    for (int t = 0; t < n_threads; t++) {

        const int r0 = t*per_thread;
        const int rows = (t == n_threads - 1) ? height - r0 : per_thread;

        const uchar *chunk_in = in + (size_t)r0*stride;

        // The calling thread takes the last chunk rather than sitting idle
        if (out_format == OUT_GRAY16_LE) {

            guint16 *chunk_out = (guint16*)ou + (size_t)r0*width;

            if (t == n_threads - 1)
                convertRows16(chunk_in, stride, line_bytes, rows, width, chunk_out, fmt);
            else
                workers.emplace_back(convertRows16, chunk_in, stride, line_bytes, rows, width,
                                     chunk_out, fmt);

        } else {

            uchar *chunk_out = ou + (size_t)r0*width;

            if (t == n_threads - 1)
                convertRows(chunk_in, stride, line_bytes, rows, width, chunk_out, fmt, lut,
                            high_byte_only);
            else
                workers.emplace_back(convertRows, chunk_in, stride, line_bytes, rows, width,
                                     chunk_out, fmt, lut, high_byte_only);
        }
    }

    for (auto &w : workers)
        w.join();
}


/* ======================== CREATE ======================== */

static GstFlowReturn gst_mipimv_src_create(GstPushSrc *pushsrc, GstBuffer **buf)
{
    GstMipiMvSrc *src = (GstMipiMvSrc *)pushsrc;

    if (!src->cap || !src->lut)
        return GST_FLOW_ERROR;

    if (src->unlocked)
        return GST_FLOW_FLUSHING;

    if (!src->cap->read(src->frame) || src->frame.empty()) {

        // A read that comes back empty because the pipeline is stopping is not an error
        if (src->unlocked)
            return GST_FLOW_FLUSHING;

        g_printerr("mipimvsrc: empty frame\n");
        return GST_FLOW_ERROR;
    }

    if (src->unlocked)
        return GST_FLOW_FLUSHING;

    /* Timestamp before unpacking. Anything measured afterwards carries the conversion time as jitter,
       and the driver's own stamp avoids even that, since the kernel takes it when the frame lands. */
    GstClockTime pts = GST_CLOCK_TIME_NONE;

    if (src->stamps == STAMP_DRIVER) {

        const double ms = src->cap->get(cv::CAP_PROP_POS_MSEC);

        // Some drivers leave this at zero, in which case there is nothing to use
        if (ms > 0.0) {

            if (src->first_driver_ms < 0.0)
                src->first_driver_ms = ms;

            const double rel_ms = ms - src->first_driver_ms;

            if (rel_ms >= 0.0)
                pts = (GstClockTime)(rel_ms*GST_MSECOND);

        } else if (src->frame_count == 0) {
            g_warning("mipimvsrc: driver reports no capture time, falling back to the frame counter. "
                      "Set stamps=clock if that suits the pipeline better.");
        }

    } else if (src->stamps == STAMP_CLOCK) {

        GstClock *clock = gst_element_get_clock(GST_ELEMENT(src));

        if (clock) {
            GstClockTime now = gst_clock_get_time(clock);
            GstClockTime base = gst_element_get_base_time(GST_ELEMENT(src));

            if (GST_CLOCK_TIME_IS_VALID(now) && GST_CLOCK_TIME_IS_VALID(base) && now >= base)
                pts = now - base;

            gst_object_unref(clock);
        }
    }

    if (!GST_CLOCK_TIME_IS_VALID(pts))
        pts = src->frame_count*src->duration;

    src->frame_count++;

    /* Unpack straight into the downstream buffer. GStreamer pools these, so there is no allocation
       per frame, and nothing outlives the buffer that carries it. */
    const gsize expected = (gsize)src->width*src->height;
    const gsize bpp = (src->out_format == OUT_GRAY16_LE) ? 2 : 1;

    *buf = gst_buffer_new_allocate(NULL, expected*bpp, NULL);
    if (*buf == NULL)
        return GST_FLOW_ERROR;

    GstMapInfo map;
    if (!gst_buffer_map(*buf, &map, GST_MAP_WRITE)) {
        gst_buffer_unref(*buf);
        *buf = NULL;
        return GST_FLOW_ERROR;
    }

    const gsize got = src->frame.total()*src->frame.elemSize();
    const gsize need = (gsize)packed_line_bytes(src->width, src->format)*src->height;

    if (got < need) {
        g_printerr("mipimvsrc: frame is %" G_GSIZE_FORMAT " bytes, need at least %" G_GSIZE_FORMAT
                   " for %dx%d. Check format and resolution.\n",
                   got, need, src->width, src->height);
        gst_buffer_unmap(*buf, &map);
        gst_buffer_unref(*buf);
        *buf = NULL;
        return GST_FLOW_ERROR;
    }

    convertToGray(src->frame, map.data, src->format, src->lut, src->high_byte_only, src->n_threads,
                  src->out_format, src->width, src->height);

    gst_buffer_unmap(*buf, &map);

    GST_BUFFER_PTS(*buf) = pts;
    GST_BUFFER_DTS(*buf) = pts;
    GST_BUFFER_DURATION(*buf) = src->duration;

    return GST_FLOW_OK;
}

/* ======================== INIT ======================== */
static GstStaticPadTemplate src_template =
GST_STATIC_PAD_TEMPLATE(
    "src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS("video/x-raw, format=(string){ GRAY8, GRAY16_LE }")
);

static void gst_mipimv_src_finalize(GObject *object)
{
    GstMipiMvSrc *src = (GstMipiMvSrc *)object;

    g_free(src->device);
    g_free(src->lut);

    G_OBJECT_CLASS(gst_mipimv_src_parent_class)->finalize(object);
}

static void gst_mipimv_src_class_init(GstMipiMvSrcClass *klass)
{
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
    GstBaseSrcClass *basesrc_class = GST_BASE_SRC_CLASS(klass);
    GstPushSrcClass *pushsrc_class = GST_PUSH_SRC_CLASS(klass);

    GstElementClass *element_class = GST_ELEMENT_CLASS(klass);

    gst_element_class_set_metadata(
        element_class,
        "MIPI Machine Vision (grayscale 8-12 bit) Camera",
        "Source/Video",
        "MIPI MV video source",
        "sanlu <sanlu@pkim.org>"
    );

    gst_element_class_add_static_pad_template(element_class, &src_template);

    gobject_class->set_property = gst_mipimv_src_set_property;
    gobject_class->get_property = gst_mipimv_src_get_property;
    gobject_class->finalize = gst_mipimv_src_finalize;

    g_object_class_install_property(gobject_class, PROP_DEVICE,
        g_param_spec_string(
            "device",
            "Device",
            "Device path (e.g. /dev/video0, 0 (just index))",
            "/dev/video0",
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)
    ));
    g_object_class_install_property(gobject_class, PROP_WIDTH,
        g_param_spec_int("width", "Width", "Frame width",
                         1, 4096, 1920, G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_HEIGHT,
        g_param_spec_int("height", "Height", "Frame height",
                         1, 4096, 1080, G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_FRAMERATE,
        gst_param_spec_fraction("framerate", "Framerate", "Frames per second",
                                1, 1, 120, 1, 30, 1, G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_FORMAT,
        g_param_spec_string("format", "Format", "Input pixel format: GRAY8, Y10P or Y12P",
                            "GRAY8", G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_TRANS,
        g_param_spec_string(
            "trans",
            "Conversion type to GRAY8",
            "How the camera's bit depth is mapped onto 8 bits:\n"
"                        - lin   (top 8 bits, linear, loses the faint end),\n"
"                        - lsb   (bottom 8 bits at full resolution, everything brighter clips),\n"
"                        - gamma (power law, exponent set by the gamma property; the only mode RMS\n"
"                                 can invert, so the only one that keeps photometry usable),\n"
"                        - ghs   (Generalized Hyperbolic Stretch, as in Siril; best looking, but\n"
"                                 not invertible by RMS, so star intensities come out wrong)",
            "lin",
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)
    ));
    g_object_class_install_property(gobject_class, PROP_GHS_B,
        g_param_spec_float("ghs-b", "GHS b",
                           "GHS shape. Defaults to -0.99 for Y10P and Y12P",
                           -10.0f, 10.0f, -0.99f,
                           (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
    g_object_class_install_property(gobject_class, PROP_GHS_D,
        g_param_spec_float("ghs-d", "GHS D",
                           "GHS stretch amount. Defaults to 10 for Y10P and 70 for Y12P",
                           0.0f, 1000.0f, 70.0f,
                           (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
    g_object_class_install_property(gobject_class, PROP_GHS_SP,
        g_param_spec_float("ghs-sp", "GHS SP",
                           "GHS symmetry point, as a fraction of full scale. Put it near the sky "
                           "background. Defaults to 0.12 for Y10P and 0.03 for Y12P",
                           0.0f, 1.0f, 0.03f,
                           (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
    g_object_class_install_property(gobject_class, PROP_GAMMA,
        g_param_spec_float("gamma", "Gamma",
                           "Exponent of the gamma transform. Put the same value in the RMS config "
                           "as gamma, and RMS will undo it before measuring star intensities",
                           0.05f, 1.0f, 0.45f,
                           (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_OUT_FORMAT,
        g_param_spec_string("out-format", "Output format",
            "What this element puts on its pad:\n"
"                        - GRAY8     (8 bit, through the trans curve),\n"
"                        - GRAY16_LE (the camera's own 10 or 12 bit values, right aligned, no curve\n"
"                                     and nothing lost. RMS cannot read this yet)",
            "GRAY8",
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_STAMPS,
        g_param_spec_string("stamps", "Timestamp source",
            "Where buffer timestamps come from:\n"
"                        - driver (the kernel's own capture time, as v4l2src uses. Unaffected by how\n"
"                                  long this element waits for CPU, so it neither drifts nor jitters),\n"
"                        - clock  (pipeline clock when the frame reaches this element: correct on\n"
"                                  average, but carries any scheduling delay as jitter),\n"
"                        - frames (a counter: perfectly smooth, but drifts whenever the real frame\n"
"                                  rate differs from the nominal one, and hides lost frames)",
            "driver",
            (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_BLACK_LEVEL,
        g_param_spec_int("black-level", "Black level",
                         "Sensor offset in the camera's own units, subtracted before the gamma or ghs "
                         "curve. Most of every pixel value is offset and it does not scale with gain, "
                         "so leaving it in makes the curve lift the offset instead of the signal. "
                         "Read it off a corner of the frame outside the lens circle. 0 disables",
                         0, 65535, 0,
                         (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    g_object_class_install_property(gobject_class, PROP_BUFFERS,
        g_param_spec_int("buffers", "Driver buffers",
                         "How many buffers the driver queues, at most 10 because that is OpenCV's own "
                         "limit. Its default of 4 covers about 160 ms at 25 fps, which is not enough "
                         "to ride out the stall when a block of frames is handed on for compression. "
                         "0 leaves the driver's own choice alone",
                         0, 10, 0,
                         (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    /* ---- camera control over I2C ----
       Off unless i2c-bus is given, so pipelines that rely on set_params.sh are unaffected. */
    g_object_class_install_property(gobject_class, PROP_I2C_BUS,
        g_param_spec_int("i2c-bus", "I2C bus",
            "Bus number in /dev/i2c-N carrying the camera's control block. "
            "-1 leaves the camera entirely alone, which is the default. "
            "-2 works it out: it walks the media graph to find which camera feeds this device, which "
            "stays correct with two cameras on one board, and falls back to looking for a single "
            "camera anywhere. Best effort - if none is found, or the bus cannot be opened, capture "
            "carries on without camera control. An explicit number is taken as a request, so failing "
            "to reach it is an error.",
            -2, 255, -1, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_I2C_ADDR,
        g_param_spec_int("i2c-addr", "I2C address",
            "I2C address of the control block. 0x3b on every MV camera seen so far.",
            0, 127, MVCAM_I2C_ADDR, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_PROBE,
        g_param_spec_boolean("probe", "Probe the camera",
            "Log model, serial, firmware, sensor size, limits, temperature and the defect pixel "
            "table size at start.",
            FALSE, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CAM_FPS,
        g_param_spec_double("cam-fps", "Sensor frame rate",
            "Frame rate to program into the camera. -1 leaves it alone, 0 takes it from the framerate "
            "property. Applied at start only. The driver also has a working frame_rate control, which "
            "is the tidier layer but is limited to whole frames; this writes the register directly and "
            "takes hundredths. The only cost is that the driver's sysfs fps file stays stale until "
            "something reads the register again.",
            -1.0, 1000.0, -1.0, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_EXPOSURE,
        g_param_spec_int("exposure", "Exposure time",
            "Manual exposure in microseconds, -1 to leave alone. Can be changed while running. "
            "Set cam-auto-exposure=0 as well, or the auto loop overrides it. Written over I2C because "
            "the driver's V4L2_CID_EXPOSURE only stores the value and never reaches the camera.",
            -1, 100000000, -1, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_GAIN,
        g_param_spec_double("gain", "Analog gain",
            "Manual gain in dB, -1 to leave alone. Can be changed while running. Set cam-auto-gain=0 "
            "as well, or the auto loop overrides it. Written over I2C for the same reason as exposure: "
            "the driver's V4L2_CID_ANALOGUE_GAIN is a stub.",
            -1.0, 100.0, -1.0, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CAM_BLACK_LEVEL,
        g_param_spec_int("cam-black-level", "Sensor black level",
            "Black level in the camera, -1 to leave alone. This is the sensor's own offset and is a "
            "different thing from the black-level property, which only shifts the software curve.",
            -1, 65535, -1, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CAM_GAMMA,
        g_param_spec_double("cam-gamma", "Sensor gamma",
            "Gamma applied inside the camera. -1 leaves it alone, 0 turns the curve off, a positive "
            "value sets and enables it. Unrelated to the gamma property, which is the software curve "
            "this element applies when mapping down to 8 bits.",
            -1.0, 10.0, -1.0, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CAM_DPC,
        g_param_spec_int("cam-dpc", "Defect pixel correction",
            "Correction of defective pixels in the camera: -1 leave, 0 off, 1 on. Worth trying on a "
            "sensor with many hot pixels, since it removes them before they ever reach the host.",
            -1, 1, -1, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CAM_IMGDIR,
        g_param_spec_int("cam-imgdir", "Image direction",
            "Orientation: -1 leave, 0 normal, 1 mirror, 2 flip, 3 both.",
            -1, 3, -1, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CAM_TESTIMG,
        g_param_spec_int("cam-testimg", "Test pattern",
            "Built-in test image: -1 leave, 0 off, 1 and 2 are vertical bar patterns. Useful for "
            "telling a transport problem apart from a sensor one.",
            -1, 15, -1, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CAM_AUTO_EXPOSURE,
        g_param_spec_int("cam-auto-exposure", "Exposure mode",
            "-1 leave, 0 manual, 1 auto once, 2 auto continuous. Use 0 for a meteor station: a "
            "changing exposure makes photometry between frames incomparable.",
            -1, 2, -1, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CAM_AUTO_GAIN,
        g_param_spec_int("cam-auto-gain", "Gain mode",
            "-1 leave, 0 manual, 1 auto once, 2 auto continuous. Use 0 for a meteor station, for the "
            "same reason as cam-auto-exposure.",
            -1, 2, -1, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CAM_LUT,
        g_param_spec_int("cam-lut", "User LUT",
            "The camera's user lookup table: -1 leave, 0 off, 1 on. Worth setting to 0 on a station "
            "that cares about photometry - a LUT left enabled rewrites every pixel on top of gamma, "
            "and nothing on the host side can tell that it happened. probe=true reports its state.",
            -1, 1, -1, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CAM_BLC,
        g_param_spec_int("cam-blc", "Black level correction mode",
            "Mode of the camera's black level correction, -1 to leave alone. Whether a mode other than "
            "the default helps depends on the sensor; read it back with probe=true first.",
            -1, 255, -1, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_CAM_CURVE,
        g_param_spec_boolean("cam-curve", "Move the curve into the camera",
            "Build this element's transfer curve - from trans, gamma, black-level and the ghs-* "
            "parameters - at the camera's own LUT resolution, upload it, and let the camera apply it. "
            "The LUT works on the sensor's 12 bit values before they are rounded to the output format, "
            "so with the camera set to Mono8 and format=GRAY8 the frame arrives already curved: no "
            "unpacking, no table lookup, and half the MIPI payload. Set trans and gamma as usual; they "
            "describe the curve either way. Says so loudly if format is not GRAY8, because then the "
            "host curves the frame a second time.",
            FALSE, G_PARAM_READWRITE));

    g_object_class_install_property(gobject_class, PROP_TEMPERATURE,
        g_param_spec_double("temperature", "Sensor temperature",
            "Sensor temperature in degrees Celsius, read from the camera on each get. -1000 when "
            "camera control is not active. Read only. Worth logging over a night: dark current and the "
            "amplitude of the hot pixel pattern both move with it.",
            -1000.0, 200.0, -1000.0, (GParamFlags)(G_PARAM_READABLE)));

    g_object_class_install_property(gobject_class, PROP_THREADS,
        g_param_spec_int("threads", "Threads",
                         "How many threads unpack the frame. The work splits cleanly, so this scales "
                         "almost linearly up to the number of cores left over from the rest of the "
                         "pipeline. 1 keeps everything on the streaming thread",
                         1, 16, 1,
                         (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

    basesrc_class->start = gst_mipimv_src_start;
    basesrc_class->stop = gst_mipimv_src_stop;
    basesrc_class->unlock = gst_mipimv_src_unlock;
    basesrc_class->unlock_stop = gst_mipimv_src_unlock_stop;
    basesrc_class->get_caps = gst_mipimv_src_get_caps;
    pushsrc_class->create = gst_mipimv_src_create;
}

static void gst_mipimv_src_init(GstMipiMvSrc *src)
{
    src->cap = NULL;
    src->lut = NULL;
    src->device = g_strdup("/dev/video0");
    src->width = 1920;
    src->height = 1080;
    src->fps_n = 30;
    src->fps_d = 1;
    src->format = FORMAT_GRAY8;
    src->trans = TRANS_LIN;
    src->out_format = OUT_GRAY8;
    src->unlocked = FALSE;

    src->ghs_b = -0.99f;
    src->ghs_d = 70.0f;
    src->ghs_sp = 0.03f;
    src->ghs_b_set = FALSE;
    src->ghs_d_set = FALSE;
    src->ghs_sp_set = FALSE;

    src->gamma = 0.45f;
    src->high_byte_only = FALSE;
    src->n_threads = 1;
    src->n_buffers = 0;
    src->black_level = 0;
    src->stamps = STAMP_DRIVER;
    src->first_driver_ms = -1.0;
    src->frame_count = 0;

    /* Camera control off by default: a bus of -1 means this element never touches I2C, so existing
       pipelines keep working exactly as before and set_params.sh stays in charge. */
    src->i2c = NULL;
    src->i2c_bus = -1;
    src->i2c_addr = MVCAM_I2C_ADDR;
    src->probe = FALSE;
    src->cam_fps = -1.0;
    src->cam_exposure_us = -1;
    src->cam_gain_db = -1.0;
    src->cam_black_level = -1;
    src->cam_gamma = -1.0;
    src->cam_dpc = -1;
    src->cam_imgdir = -1;
    src->cam_testimg = -1;
    src->cam_auto_exposure = -1;
    src->cam_auto_gain = -1;
    src->cam_lut = -1;
    src->cam_blc = -1;
    src->cam_curve = FALSE;
    src->curve_in_camera = FALSE;
    src->fc_valid = FALSE;

    gst_base_src_set_live(GST_BASE_SRC(src), TRUE);
}

/* ======================== PLUGIN ======================== */

static gboolean plugin_init(GstPlugin *plugin)
{
    gboolean ok = gst_element_register(plugin, "mipimvsrc", GST_RANK_NONE, gst_mipimv_src_get_type());
    return ok;
}


GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    mipimvsrc,
    "MIPI Machine Vision Camera plugin",
    plugin_init,
    "1.0",
    "GPL",
    "mipimvsrc",
    "mipimvsrc"
)
