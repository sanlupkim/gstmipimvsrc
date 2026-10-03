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
    PROP_THREADS
};

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
static void build_ghs_lut(unsigned char *lut, int lut_size, float B, float D, float SP, int black)
{
    const float LP = 0.0f, HP = 1.0f;

    B = -B;

    // B == 1 puts a zero in every denominator below, and a flat curve carries no information anyway
    if (fabsf(B - 1.0f) < 1e-6f || D <= 0.0f) {
        g_warning("mipimvsrc: GHS parameters degenerate (b=%.3f d=%.3f), falling back to linear", -B, D);
        int div = lut_size/256;
        for (int i = 0; i < lut_size; i++)
            lut[i] = (unsigned char)(i/(div > 0 ? div : 1));
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
            lut[i] = (unsigned char)(i/(div > 0 ? div : 1));
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

        lut[i] = (unsigned char)lrintf(255.0f*CLAMP(y, 0.0f, 1.0f));
    }
}


/* ======================== START ======================== */
// gst-launch-1.0 mipimvsrc format=Y12P framerate=25/1 width=1920 height=1080 ! videoconvert ! filesink location=/tmp/mp2.raw
//
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

    src->high_byte_only = (src->trans == TRANS_LIN) && (src->format != FORMAT_GRAY8);

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
    "LGPL",
    "mipimvsrc",
    "mipimvsrc"
)
