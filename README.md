# mipimvsrc

A GStreamer source for MIPI machine vision cameras that deliver more than 8 bits per pixel, written
for use with [RMS](https://github.com/CroatianMeteorNetwork/RMS).

RMS works in 8 bits throughout: its compressor takes `uint8` frames and the FF format stores 8 bit
layers. A 10 or 12 bit sensor therefore has to be mapped down somewhere, and where that mapping
happens decides what survives. Letting the camera do it in its own GRAY8 mode saturates bright
objects early, which costs exactly the measurements a meteor station is there to make. This element
does the mapping instead, under control of a transfer curve.

## Why the extra bits are worth unpacking

Measured on an IMX178 mono at 40 ms exposure, full frame 2400x2064, sky background above the black
level of 199:

| gain | sky signal | temporal noise | saturated pixels (12 bit) |
|------|-----------:|---------------:|--------------------------:|
| 0 dB | 0 ADU | 0.88 | 0 |
| 20 dB | 4 ADU | 7.12 | 1 |
| 40 dB | 28 ADU | 49.2 | 11 |

The same frames through the camera's own 8 bit mode saturate over two thousand pixels. The sensor is
not the limit; the conversion is. At 40 dB the noise spans about 6 bits, so the low bits of a 12 bit
word carry nothing and there is no point storing them: the gain is in choosing *where* the 256 output
levels land.

## Building

```sh
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libopencv-dev
make
make install          # into ~/.local/share/gstreamer-1.0/plugins
gst-inspect-1.0 mipimvsrc
```

Keep `-O3`. Without it the unpacking loop is 3.2x slower, which on a Raspberry Pi 5 is the difference
between fitting in the frame budget and dropping frames.

## Properties

| property | default | what it does |
|---|---|---|
| `device` | `/dev/video0` | Device path, or a bare number for an index |
| `width`, `height` | 1920x1080 | Frame size, after any ROI set on the driver |
| `framerate` | 30/1 | Only used for buffer duration and the `frames` stamp mode |
| `format` | `GRAY8` | Input pixel format: `GRAY8`, `Y10P` or `Y12P` |
| `out-format` | `GRAY8` | What goes on the pad: `GRAY8` or `GRAY16_LE` |
| `trans` | `lin` | Transfer curve, see below |
| `gamma` | 0.45 | Exponent for `trans=gamma` |
| `black-level` | 0 | Sensor offset, subtracted before the curve. 0 disables |
| `ghs-b`, `ghs-d`, `ghs-sp` | per format | Shape of the GHS curve |
| `stamps` | `driver` | Where timestamps come from, see below |
| `threads` | 1 | Threads used for unpacking |
| `buffers` | 0 | Driver queue depth, at most 10. 0 leaves the driver's own choice |

### Transfer curves

`lin` keeps the top 8 bits. Linear, and by far the cheapest, since the answer is the packed high byte
and neither the bit assembly nor the table lookup is needed. It throws away the faint end.

`lsb` keeps the bottom 8 bits at full resolution and clips everything brighter.

`gamma` applies `255*x^gamma` after subtracting the black level. **This is the only curve RMS can
undo**: put the same exponent in the RMS config as `gamma` and star intensities come out right. Use
this one unless photometry does not matter.

`ghs` is the Generalized Hyperbolic Stretch used by Siril. It resolves the faint end better than
gamma but RMS cannot invert it, so magnitudes come out wrong.

How the 256 output levels get shared out, measured on a real frame at 40 dB:

| curve | background (±3σ) | faint stars | bright events |
|---|---:|---:|---:|
| `gamma 0.7` | 30 | 15 | **213** |
| `gamma 0.45` | 56 | 19 | 175 |
| `ghs` | **89** | **37** | 130 |

Read the black level off a corner of the frame outside the lens circle, in the camera's own units.
Leaving it at 0 wastes the curve: the sensor offset is most of every pixel value and does not scale
with gain, so the steepest part of the curve ends up lifting the offset rather than the signal.

### 16 bit output

`out-format=GRAY16_LE` puts the camera's own values on the pad, right aligned in 16 bits, with no
curve and nothing discarded. `trans`, `gamma` and `black-level` are ignored in this mode: there is no
mapping to choose when nothing is being thrown away.

Right aligned means a 12 bit camera produces 0 to 4095 rather than filling the 16 bit range, so the
numbers stay in the camera's own units and match what `v4l2-ctl` would capture. Frames are twice the
size, 9.9 MB at 2400x2064, so about 250 MB/s at 25 fps.

RMS cannot consume this yet; its compressor takes `uint8` and `gst_colorspace` knows only `GRAY8` and
`BGR`. Until that changes the use for it is a second branch off a `tee`, recording lossless data
while the 8 bit branch feeds RMS:

```
mipimvsrc ... ! tee name=t \
  t. ! queue ! video/x-raw,format=GRAY16_LE ! ... recording ... \
  t. ! queue ! appsink name=appsink
```

Note that a `tee` cannot give the two branches different formats, so this needs two elements or a
conversion; the simplest arrangement today is a second pipeline for the occasional lossless capture.

### Timestamps

`driver` takes the kernel's own capture time. Unaffected by how long this element waits to be
scheduled, so it neither drifts nor jitters. Use it.

`clock` reads the pipeline clock when the frame arrives here. Correct on average, but any scheduling
delay shows up as jitter, and RMS resets its PTS smoothing whenever a frame lands more than one
interval late.

`frames` counts frames. Perfectly smooth, but drifts whenever the real rate differs from the nominal
one, and hides lost frames instead of reporting them.

### Performance

Per frame at 2400x2064 Y12P, measured with `-O3`:

| | ms |
|---|---:|
| unpacking and curve, one thread | 5.7 |
| the same, two threads | 3.0 |
| `trans=lin` fast path | 2.4 |
| pure memory movement (the floor) | 0.8 |

The table lookup is 69% of that and cannot be vectorised: each lookup depends on the value just
assembled, so the loads cannot be run ahead. Splitting unpacking and lookup into separate passes to
allow SIMD measured *slower* (8.8 ms), as the intermediate buffer costs more than the vectorisation
saves. Threads are the only lever that does not trade away precision.

`buffers` matters more than it looks. The driver's default of 4 covers about 160 ms at 25 fps, which
is not long enough to ride out the pause when RMS hands a block of 256 frames to its compressor, and
frames are lost once per block. Raising it to 10, OpenCV's own ceiling, stopped that. Asking for more
than 10 leaves the device with no buffers at all and every read returns empty.

## Use with RMS

RMS parses the device string itself when it names its sink `name=appsink`, rather than handing it to
OpenCV, which is what makes its GStreamer timestamping and raw video saving available:

```
device: mipimvsrc device=/dev/video8 format=Y12P trans=gamma gamma=0.7 black-level=199 \
    stamps=driver threads=2 buffers=10 width=2400 height=2064 framerate=25/1 \
  ! video/x-raw,format=GRAY8,width=2400,height=2064,framerate=25/1 \
  ! queue max-size-buffers=20 max-size-bytes=0 max-size-time=0 \
  ! appsink max-buffers=100 drop=true sync=0 name=appsink
```

with, in the same config:

```
media_backend: gst
gst_colorspace: GRAY8
width: 2400
height: 2064
gamma: 0.7
```

`gamma` has to match the element's, or photometry will be wrong.

## Known limitations

- `unlock()` stops a further read being started, but cannot interrupt one already in progress, since
  OpenCV offers no way to do so. A shutdown still waits for the frame the driver is holding, or for
  its own timeout if the camera has stopped delivering.
- Camera controls such as exposure and gain are not exposed; set them with `v4l2-ctl`.
- The driver queue cannot go above 10 buffers, which is OpenCV's limit, not the driver's.
- No SER or FITS writing. GStreamer has no SER support either, though the format is simple enough
  that a sink would be short work once 16 bit output is being recorded.
