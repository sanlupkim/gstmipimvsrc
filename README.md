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

### If GStreamer says `no element "mipimvsrc"`

The plugin was not loaded. In order of likelihood:

```sh
make                      # did the link actually succeed? read the output, do not assume
make install              # installs and clears the registry cache
gst-inspect-1.0 mipimvsrc # names the reason if the file is there but unloadable
```

`gst-inspect-1.0` finding nothing while the file exists usually means it failed to load and has been
blacklisted. This says why:

```sh
GST_DEBUG=GST_PLUGIN_LOADING:5 gst-inspect-1.0 mipimvsrc 2>&1 | grep -i mipimv
gst-inspect-1.0 -b            # lists blacklisted plugins
```

A plugin can also be shadowed by an older copy: `gst-inspect-1.0 mipimvsrc | grep Filename` says which
file is actually in use, and `GST_PLUGIN_PATH` can be pointing somewhere else entirely.

## Properties

| property | default | what it does |
|---|---|---|
| `device` | `/dev/video0` | Device path, or a bare number for an index |
| `width`, `height` | from the camera | Frame size. Read from the camera's ROI when `i2c-bus` is set, so normally leave both out |
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

### Camera control

These are off unless `i2c-bus` is set, so existing pipelines are unaffected.

With it set, the element also reads the frame size from the camera's ROI, so `width` and `height` can
be left out of the pipeline. That is the better way round: the driver writes the ROI registers from the
media-ctl format, so it owns the size, and this element cannot crop - a `width` that disagrees with the
ROI does not give a smaller picture, it gives a sheared one. Give them explicitly only if you want that
disagreement reported rather than corrected.

| property | default | what it does |
|---|---|---|
| `i2c-bus` | -1 | -1 touches nothing, -2 finds the bus itself, or give the number |
| `i2c-addr` | 0x3b | Address of the control block |
| `probe` | false | Log model, serial, firmware, limits, temperature at start. Names the registers this firmware does not implement instead of printing their all-ones value as a number |
| `exposure` | -1 | Exposure in microseconds |
| `gain` | -1 | Gain in dB |
| `cam-auto-exposure` | -1 | 0 manual, 1 auto once, 2 auto continuous |
| `cam-auto-gain` | -1 | Same values |
| `cam-ae-target` | -1 | Mean level the auto loop aims at, 1..255 |
| `cam-ae-max-time` | -1 | Longest exposure the auto loop may use, µs |
| `cam-ag-max-gain` | -1 | Highest gain the auto loop may use, dB |
| `cam-ae-roi` | — | `x,y,w,h` for the metering window, or `off` for the whole frame |
| `cam-fps` | -1 | Frame rate; 0 takes it from `framerate`. Start only |
| `cam-black-level` | -1 | The sensor's own offset |
| `cam-gamma` | -1 | Camera gamma; 0 disables the curve |
| `cam-dpc` | -1 | Defect pixel correction, 0 off, 1 on |
| `cam-imgdir` | -1 | 0 normal, 1 mirror, 2 flip, 3 both |
| `cam-testimg` | -1 | Built-in test pattern |
| `cam-lut` | -1 | User LUT, 0 off, 1 on. Set 0 if photometry matters |
| `cam-blc` | -1 | Black level correction mode, 0 automatic, 1 manual |
| `temperature` | — | Sensor temperature in °C, read only |

`-1` everywhere means "leave whatever the camera already has", so you can set one control without
disturbing the rest.

### Transfer curves

`lin` keeps the top 8 bits. Linear, and by far the cheapest, since the answer is the packed high byte
and neither the bit assembly nor the table lookup is needed. It throws away the faint end.

`lsb` keeps the bottom 8 bits at full resolution and clips everything brighter.

`gamma` applies `255*x^gamma` after subtracting the black level. **This is the only curve RMS can
undo**: put the same exponent in the RMS config as `gamma` and star intensities come out right. Use
this one unless photometry does not matter.

`trans`, `gamma`, `black-level` and the `ghs-*` shape can be changed while the pipeline runs: the
table is rebuilt on the spot, so a day mode and a night mode can carry different curves. The one
exception is `cam-curve`, where the curve lives in the camera and is uploaded once - changing it mid
stream is refused out loud rather than accepted and ignored.

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

### Auto exposure, and why it needs limits

`cam-auto-exposure=2` and `cam-auto-gain=2` hand exposure and gain to the camera. **Do not use them at
night.** A changing exposure makes photometry between frames incomparable, and RMS has no way to know
what changed between one FF block and the next. They exist for daytime all-sky capture, where the sky
runs from full sun to overcast rain and no fixed setting covers it.

The mode registers only say who decides. Four more say what it decides towards, and without them "auto"
means the vendor's defaults, which assume a scene that fills the frame:

- `cam-ae-target` is a mean output level. On an all-sky frame aim low, 60 to 90 rather than the usual
  128: the sky is most of the frame, and metering it to mid grey puts the sun and the brightest cloud
  into clipping, which is where the only detail is.
- `cam-ae-roi` matters more than it looks. An all-sky lens does not fill a 4:3 sensor, so the frame
  corners are outside the image circle and permanently black. Metering the whole frame averages that
  dead area in, reads darker than the sky actually is, and overexposes by roughly that much. Give it a
  window inside the circle. `off` restores whole-frame metering, which is not the same as leaving the
  property empty - the camera keeps its last window across power cycles.
- `cam-ae-max-time` caps the exposure. The camera clamps it to one frame period anyway, but set it well
  below only if the scene never gets dark - with day mode running into dusk, just under one frame
  period is the useful setting. The cost of a high ceiling is that the loop takes longest to come back
  down from an exposure at the ceiling, which is what makes a sunrise overshoot.
- `cam-ag-max-gain` caps the gain, and the obvious advice - set it to zero, there is light to spare -
  is wrong. A cap only binds when the exposure ceiling has already been reached, so in full sun the
  loop sits at minimum exposure and 0 dB whatever the cap says. What the cap actually decides is how
  the picture ends in twilight, and day mode runs until the Sun is 9 degrees down. Zero makes dusk go
  black; something like 15 to 20 dB costs nothing by day and keeps the last of the light.

How much range the exposure alone has to cover: measured on an IMX178 with an all-sky lens, a sky in
full sun is still **too dark** at 10 µs and 0 dB. So the short end has margin to spare, and the whole
day from full sun to late dusk sits inside the exposure register with the gain left at zero. The gain is
there for night, not for the day.

`probe=true` prints all four back, and says whether the metering window is on. With the loop running it
also prints what the camera settled on, read from `Exp_Time` and `Cur_Gain` rather than from the manual
registers - those still hold whatever was written by hand and report the request, not the state.

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

The first frame is anchored to the pipeline's running time rather than to zero, and that detail matters.
The anchor is applied to every stamp mode that counts from the first frame - the driver's stamps and the
frame-counter fallback alike - because which one ends up supplying the timestamp is not known until the
driver has been asked, by which time the moment to measure has gone.
The camera can take seconds to deliver frame one - starting the stream, settling the sensor, and with
`cam-curve` also the two and a half second LUT transfer - and a PTS of zero hands all of that back as a
constant error in every absolute timestamp downstream. RMS computes its frame time as
`start_timestamp + pts`, so a session whose first frame arrived four seconds late had every frame dated
four seconds early, and reported `Buffer fill: 100.0%` with no dropped frames for the whole night. The
element now prints the offset it anchored at:

```
mipimvsrc: first frame 3.812 s after the pipeline started, anchoring the driver timestamps there
```

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

## Camera control

This camera exposes nothing through V4L2 controls. Exposure, gain, frame rate, black level, ROI and
the sensor's own pixel format all live in an I2C register space, which the vendor drives with
`mv_mipi_i2c.sh` and two small helper binaries. That protocol is reimplemented here directly, so the
element does not shell out and does not care where the vendor scripts live.

### What this element sets, and what it deliberately does not

The split follows who owns each register. The `veye_mvcam` driver writes `Pixel_Format` and the ROI
registers itself from the media-ctl format, and its `frame_rate`, `roi_x` and `roi_y` V4L2 controls
work. Writing any of those over I2C would leave the driver's cached state disagreeing with the
hardware, so this element does not touch them - use `media_setting_rpi5.sh` and `v4l2-ctl`.

Exposure and gain are the opposite case. The driver registers `V4L2_CID_EXPOSURE` and
`V4L2_CID_ANALOGUE_GAIN`, but its handler only stores the number - the fields are commented
`//just save the value` in the driver source and are never written to the camera. That is why the
vendor's own `set_params.sh` sets them over I2C, and why this element does too.

| parameter | who owns it | how to set it |
|---|---|---|
| pixel format, ROI size | driver, from the media-ctl format | `media_setting_rpi5.sh -fmt -w -h` |
| ROI offset | driver, working V4L2 control | `media_setting_rpi5.sh -x -y`, or `v4l2-ctl --set-ctrl roi_x=` |
| frame rate | driver has a working control, but the register is safe to write | **this element**, or `v4l2-ctl --set-ctrl frame_rate=25` |
| exposure, gain | nobody - the V4L2 controls are stubs | **this element**, or `mv_mipi_i2c.sh` |
| black level, gamma, DPC, test image, orientation | nobody | **this element**, or `mv_mipi_i2c.sh` |

Frame rate is the one borderline case. The driver's `frame_rate` control works, which makes it the
tidier layer, but it only takes whole frames. Writing the register directly leaves nothing but the
driver's `cur_fps` cache stale - its getter and its ROI path both re-read the register - so `cam-fps`
is provided and takes hundredths of a frame. Either path is fine.

So this replaces everything `mv_mipi_i2c.sh` was doing in `set_params.sh`, and none of the geometry:

```
mipimvsrc device=/dev/video8 i2c-bus=-2 probe=true \
    format=Y12P trans=gamma gamma=0.7 black-level=199 \
    cam-auto-exposure=0 cam-auto-gain=0 exposure=40000 gain=30 cam-fps=0 \
    stamps=driver threads=2 buffers=10 width=2400 height=2064 framerate=25/1 \
  ! video/x-raw,format=GRAY8,width=2400,height=2064,framerate=25/1 \
  ! queue max-size-buffers=20 max-size-bytes=0 max-size-time=0 \
  ! appsink max-buffers=100 drop=true sync=0 name=appsink
```

The geometry still comes from the first line of `set_params.sh`, unchanged:

```sh
rpi5_scripts/media_setting_rpi5.sh mvcam -fmt RAW12 -x 344 -y 0 -w 2400 -h 2064 -c 1
```

The element reads the ROI back and compares it with `width` and `height`, so a mismatch shows up as a
line in the log rather than as a sheared image:

```
camera: ROI 2400x2064 at 344,0
```

### Which bus

`i2c-bus=-2` works it out. It walks sysfs to find the media device that owns your capture node, then
asks that media device for its entity names: the sensor appears as `mvcam 4-003b`, so the bus number
is in the name. That ties the answer to the `device` you gave, which is what makes it correct on a
board with two cameras:

```
camera: /dev/video8 is fed by the camera on bus 4
```

If there is no media graph to follow it falls back to looking for a camera anywhere in sysfs, which
can only be trusted when there is exactly one - with two it reports the count and stops rather than
configuring whichever it found first.

The two spellings differ in how they fail:

- **`-2` is best effort.** No camera found, or a bus that cannot be opened (usually because the user is
  not in the `i2c` group), logs a line and capture carries on without camera control. Nothing that
  worked before starts failing because of it.
- **An explicit number is a request**, so a bus that cannot be opened fails the pipeline. If you asked
  for bus 4, silently ignoring it would be worse than stopping.

Two cameras need nothing special, but an explicit bus is still the shortest thing to debug:

```
mipimvsrc device=/dev/video8 i2c-bus=-2 ...
mipimvsrc device=/dev/video0 i2c-bus=-2 ...
```

To see the pairing by hand:

```sh
ls -d /sys/bus/i2c/devices/i2c-*/*-003b/veye_mvcam
media-ctl -d /dev/media0 -p | grep entity
```

### Setting controls with i2c-bus left at -1

They are ignored, because `-1` means the element does not open I2C at all. It says so at start rather
than leaving you to wonder why the gain did not change:

```
mipimvsrc: i2c-bus is -1, so the camera is left alone and these were ignored: exposure gain.
           Add i2c-bus=-2 to apply them.
```

### Why the default is -1 and not -2

Because `-1` means *off*, not *figure it out*: with it this element does not open I2C at all and the
camera is configured by `mv_mipi_i2c.sh` as before. Leaving that as the default keeps every existing
pipeline behaving exactly as it did. Turning camera control on is one argument.

### Setting controls while running

`exposure`, `gain`, `cam-black-level`, `cam-gamma`, `cam-dpc`, `cam-imgdir`, `cam-testimg` and the two
auto modes take effect immediately when set on a running pipeline. `cam-fps` is applied at start only:
changing it mid-stream would leave the negotiated caps claiming a rate the camera no longer delivers. Every write is read back and logged,
because the camera clamps values out of range rather than refusing them:

```
camera: gain 30.0 dB -> 30.0 dB
camera: exposure 40000 us -> 39997 us
```

Pair the manual values with `cam-auto-exposure=0` and `cam-auto-gain=0`. Writing a manual exposure
while the camera is still in an auto mode is accepted and then immediately overridden by the auto loop,
which looks like the write having silently failed. For a meteor station the auto modes are wrong
anyway: an exposure that moves makes photometry between frames incomparable.

### Two properties that are easy to confuse

`black-level` and `gamma` are the **software** curve this element applies while mapping down to 8 bits.
`cam-black-level` and `cam-gamma` are the **camera's own** settings and are a different thing. In the
example above the camera's gamma is left alone and the mapping is done here, because that is the one
curve RMS can invert.

### Black level: the register and the pedestal are different things

`cam-blc` selects between the camera calibrating its own black level (mode 0) and subtracting the value
in the black level register (mode 1). In mode 0 **the register is not applied**, so reading it back gives
a number unrelated to the pedestal actually present in the data - one camera reported 60 while its RAW12
frames sat at about 199. That is not a fault, and `probe=true` now prints both together with which one is
in force.

It matters because `black-level` on this element is a fixed number, and in mode 0 the pedestal the camera
produces can move between nights and with temperature. If the pedestal drifts up past `black-level` the
sky clips to zero; if it drifts down, range is spent on the pedestal instead of the sky. Pinning it with
`cam-blc=1` and a measured value removes that, but do measure first: switching to mode 1 makes the
register live, and if it holds something far from the real pedestal the image changes abruptly.

The pedestal itself is measured by capping the lens and taking the median of a RAW12 frame. It cannot be
read off a normal sky frame, and it cannot be read off the unlit-looking corners of a daylight frame
either - on this lens those corners still carried 88% of the centre's signal at 0 dB.

### Frames the camera dropped by itself

The camera counts frames the sensor produced and frames it actually sent. The element takes both at
start and reports the difference at stop:

```
camera: produced 30012 frames, sent 30012
camera: produced 30012 frames, sent 29987, DROPPED 25 inside the camera
```

This is worth having because a gap here is invisible from the host: no amount of `buffers`, queue depth
or thread tuning can recover a frame the camera never sent, and without the counters it looks exactly
like the host being too slow. If the two numbers match and RMS still reports drops, the problem is on
this side of the MIPI link.

The camera's error code is read at the same points and printed when it is non-zero.

### A LUT left enabled is worth checking for

`probe=true` reports whether the camera's user lookup table is active:

```
camera: user LUT ON  <- this alters pixel values on top of gamma
```

If it is, every pixel has been through an arbitrary curve before the one this element applies, and
nothing downstream can tell. On a station that does photometry that is a bug waiting to be blamed on
something else, so `cam-lut=0` is cheap insurance.

### Sensor temperature

`temperature` is read from the camera each time the property is read, so it can be polled during a run.
Worth logging: measurements on an IMX178 found the hot pixel pattern's amplitude moving from 27 to 43
ADU across nights while the sky stayed put, which was temperature rather than anything in the sky.

### Defect pixel correction, and why you cannot load your own map

`cam-dpc=1` turns on whatever correction the camera has. That is the whole interface: the vendor's
[register map](https://wiki.veye.cc/index.php/MV_Series_MIPI_Camera_Register_Map) documents DPC as a
single bit (bit 4 of ISP module ctrl, 0x0C00) and nothing else.

**There is no way to upload a defect pixel list.** No such register exists in the published map, in the
driver header, or in the vendor's tools. The camera's own manual does not mention DPC at all.

Three registers named `DPC_Start` (0xC64), `DPC_Status` (0xC68) and `DPC_Count` (0xC6C) do appear in
every version of the vendor's driver header, which suggests the camera can run its own detection pass -
but they are absent from the published register map, no code they ship touches them, and nothing
describes what values they take. `probe=true` prints 0xC6C as a raw register value for that reason,
rather than calling it a pixel count - and on the IMX178 firmware tested here it reads all ones, i.e. the
register is not implemented at all, so there is no detection pass to read a result from.

The LUT is not an alternative. It is a tone curve - `LUT Count` (0x0C74) is 1024 or 4096 entries, with
the data at 0x1800-0x37FF - so it maps an intensity to an intensity, identically for every pixel. The
samples the vendor ships are gamma 0.8, gamma 1.2 and linear.

So the hot pixel map stays on the host. Measurements on an IMX178 with this lens found about 1700
pixels standing more than 8 levels above their surroundings, and masking them cut the detections that
were not stars by 58 to 80%. Whether `cam-dpc=1` covers any of those is worth testing empirically:
enable it, collect a night, rebuild the fixed pattern and compare the count.

### Moving the curve into the camera

`cam-curve=true` builds this element's transfer curve at the camera's own LUT resolution, uploads it,
and lets the camera apply it. With the camera set to Mono8 the frame then arrives already curved and the
element has nothing left to do.

**This is measured, not assumed.** `LUT Count` (0xC74) reports 4096 entries in Mono12 *and in Mono8*,
1024 in Mono10 - so the table works on the sensor's 12 bit values, before the logic unit the vendor
describes as performing *"data rounding to achieve a different pixel format"*. And the table really does
reach raw data: with a gamma 0.5 curve loaded, a population sitting at 199 came back at 903, where
`4095*(199/4095)^0.5` is 903.

What it buys, per camera:

| | before | after |
|---|---|---|
| host CPU per frame | 2.4 ms (`lin`) to 5.7 ms (a curve), single threaded at 2400x2064 | nothing |
| MIPI payload at 25 fps | 186 MB/s | **124 MB/s** |
| tonal spread | ours | still ours, unlike the camera's own 8 bit curve |

That last column is the point worth repeating. The camera's built-in Mono8 curve put the whole sky
inside about 5 output levels on this sensor; gamma 0.5 spreads it over 40. `cam-curve` keeps the second
figure while paying the first column's price of nothing.

Set `trans`, `gamma`, `black-level` and the `ghs-*` parameters exactly as before - they describe the
curve either way. All four `trans` modes can move, including `ghs`, which gives the best spread of the
four but which RMS cannot invert, so it suits a preview or a long-exposure stack rather than photometry.

```
mipimvsrc device=/dev/video8 i2c-bus=-2 cam-curve=true \
    format=GRAY8 trans=gamma gamma=0.7 black-level=199 \
    cam-auto-exposure=0 cam-auto-gain=0 exposure=40000 gain=30 \
    width=2400 height=2064 framerate=25/1 stamps=driver buffers=10 ! ...
```

with the camera configured for Mono8 rather than RAW12:

```sh
rpi5_scripts/media_setting_rpi5.sh mvcam -fmt RAW8 -x 344 -y 0 -w 2400 -h 2064 -c 1
```

Once the upload succeeds the element **stops curving by itself**, and this is not left to the user to
arrange. The uploaded table already folds in `black-level`, so the camera's output has black at zero;
applying the host curve on top would subtract the black level a second time and everything below it
would clamp to zero. The host table is replaced by the plain top-8-bits ramp, which is the identity for
`GRAY8` input, and the `trans=lin` fast path is used so there is no lookup at all.

This is worth stating because the first version only *warned* when `format` was not `GRAY8`, which
missed the real problem: with `black-level=199` the host table maps everything under 199 to black, so
even `GRAY8` came out completely black. Measured on the first test night: avepixel median 0, max 255 -
only the pixels whose camera output already exceeded 199 survived.

`format=GRAY8` with the camera in Mono8 is still what you want, because that is what drops the
unpacking and halves the MIPI payload. With `Y12P` the feature works, it simply saves less.

Do not also set `cam-gamma`: uploading the curve turns the camera's internal gamma off, and the
uploaded table is the curve.

The table is scaled to `255*(count/256)`, which is 4080 rather than 4095, because the rounding divides
by 16 and `4095/16` is 255.94 - filling the table to its maximum leaves every value a level high.
At 4080 the camera's Mono8 byte matches what this element would have produced, median difference 0.

The transfer is paced: 500 ms after the start register and 1 ms after each of the 2048 data writes, so
about two and a half seconds at pipeline start. That pacing comes from the vendor's `lut_rw.c` and it is
not optional - sending the table at full speed leaves the camera holding only part of it, which shows up
as an image crushed to black rather than as any kind of error. Eight entries are read back afterwards
and a mismatch fails the upload instead of enabling a broken curve.

Writing and reading the table are not symmetric, which is worth knowing before writing any check: a
write to `LUT_DATA + i*2` carries entries *i* and *i+1* packed into one 32 bit register, but a **read of
the same address returns entry i alone**, in the low half, with the high half zero. The vendor's reader
shows this - it walks its index by one where the writer walks by two. Comparing a read against the
packed pair rejects a perfectly good upload.

`test_lut_path.sh` is what established all of this, and re-running it is the way to check a different
camera model before trusting any of it.

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
- Camera control sets only what the driver does not own, so `media_setting_rpi5.sh` is still required
  for format and geometry. The ROI is read back and checked, never written.
- Of what the vendor's `mv_mipi_i2c_new.sh` can reach, these are deliberately not exposed:
  - **`roi`, `pixelformat`, `readmode`** - the driver owns the frame geometry and format.
  - **`trgmode`, `trgsrc`, `trgone`** - the driver has working V4L2 controls for these.
  - **`lanenum`, `clkmode`, `mipidatarate`, `i2caddr`, `i2cport`, `snsreg`** - changing the link
    configuration or poking raw sensor registers under a bound driver breaks the stream.
  - **`reboot`, `factoryparam`, `paramsave`** - `paramsave` writes flash, and a property that fires on
    every pipeline start is the wrong shape for that. Use `mv_mipi_i2c.sh` when you mean it.
  - **`aatarget`, `aemaxtime`, `agmaxgain`, `aaroi`** - only relevant with auto exposure or gain, which
    a meteor station should not be using.
  - **`lut`** (loading a table), **`osdmode`**, **`slavemode`**, the remaining trigger timing, and the
    GPIO/strobe outputs. Reachable the same way if a use appears; `mvcam.h` is where to add them.
- `cam-fps` writes the `Framerate` register (hundredths of a frame). The camera also has `FrameRate_Ex`
  at ten-thousandths, but the driver reads `Framerate`, so that is the one kept in step.
- No SER or FITS writing. GStreamer has no SER support either, though the format is simple enough
  that a sink would be short work once 16 bit output is being recorded.

## License

GPL-3.0-or-later. See [LICENSE](LICENSE).

**Why GPL at all:** the register map, the scaling factors and the I2C transfer format in `mvcam.cpp`
were worked out from the vendor's tools in
[veyeimaging/raspberrypi_v4l2](https://github.com/veyeimaging/raspberrypi_v4l2) - specifically
`mv_tools_rpi/mv_mipi_i2c_new.sh` and `mv_tools_rpi/sources/i2c_4read.c` / `i2c_4write.c`. Register
addresses are arguably interface rather than expression, but the protocol was reimplemented after
reading their code closely, so treating this as a derived work is the honest position.

That repository is **internally inconsistent about its licence**, which is worth recording because it
is awkward to reconstruct later:

| | licence |
|---|---|
| repository root `LICENSE` | GPL-3.0, full text |
| the userspace tools this derives from | no per-file header, so the root `LICENSE` governs → GPL-3 |
| the kernel driver `veye_mvcam.c` | `SPDX-License-Identifier: GPL-2.0` and `MODULE_LICENSE("GPL v2")` |

GPL-2.0-only and GPL-3 are incompatible, so that distinction matters. **Nothing here is derived from the
driver.** It was read only to establish facts about behaviour - which registers it writes from the
media-ctl format, and that its `exp_val` / `again_val` never reach the camera - and those findings are
recorded as comments, not as code. The register addresses appear in both the GPL-3 script and the
driver's GPL-2 header; they were taken from the script, and in any case an address is a fact about an
interface.

**Why version 3 and not 2:** the element links OpenCV, whose Apache-2.0 patent clause is compatible
with GPL-3 but not with GPL-2. GStreamer itself is LGPL-2.1-or-later, which a GPL-3 plugin may link.

**What this does not affect:** [RMS](https://github.com/CroatianMeteorNetwork/RMS). None of its code is
here and none of its headers are included - it only runs a pipeline that happens to contain this
element. Its own licence imposes nothing on this module, and this module imposes nothing on RMS.
GStreamer loads this plugin into whichever process runs the pipeline, so strictly speaking the two end
up in one address space, but RMS is GPL-3 as well, so there is no friction either way.

`GST_PLUGIN_DEFINE` declares `"GPL"` accordingly. It used to say `"LGPL"`, which GStreamer would have
taken at face value when checking whether this plugin may sit in a pipeline with others.
