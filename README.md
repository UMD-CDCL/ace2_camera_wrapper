# ace2_camera_wrapper

ROS2 C++ wrapper for the Basler **ace 2 a2A2448-75ucPRO** (5 MP colour, USB3) built
directly on the pylon 8 C++ SDK. It publishes colour images and camera info on the
standard topics and exposes the camera settings that actually matter as ROS
parameters.

This is a deliberately small alternative to Basler's own `pylon_ros2_camera`, which
also lives in this workspace. Use that one if you need its full service surface.
Use this one if you want a single node you can read in one sitting.

## Interface

| Topic / service | Type | Notes |
| --- | --- | --- |
| `ace2/image_raw` | `sensor_msgs/Image` | `bgr8` by default |
| `ace2/camera_info` | `sensor_msgs/CameraInfo` | published with every frame |
| `ace2/image_raw/compressed` | `sensor_msgs/CompressedImage` | only if `image_transport_plugins` is installed |
| `~/temperature` | `sensor_msgs/Temperature` | only when `temperature_publish_period > 0` |
| `set_camera_info` | `sensor_msgs/SetCameraInfo` | from `camera_info_manager`, writes the calibration file |
| `~/execute_software_trigger` | `std_srvs/Trigger` | grabs one frame when `trigger_mode` is `On` |

Topic names are relative, so under a namespace they become
`/<namespace>/ace2/image_raw` and `/<namespace>/ace2/camera_info`. Remap them from
the launch file if you need a different layout.

The node is a **ROS2 component**, registered as
`ace2_camera_wrapper::Ace2CameraWrapperNode`. It can run as its own process or be
loaded into a container alongside other components, which is what allows a 15 MB
frame to reach a subscriber without being copied. See [Composition](#composition).

## Building

The pylon SDK is not a ROS package and has no rosdep key, so it is found on its
own. The default prefix is `/opt/pylon`, and `PYLON_ROOT` in the environment
overrides it.

```bash
cd ~/ros2_ws
colcon build --packages-select ace2_camera_wrapper
source install/setup.bash
```

The link line comes from `pylon-config` and includes an rpath to
`$PYLON_ROOT/lib`, so the node finds `libpylonbase.so` without `LD_LIBRARY_PATH`.

ROS dependencies are `rclcpp`, `sensor_msgs`, `std_srvs`, `image_transport` and
`camera_info_manager`. There is no OpenCV or `cv_bridge` dependency, because pylon
does the demosaicing itself.

## Running

```bash
ros2 launch ace2_camera_wrapper ace2_camera_wrapper.launch.py

# a few of the common overrides
ros2 launch ace2_camera_wrapper ace2_camera_wrapper.launch.py \
    frame_rate:=60.0 exposure_auto:=Off output_encoding:=passthrough
```

Every parameter lives in `launch/ace2_camera_wrapper.launch.py` with a comment, and
`ros2 param describe /ace2_camera_wrapper <name>` prints the same description at
runtime. The rest of this file covers only the things that are easy to get wrong.

## Composition

Loading the camera into a container with its consumers is the whole point of the
component form. An intra-process subscriber gets the frame by move, with no copy
and no serialisation.

```bash
# own process (default), behaves exactly as it always did
ros2 launch ace2_camera_wrapper ace2_camera_wrapper.launch.py

# inside a container, so components in that container get frames without a copy
ros2 launch ace2_camera_wrapper ace2_camera_wrapper.launch.py use_composition:=true
```

Measured on this hardware at 60 fps with `bgr8` output, one subscriber in the
container and one in a separate process, running at the same time:

| Subscriber | Frames received | Effective |
| --- | --- | --- |
| same container (intra-process) | every frame, 60 fps | ~940 MB/s |
| separate process (DDS) | ~30 fps, about half dropped | ~450 MB/s |

Three conditions all have to hold, or you quietly fall back to a copy:

1. **`use_image_transport` must be `false`** (the default). Every publish overload
   in `image_transport` dereferences into a copy, including the shared-pointer
   ones, so that path can never be zero-copy. It is still there when you want
   `ace2/image_raw/compressed`, and the node warns on startup when it is on.
2. **The container and the component both need intra-process comms.** The launch
   file passes `use_intra_process_comms: True` already.
3. **The subscriber's QoS must match**, that is `best_effort` with depth 1 by
   default. A mismatched subscriber does not connect at all.

Subscribe with `sensor_msgs::msg::Image::UniquePtr` or
`const sensor_msgs::msg::Image::ConstSharedPtr &` in the callback. Taking the
message by value copies it and throws the benefit away.

### Using it from another package

For composition, nothing needs to be compiled against this package. Reference the
plugin by name from your own launch file:

```python
launch_ros.descriptions.ComposableNode(
    package='ace2_camera_wrapper',
    plugin='ace2_camera_wrapper::Ace2CameraWrapperNode',
    name='ace2_camera_wrapper',
    parameters=[{'frame_rate': 60.0}],
    extra_arguments=[{'use_intra_process_comms': True}],
)
```

Add `<exec_depend>ace2_camera_wrapper</exec_depend>` to your `package.xml`, or load
it at runtime with `ros2 component load /your_container ace2_camera_wrapper
ace2_camera_wrapper::Ace2CameraWrapperNode`.

To construct the class directly in C++ instead:

```cmake
find_package(ace2_camera_wrapper REQUIRED)
target_link_libraries(my_target ace2_camera_wrapper::ace2_camera_wrapper)
```

The exported target carries pylon's include directories, so a package that
includes the header needs no pylon setup of its own, as long as the SDK sits at
the same prefix. Only direct includers need this. Composition does not.

## Pixel format and why the default is Bayer

The camera's `DeviceLinkThroughputLimit` defaults to 360 MB/s, and a full frame is
2448 x 2048. That budget is what decides the achievable frame rate:

| `pixel_format` | bytes/frame | frames/s the link allows |
| --- | --- | --- |
| `BayerRG8` | 5.01 MB | ~71 |
| `BGR8` (camera demosaics) | 15.04 MB | ~24 |

Debayering on the host costs about 1.2 ms per frame, measured on this machine, so
the default is `pixel_format: BayerRG8` with `output_encoding: bgr8`. Paying 1.2 ms
of CPU is much cheaper than giving up two thirds of the frame rate.

Set `output_encoding: passthrough` to publish `bayer_rggb8` untouched and let a
downstream node or a GPU demosaic. That is the cheapest option on the host by far,
and it also cuts the published volume by three, which matters for any subscriber
that is not in the same container.

`sharpness_enhancement` and `noise_reduction` are Basler's PGI steps. They only run
when the camera itself demosaics, so with any Bayer `pixel_format` they are
read-only. They default to `-1.0`, meaning "leave alone", and are only worth
setting alongside `pixel_format: RGB8` or `BGR8`.

## Frame rate

Measured against the camera's own timestamp counter, at full 2448 x 2048 with
`BayerRG8`, with no failed grabs in any case:

| Setting | Achieved |
| --- | --- |
| `frame_rate: 30.0` | 30.00 fps |
| `frame_rate: 60.0` | 60.00 fps |
| free-running, default 360 MB/s limit | **71.80 fps** |
| free-running, `device_link_throughput_limit: 419430400` | **75.34 fps** |

So the practical ceiling is 71.8 fps out of the box, and **75.34 fps** is the hard
maximum. Raising the throughput limit is what gets you the last 3.5 fps, and 75.34
is then the sensor itself: `SensorReadoutTime` is 13.27 ms, and 1 / 0.01327 is
75.3. Asking for more than that just returns 71.8 or 75.3, it does not fail.

Two things to get right when raising the rate:

- **`exposure_auto_upper_limit` must stay below `1e6 / frame_rate` microseconds.**
  Otherwise the auto exposure stretches past the frame period in a dim scene and
  the camera drops the rate to match. This is the single most common reason a
  requested rate is not met, so the node warns at startup when the two conflict.
- **Raise `usbfs_memory_mb`**, see below.

`stats_log_period` logs the node's own measured grab rate every N seconds. Prefer
it over `ros2 topic hz` when checking whether a rate is met, because it counts
what the camera delivered rather than what survived the transport.

## Region of interest

Left at the defaults (`image_width: 0`, `image_height: 0`, `offset_x: -1`,
`offset_y: -1`) the node does not touch the ROI, so you get what
`startup_user_set` gives you: the nominal **2448 x 2048** imaging area centred on
the sensor. That is the resolution on the datasheet and the one any calibration
file will have been made against.

The sensor is addressable out to 2472 x 2064, but those extra rows and columns sit
outside the specified imaging area, so you have to ask for them by number rather
than by leaving a parameter at zero. Shrinking the ROI is the cheapest way to a
higher frame rate, since it shrinks the frame against the link budget above.

## Timestamps

`timestamp_source: ros_time` (the default) stamps the frame when the host retrieves
it. Robust, but it carries the host's scheduling jitter.

`timestamp_source: camera` uses the device counter, which ticks at 1 GHz on this
model, so one tick is one nanosecond. This was measured against the host clock
rather than assumed. It is jitter-free, and it is re-anchored to ROS time every
`timestamp_resync_period` seconds so the camera and host oscillators cannot drift
apart without bound. Each re-anchor is a small step in the stamps, so leave it at
0 if you would rather have smoothness than bounded drift.

`timestamp_offset` is added to every stamp either way, to compensate a known
exposure and transfer latency.

## Things that will bite you

**`no Basler camera found` while `lsusb` shows the camera.** pylon drives the
camera through libusb, which needs read *and* write access to
`/dev/bus/usb/<bus>/<device>`. `lsusb` only reads descriptors, which the read bit
alone allows, so with the wrong permissions the camera is listed by `lsusb` and
still invisible to `CTlFactory::EnumerateDevices`, which returns zero devices and
makes the node throw. Read access is enough to see the camera and not enough to
open it, which is why this looks like a missing camera rather than a denied one.

pylon's installer ships the rule that fixes it:

```
# /etc/udev/rules.d/69-basler-cameras.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="2676", MODE:="0666", TAG+="uaccess", TAG+="udev-acl"
```

**That rule only takes effect on the host.** If the node runs inside Docker, the
copy of the rule in the image does nothing: no udev daemon runs in a Docker
container, and `/dev/bus/usb` is bind-mounted from the host's devtmpfs, so the mode
is whatever the host assigned. Installing pylon in the image therefore does not
grant access to the camera. Put the rule on the *host*, reload it, and replug the
camera:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Because it is a bind mount, the corrected mode shows up inside the Docker
container immediately, with no image rebuild and no restart. Verify from wherever
the node actually runs:

```bash
lsusb | grep -i basler        # -> Bus 004 Device 002: ID 2676:ba05 Basler ...
ls -l /dev/bus/usb/004/002    # want crw-rw-rw-, not crw-rw-r--
```

A one-off `sudo chmod 666 /dev/bus/usb/004/002` unblocks the camera now and is lost
on the next replug.

**QoS.** The publisher defaults to `best_effort` with depth 1, which is the usual
profile for a sensor stream and the only sane one for 15 MB frames. A subscriber
that asks for `RELIABLE` will not connect to a `best_effort` publisher and will
simply see nothing, with no error anywhere. Set `qos_reliability: reliable` if you
need that, and expect the memory cost.

**`usbfs_memory_mb` defaults to 16.** This caps the kernel memory userspace may
have pinned for in-flight USB transfers, and the default is a conservative guard
that predates USB3 cameras. Pylon's own defaults for this camera
(`NumMaxQueuedUrbs` 64 x `MaxTransferSize` 262144) come to exactly 16 MiB, so a
single camera already sits at the ceiling. It streams fine at 30 fps, but this is
where failed grabs show up at higher frame rates, with a larger `max_num_buffer`,
or with a second camera. Basler recommends 1000.

```bash
# check the current value
cat /sys/module/usbcore/parameters/usbfs_memory_mb

# raise it now (lost on reboot)
echo 1000 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

To make it persist, add `usbcore.usbfs_memory_mb=1000` to
`GRUB_CMDLINE_LINUX_DEFAULT` in `/etc/default/grub` and run `sudo update-grub`.
That is the route to use when `usbcore` is built into the kernel, which is the
usual case and means `/etc/modprobe.d` is ignored. A reboot-free equivalent that
works either way:

```bash
echo 'w /sys/module/usbcore/parameters/usbfs_memory_mb - - - - 1000' \
  | sudo tee /etc/tmpfiles.d/usbfs-memory.conf
```

All three commands target the host, since `/sys/module/usbcore` belongs to the
kernel and a Docker container has no writable copy of it. Setting this from inside
a container fails even as root, so raise it on the host and read the value back
from the container to confirm it applied.

**`Off` and `On` are YAML booleans.** `exposure_auto`, `gain_auto`,
`balance_white_auto`, `light_source_preset`, `color_space` and `trigger_mode` all
take GenICam values that include `Off` and `On`, and YAML 1.1 reads both as
booleans. Those six parameters are therefore declared with dynamic typing and
coerce a bool back to `Off` or `On`, so all of these work:

```bash
ros2 param set /ace2_camera_wrapper exposure_auto Off
ros2 param set /ace2_camera_wrapper exposure_auto Continuous
```

**Auto exposure fights the frame rate.** `exposure_auto_upper_limit` needs to stay
below `1e6 / frame_rate` microseconds, or the auto function will starve the frame
rate to reach its brightness target. The default 20000 us suits 30 fps.

## Runtime reconfigure

The knobs worth tuning while looking at the image are live via `ros2 param set`:
the exposure, gain, white balance and colour parameters, plus `frame_rate`,
`reverse_x`, `reverse_y`, `frame_id`, `timestamp_source` and `timestamp_offset`.

Anything that changes the buffer layout or the transport (`pixel_format`, the ROI,
the USB and QoS settings) is read once when the camera is opened. Setting one of
those is rejected with a message saying so, rather than being half-applied.

Note that a manual value is only writable while its auto function is off. Setting
`exposure_time` while `exposure_auto` is `Continuous` logs a warning and does
nothing, which is the camera's rule, not the node's.

## Calibration

Uncalibrated, `CameraInfo` carries only the image size and the node says so once.
To calibrate:

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
    --ros-args -r image:=/ace2/image_raw -r camera:=/ace2
```

Pressing **Commit** calls `set_camera_info`, which writes
`~/.ros/camera_info/<camera_name>.yaml`. Point `camera_info_url` at a file to load
it from somewhere else, for example
`camera_info_url:=file:///home/user/ace2_calib.yaml`.

## Robustness

The grab thread owns the camera for its whole lifetime. On a grab error, a stalled
stream or the camera being unplugged, it closes the device, waits
`reconnect_delay` seconds and reconnects, reapplying every parameter. The node
stays up throughout. A grab timeout is only treated as a stall when the camera is
free-running, because a triggered camera is idle by design between triggers.

## Verified on this hardware

Checked against the attached camera (`a2A2448-75ucPRO`, serial 42003420, pylon
8.0.0, ROS 2 Humble):

- 30.00, 60.00, 71.80 and 75.34 fps hit exactly as described above, zero failed
  grabs in every run
- `ace2/image_raw` at 2448 x 2048, `step` 7344, encoding `bgr8`
- `passthrough` publishing `bayer_rggb8` with `step` 2448
- BayerRG8 to BGR8 conversion in 1.2 ms per frame
- device counter confirmed at 1 tick = 1 ns
- software trigger mode grabbing exactly one frame per service call
- component loads into a container and an intra-process subscriber receives every
  frame at 60 fps while an out-of-process one drops about half
- this model has no `Binning` or `Decimation` features, so none are exposed

Note that the frame rates above were verified against the camera's timestamp
counter rather than the host clock, since the two can disagree by several percent
on a virtualised host. `stats_log_period` uses the host clock, so treat it as
indicative rather than exact.
