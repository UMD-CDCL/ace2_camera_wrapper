# Launch the Basler ace 2 a2A2448-75ucPRO wrapper.
#
# Every camera parameter lives in the `params` dict below so it is easy to find
# and edit. The handful that change often are also exposed as launch arguments,
# e.g.
#
#   ros2 launch ace2_camera_wrapper ace2_camera_wrapper.launch.py \
#       frame_rate:=60.0 exposure_auto:=Off
#
# For a camera that is not mounted upright, `image_rotation` turns the published
# frame clockwise and `flip_horizontal` / `flip_vertical` mirror it, e.g.
#
#   ros2 launch ace2_camera_wrapper ace2_camera_wrapper.launch.py image_rotation:=90
#
# Sensible starting points for this camera:
#   * pixel_format BayerRG8 + output_encoding bgr8 is the fast path. A frame is
#     2448x2048, so 5.0 MB as Bayer against the camera's 360 MB/s link budget
#     leaves room for ~71 fps. Asking the camera for BGR8 instead makes the frame
#     15.0 MB and caps the link at ~24 fps, so host debayering (~1.2 ms/frame)
#     is much the better trade.
#   * output_encoding 'passthrough' with pixel_format BayerRG8 publishes
#     bayer_rggb8 and does no work on the host at all. Use it when a downstream
#     node or a GPU does the demosaicing.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.conditions import IfCondition, UnlessCondition

import launch_ros.actions
import launch_ros.descriptions
from launch_ros.parameter_descriptions import ParameterValue

# Match the rest of the workspace: run inside the robot's namespace when one is
# set, and at the root otherwise.
default_namespace = os.environ.get('SPOT_NAME', '')


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(
            'namespace', default_value=default_namespace,
            description='Namespace for the camera topics. Empty puts them at the root.'),
        DeclareLaunchArgument(
            'frame_id', default_value='ace2_camera_optical_frame',
            description='frame_id stamped on Image and CameraInfo.'),
        DeclareLaunchArgument(
            'device_serial_number', default_value='',
            description='Serial of the camera to open. Empty opens the first one found.'),
        DeclareLaunchArgument(
            'camera_info_url', default_value='',
            description='Calibration URL'),
        DeclareLaunchArgument(
            'frame_rate', default_value='60.0',
            description='Acquisition rate in Hz. 0 frees the rate (exposure and link limited).'),
        DeclareLaunchArgument(
            'exposure_auto', default_value='Continuous',
            description='ExposureAuto: Off, Once or Continuous.'),
        DeclareLaunchArgument(
            'output_encoding', default_value='bgr8',
            description='Published encoding: bgr8, rgb8, mono8 or passthrough.'),
        DeclareLaunchArgument(
            'exposure_auto_upper_limit', default_value='15000.0',
            description='Upper bound in us for the auto exposure. Keep it below '
                        '1e6/frame_rate, or the auto function will hold the frame rate '
                        'down to reach its brightness target.'),
        DeclareLaunchArgument(
            'image_rotation', default_value='0',
            description='Rotate the published image clockwise by 0, 90, 180 or 270 '
                        'degrees, for a camera that is not mounted upright. 90 and 270 '
                        'swap the published width and height.'),
        DeclareLaunchArgument(
            'flip_horizontal', default_value='false',
            description='Mirror the published image about its vertical axis, before '
                        'image_rotation.'),
        DeclareLaunchArgument(
            'flip_vertical', default_value='false',
            description='Mirror the published image about its horizontal axis, before '
                        'image_rotation.'),
        DeclareLaunchArgument(
            'stats_log_period', default_value='0.0',
            description='Log the measured grab rate every N seconds. 0 disables.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Node log level.'),
        DeclareLaunchArgument(
            'use_image_transport', default_value='false',
            description='true routes frames through image_transport, adding the '
                        'compressed transports but copying every frame. false uses '
                        'plain publishers, which can move a frame to a subscriber in '
                        'the same process with no copy.'),
        DeclareLaunchArgument(
            'use_composition', default_value='false',
            description='true loads the camera as a component into a container, so '
                        'components in that container receive frames without a copy. '
                        'false runs it as its own process, exactly as before.'),
        DeclareLaunchArgument(
            'container_name', default_value='ace2_camera_container',
            description='Name of the component container created when use_composition '
                        'is true.'),
    ]

    params = {
        # ---------------------------------------------------------- device ----
        # Set device_serial_number when more than one Basler camera is attached.
        # This camera reports serial 42003420.
        # Every substitution below is pinned to an explicit type. launch_ros
        # otherwise infers the type from the substituted text, and YAML would turn
        # a serial like 42003420 into an integer and 'Off' into a boolean, neither
        # of which matches the parameter the node declares.
        'device_serial_number': ParameterValue(
            LaunchConfiguration('device_serial_number'), value_type=str),
        'device_user_id': '',              # alternative filter, matches DeviceUserID

        # ------------------------------------------------------ ROS output ----
        'camera_name': 'ace2_camera',      # names/validates the camera_info_url file
        'frame_id': ParameterValue(LaunchConfiguration('frame_id'), value_type=str),
        'camera_info_url': ParameterValue(
            LaunchConfiguration('camera_info_url'), value_type=str),

        # A 5 MP colour stream is a lot to keep reliable, so the publisher matches
        # the usual sensor-data profile. Note that a subscriber asking for
        # RELIABLE will not connect to a BEST_EFFORT publisher and will simply see
        # no messages, so switch this to 'reliable' if you need that.
        'qos_reliability': 'best_effort',
        'qos_depth': 1,

        # False publishes with plain rclcpp publishers, which hand each frame to a
        # same-process subscriber by move rather than copying it. True routes
        # through image_transport instead, which adds image_raw/compressed and the
        # other transports but copies every frame on the way out, because every
        # publish overload in image_transport dereferences into a copy.
        'use_image_transport': ParameterValue(
            LaunchConfiguration('use_image_transport'), value_type=bool),

        # ----------------------------------------------------- acquisition ----
        # Loading a user set first makes startup repeatable: without it, any
        # parameter left at its default silently inherits whatever the previous
        # run wrote into the camera. Empty string skips the load.
        'startup_user_set': 'Default',

        'pixel_format': 'BayerRG8',        # Mono8, Mono12, RGB8, BGR8, YCbCr422_8, BayerRG8, BayerRG12, ...
        'output_encoding': ParameterValue(
            LaunchConfiguration('output_encoding'), value_type=str),
        'frame_rate': ParameterValue(LaunchConfiguration('frame_rate'), value_type=float),

        # latest_image_only drops any backlog to keep latency low. Use
        # one_by_one when recording, so no frame is skipped.
        'grab_strategy': 'latest_image_only',
        'grab_timeout_ms': 5000,
        'max_num_buffer': 16,
        'reconnect_delay': 2.0,            # retry delay after an error or unplug

        # ------------------------------------------------ region of interest ---
        # Left at 0/0/-1/-1 the camera keeps the ROI from startup_user_set, which
        # is the nominal 2448x2048 imaging area centred on the sensor. That is the
        # resolution on the datasheet and the one a calibration file will match.
        # The array is addressable to 2472x2064, but those extra rows and columns
        # are outside the specified imaging area, so ask for them by number if you
        # really want them. A smaller ROI is the cheapest way to a higher frame
        # rate, since it shrinks the frame against the link budget.
        'image_width': 0,
        'image_height': 0,
        'offset_x': -1,                    # -1 centres the ROI on the sensor
        'offset_y': -1,
        # --------------------------------------------------------- orientation -
        # Two ways to turn the image, and they are not interchangeable.
        #
        # reverse_x / reverse_y mirror on the sensor and cost nothing, but they
        # shift the phase of the Bayer tile without the camera saying so, so with
        # the Bayer pixel_format above they wreck the colours. They are only safe
        # alongside a Mono or an RGB/BGR pixel_format.
        'reverse_x': False,                # mirror horizontally on the camera
        'reverse_y': False,                # mirror vertically on the camera

        # image_rotation / flip_horizontal / flip_vertical turn the frame on the
        # host instead. They fold into a single pass, relabel the Bayer pattern
        # and rewrite CameraInfo to match, so they are the ones to use for a
        # rotated mounting. The flips happen first, then the clockwise rotation.
        #
        # The cost is one extra pass over the frame. A 90 or 270 rotation
        # transposes, so it moves a pixel at a time rather than a row at a time
        # and is the expensive case; 180 and the plain flips stay row-wise. All
        # three are read once when the node starts, so the grab loop pays a
        # single branch per frame and never a lock.
        'image_rotation': ParameterValue(
            LaunchConfiguration('image_rotation'), value_type=int),
        'flip_horizontal': ParameterValue(
            LaunchConfiguration('flip_horizontal'), value_type=bool),
        'flip_vertical': ParameterValue(
            LaunchConfiguration('flip_vertical'), value_type=bool),

        # ---------------------------------------------------------- exposure --
        'exposure_auto': ParameterValue(
            LaunchConfiguration('exposure_auto'), value_type=str),
        'exposure_time': 5000.0,           # us, used when exposure_auto is Off
        'exposure_auto_lower_limit': 100.0,    # us
        # Keep the upper limit under 1e6/frame_rate or the auto function will
        # starve the frame rate to reach its brightness target. 20 ms suits 30 fps.
        'exposure_auto_upper_limit': ParameterValue(
            LaunchConfiguration('exposure_auto_upper_limit'), value_type=float),  # us

        # ------------------------------------------------------------- gain ----
        'gain_auto': 'Continuous',         # Off, Once, Continuous
        'gain': 0.0,                       # dB (0 to 48), used when gain_auto is Off
        'gain_auto_lower_limit': 0.0,      # dB
        'gain_auto_upper_limit': 24.0,     # dB, capped to keep noise reasonable
        'auto_target_brightness': 0.5,     # 0 to 1, target for the auto functions
        'black_level': 0.0,                # DN
        'gamma': 1.0,                      # 1.0 is linear

        # ------------------------------------------------------------ colour ---
        'balance_white_auto': 'Continuous',    # Off, Once, Continuous
        'balance_ratio_red': 1.0,          # applied only while balance_white_auto is Off
        'balance_ratio_green': 1.0,
        'balance_ratio_blue': 1.0,
        # Off, Tungsten, Daylight5000K, Daylight6500K, FactoryLED6000K
        'light_source_preset': 'Daylight6500K',
        'color_space': 'sRgb',             # Off or sRgb
        'saturation': 1.0,                 # 0 to 2
        'hue': 0.0,                        # degrees, -180 to 180
        'contrast': 0.0,                   # -1 to 1
        'brightness': 0.0,                 # -1 to 1

        # PGI sharpening and denoising only run when the camera itself demosaics,
        # so with a Bayer pixel_format they are read-only. A negative value means
        # "leave alone", which keeps startup free of pointless warnings. Set these
        # only alongside pixel_format RGB8 or BGR8.
        'sharpness_enhancement': -1.0,
        'noise_reduction': -1.0,

        # ------------------------------------------------ link and transport ---
        # 0 keeps the camera and pylon defaults (360 MB/s link limit,
        # 262144 byte transfers, 64 queued URBs). Lower the throughput limit when
        # several cameras share one USB controller.
        'device_link_throughput_limit': 0,   # bytes/s
        'max_transfer_size': 0,              # bytes
        'num_max_queued_urbs': 0,

        # --------------------------------------------------------- triggering --
        # Off free-runs. With On plus trigger_source Software, call the
        # ~/execute_software_trigger service for each frame.
        'trigger_mode': 'Off',
        'trigger_source': 'Software',       # Software, Line1, Line2, Line3, ...

        # --------------------------------------------------------- timestamps --
        # ros_time stamps when the host retrieves the frame: robust, but it
        # carries the host's scheduling jitter. camera uses the device counter
        # (1 tick = 1 ns on this model), which is jitter-free, re-anchored to ROS
        # time every timestamp_resync_period seconds so the camera and host
        # oscillators cannot drift apart without bound.
        'timestamp_source': 'ros_time',
        'timestamp_resync_period': 10.0,
        'timestamp_offset': 0.0,           # s, added to every stamp

        # -------------------------------------------------------- diagnostics --
        # Publishes DeviceTemperature on ~/temperature. 0 disables. This camera
        # idles around 46 C, and BslTemperatureStatus flags Critical well before
        # anything is damaged.
        'temperature_publish_period': 0.0,  # s

        # The node's own measured grab rate, which is the honest number when
        # checking whether a requested frame rate is met. `ros2 topic hz` reports
        # only what survived the transport.
        'stats_log_period': ParameterValue(
            LaunchConfiguration('stats_log_period'), value_type=float),
    }

    # The node publishes ace2/image_raw and ace2/camera_info by default, which land
    # under the namespace above. Change the right-hand side to move them.
    remappings = [
        ('ace2/image_raw', 'ace2/image_raw'),
        ('ace2/camera_info', 'ace2/camera_info'),
    ]

    # Its own process. This is the default and behaves exactly as it did before
    # the node became a component.
    standalone_node = launch_ros.actions.Node(
        condition=UnlessCondition(LaunchConfiguration('use_composition')),
        package='ace2_camera_wrapper',
        namespace=LaunchConfiguration('namespace'),
        executable='ace2_camera_wrapper_node',
        name='ace2_camera_wrapper',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=remappings,
    )

    # Loaded into a container instead. Anything else added to this container gets
    # the frames without a copy, provided it also enables intra-process comms.
    # Use component_container_mt if a component in here needs a multi-threaded
    # executor.
    composed_container = launch_ros.actions.ComposableNodeContainer(
        condition=IfCondition(LaunchConfiguration('use_composition')),
        name=LaunchConfiguration('container_name'),
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='ace2_camera_wrapper',
                plugin='ace2_camera_wrapper::Ace2CameraWrapperNode',
                name='ace2_camera_wrapper',
                namespace=LaunchConfiguration('namespace'),
                parameters=[params],
                remappings=remappings,
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    return LaunchDescription(launch_args + [
        launch_ros.actions.SetParameter(name='use_sim_time', value=False),
        standalone_node,
        composed_container,
    ])
