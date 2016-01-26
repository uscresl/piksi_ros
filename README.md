Piksi ROS
=========

A python based driver for SwiftNav's Piksi RTK GPS modules.

Features:
---------

* Publishes:
    * NavSatFix (separate ones for RTK fix and SPP fix)
    * Odometry (with positioning from RTK data)
    * TimeReference
    * as well as custom Observations and Ephemeris msgs.
* Can send/receive observations from another Piksi through UDP or Serial (for setups that don't use the external radios)
* Supports setting any of the settings on the piksi at startup (saving to flash is optional)
* Can publish TF transforms from UTM->base_station and/or base_station->rover
* Supplies diagnostic info.
* Keeps track of fix mode through watchdog timers.

Published Topics:
-------

* ~fix (sensor_msgs/NavSatFix). Best fix available.
* ~spp_fix (sensor_msgs/NavSatFix). Single point precision fix.
* ~rtk_fix (sensor_msgs/NavSatFix). RTK fix (when available).
* ~odom (nav_msgs/Odometry). RTK based odometry.
* ~time (sensor_msgs/TimeReference). GPS clock (does not match UTC as it does not account for leap seconds)
* ~observations (piksi_ros/Observations). Satellite observations. Enable through *publish_observations* parameter.
* ~ephemeris (piksi_ros/Ephemeris). Satellite observations. Enable through *publish_ephemeris* parameter.

Parameters:
-----------

Piksi stuff:

* **port** - The serial port device of the Piksi unit. *Default: /dev/ttyUSB0*.
* **baud_rate** - Baud rate for the serial port
* **piksi/setting_section/setting_name** - These can be used to change settings on the piksi at driver startup. Settings are not committed to device flash unless *piksi_save_settings* is set appropriately.
* **piksi_save_settings** - Commits piksi settings to device flash if set true. *Default: false*.

Frames:

* **frame_id** - Frame id for GPS messages
* **rtk_frame_id** - Frame id for the base.
* **utm_frame_id** - Frame id for an UTM coordinate frame.
* **child_frame_id** - Frame id for the rover (typeically base_link or base_link_stabilized)

Publish settings:

* **publish_utm_rtk_tf** -
* **publish_rtk_child_tf** -
* **publish_observations** -
* **publish_ephemeris** -

Driver settings:

* **rtk_fix_timeout** -
* **spp_fix_timeout** -
* **rtk_h_accuracy** -
* **rtk_v_accuracy** -

Diagnostics:

* **diag/heartbeat_freq** - Expected frequency for heartbeats.
* **diag/update_freq** - Expected update frequency.
* **diag/freq_tolerance** - Frequency tolerance.
* **diag/window_size** - Diagnostics window size.
* **diag/min_delay** - Min timestamp delay for topics.
* **diag/max_delay** - Max timestamp delay for topics.

Observation sending/receiving:

* **obs/udp/send** - Whether to send observations received from Piksi through UDP.
* **obs/udp/receive** - Whether to receive observations through UDP and forward to Piksi.
* **obs/udp/host** - UDP host/ip. Leave blank for receiver for easier configuration.
* **obs/udp/port** - UDP port to use.

* **obs/serial/send** - Whether to send observations received from Piksi through a serial port.
* **obs/serial/receive** - Whether to receive observations through a serial port and forward to Piksi.
* **obs/udp/port** - Serial port.
* **obs/udp/baud_rate** - Serial port baud rate.

Other:

* **sbp_log_file** - Specify a log file to log all incoming SBP packets from Piksi to.
* **debug** - Enable extra debug messages.

Published transforms
--------------------

* utm_frame_id -> rtk_frame_id
* rtk_frame_id -> child_frame_id

Todo:
-----

* Add/improve covariance estimate for published data.
* Add dynamic reconfigure support
* Rewrite in C++
