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
* Dynamic reconfigure support.

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

Parameters with * can be changed through dynamic reconfigure.

### Piksi stuff:

* **port** - The serial port device of the Piksi unit. *Default: /dev/ttyUSB0*.
* **baud_rate** - Baud rate for the serial port. *Default: 1000000*.

### Piksi hardware settings:

You can have the driver set settings on the piksi at startup. Settings are not committed to device flash unless *piksi_save_settings* is set appropriately.

* **piksi/SECTION/SETTING** - Possible SECTION and SETTING values can be found in Piksi documentation or in the piksi console (subset can be found in piksi_settings.yaml). *
* **piksi_save_settings** - Commits piksi settings to device flash if set true. *Default: false*.

The piksi settings as they were on driver startup can be accessed through the parameter server under *~piksi_original_settings/*.

### Frames:

* **frame_id** - Frame id for GPS messages. *Default: piksi*
* **rtk_frame_id** - Frame id for the base. *Default: rtk_gps*
* **utm_frame_id** - Frame id for an UTM coordinate frame. *Default: utm*
* **child_frame_id** - Frame id for the rover (typically base_link or base_link_stabilized). *Default: base_link*

### Publish settings:

* **publish_utm_rtk_tf** - Publish utm_frame_id->rtk_frame_id transform. *Default: False*
* **publish_rtk_child_tf** - Publish rtk_frame_id->child_frame_id transform. *Default: False*
* **publish_observations** - Publish observations (list of tracked satellites and related measurements). *Default: False*
* **publish_ephemeris** - Publish ephemeris information. *Default: False*

### Driver settings:

* **rtk_fix_timeout** - Time to wait from last RTK message received until switching to SPP positions for best fix. *Default: 0.2* *
* **spp_fix_timeout** - Time to wait from last SPP message receveid until declaring no-fix. *Default: 1.0* *
* **rtk_h_accuracy** - RTK horizontal sddev. *Default: 0.04* *
* **rtk_v_accuracy** - RTK vertical sddev. *Default: 0.12* *

### Diagnostics:

* **diag/heartbeat_freq** - Expected frequency for heartbeats. *Default: 1.0*
* **diag/update_freq** - Expected update frequency. *Default: 10.0*
* **diag/freq_tolerance** - Frequency tolerance. *Default: 0.1*
* **diag/window_size** - Diagnostics window size. *Default: 10*
* **diag/min_delay** - Min timestamp delay for topics. *Default: 0.0*
* **diag/max_delay** - Max timestamp delay for topics. *Default: 0.2*

### Observation sending/receiving:

* **obs/udp/send** - Whether to send observations received from Piksi through UDP. *Default: False*
* **obs/udp/receive** - Whether to receive observations through UDP and forward to Piksi. *Default: False*
* **obs/udp/host** - UDP host/ip. Leave blank for receiver for easier configuration. *Default: ''*
* **obs/udp/port** - UDP port to use. *Default: 50785*

* **obs/serial/send** - Whether to send observations received from Piksi through a serial port. *Default: False*
* **obs/serial/receive** - Whether to receive observations through a serial port and forward to Piksi. *Default: False*
* **obs/udp/port** - Serial port. *Default: ''*
* **obs/udp/baud_rate** - Serial port baud rate. *Default: 115200*

### Other:

* **sbp_log_file** - Specify a log file to log all incoming SBP packets from Piksi to. *Default: ''*
* **debug** - Enable extra debug messages. *Default: False*

Published transforms
--------------------

The following transforms are published if configured to do so:

* utm_frame_id -> rtk_frame_id
* rtk_frame_id -> child_frame_id


Issues/notes:
-------------

* Covariance info is either set by parameters or set by a hack. It does not reflect an instantanious estimate of the covariance.
* No error-checking when setting parameters.

Todo:
-----

* Add/improve covariance estimate for published data.
* Rewrite in C++
