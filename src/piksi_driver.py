#!/usr/bin/env python

PKG = "piksi_ros"

# TODO add rinex logging (copy from https://github.com/swift-nav/piksi_tools/blob/master/piksi_tools/console/observation_view.py)

# Import system libraries
import time
import sys
import os
import socket
import threading
from collections import deque, defaultdict
import yaml
import ast

# Import ROS libraries
import rospy
import roslib; roslib.load_manifest(PKG)
import rospkg
import diagnostic_updater
import diagnostic_msgs
import tf2_ros
from dynamic_reconfigure.server import Server, extract_params


# Import ROS messages
from sensor_msgs.msg import NavSatFix, TimeReference, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from diagnostic_msgs.msg import DiagnosticStatus

from piksi_ros.msg import Observations, Obs, Ephemeris
from piksi_ros.cfg import PiksiDriverConfig

# Import SBP libraries
from sbp.client.drivers.base_driver import BaseDriver
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.rotating_logger import RotatingFileLogger

# Import SBP message ids
from sbp.system import SBP_MSG_HEARTBEAT, SBP_MSG_STARTUP
from sbp.observation import SBP_MSG_OBS, SBP_MSG_BASE_POS, SBP_MSG_EPHEMERIS
from sbp.navigation import SBP_MSG_GPS_TIME, SBP_MSG_BASELINE_NED, SBP_MSG_VEL_NED, SBP_MSG_POS_LLH, SBP_MSG_DOPS
from sbp.logging import SBP_MSG_LOG
from sbp.settings import *

# Import other libraries
from pyproj import Proj

tree = lambda: defaultdict(tree)

def pretty(d, indent=0):
   for key, value in d.iteritems():
      print '\t' * indent + str(key)
      if isinstance(value, dict):
         pretty(value, indent+1)
      else:
         print '\t' * (indent+1) + str(value)

def extract_params_callback(config, parents=()):
    res = []
    for name, g in config.iteritems():
        new_parents = parents+(g['name'],)
        for p in g['parameters']:
            res.append((new_parents, p, g['parameters'][p]))
        res.extend(extract_params_callback(g['groups'], parents=new_parents))
    return res

"""
About dynamic reconfigure for this driver.
It turns out that dynamic reconfigure is one hell of a mess:
- There are limitations on what you can name things, as names are used to generate variable names somewhere along the way.
- While settings can be grouped in the DR config, the grouping acts only as a visual grouping, the parameters are not grouped onto the rosparam server but just saved to the nodes namespace (~param instead of ~group/param).
- You can't have two parameters with the same name but in separate groups (because of the issue above).
- Despite what the documentation says, you need to specify max/min for numeric values, if you don't then they will be set to some C++ method and config fails to load in python (probably works with C++ nodes though).
- You can't specify a namespace prefix for your DR parameters.

Because of these problems, we extend the DR server class to handle prefixes (see below).
We also name the piksi settings parameters as piksi__section__param in DR because there are multiple params in different sections called 'mode' for example.
On startup, the node will load parameters from ~piksi/ and set those settings on the Piksi unit. It will also update the ~dynamic_reconfigure/ namespace where we store the parameters.
"""
class GroupServer(Server):

    def __init__(self, type, callback, ns_prefix=(), nest_groups=True):
        self.ns_prefix = ns_prefix
        self.nest_groups = nest_groups
        super(GroupServer, self).__init__(type, callback)

    def _get_param_name(self, parents, name):
        if self.nest_groups:
            return '~' + '/'.join(self.ns_prefix+parents+(name,))
        else:
            return '~' + '/'.join(self.ns_prefix+(name,))

    def _get_config_name(self, parents, name):
        #return '__'.join(parents+(name,))
        return name

    def _extract_param_tree(self, desc, parents=()):
        res = []
        for g in desc['groups']:
            new_parents = parents+(g['name'],)
            for p in g['parameters']:
                res.append((new_parents, p))
            res.extend(self._extract_param_tree(g, parents=new_parents))
        return res

    def _copy_from_parameter_server(self):

        for parents, param in self._extract_param_tree(self.type.config_description):
            try:
                self.config[self._get_config_name(parents, param['name'])] = rospy.get_param(self._get_param_name(parents, param['name']))
            except KeyError:
                pass

    def _copy_to_parameter_server(self):
        for parents, param in self._extract_param_tree(self.type.config_description):
            rospy.set_param(self._get_param_name(parents, param['name']), self.config[self._get_config_name(parents,param['name'])])

    def _clamp(self, config):

        for param in extract_params(self.type.config_description):
            maxval = self.type.max[param['name']]
            minval = self.type.min[param['name']]
            val = config[param['name']]
            if val > maxval and maxval != "":
                config[param['name']] = maxval
            elif val < minval and minval != "":
                config[param['name']] = minval

def calculate_utm_zone(lat, lon):
    """ Determine the UTM zone that a lon-lat point falls in. Returns and
    integer and a string, either ('N') or ('S'). """
    if lat >= 0:
            hemi = 'N'
    else:
            hemi = 'S'
    zone = int((180 + lon) // 6) + 1

    return (zone, hemi)

# Driver class for handling UDP connections for SBP
class UDPDriver(BaseDriver):
    def __init__(self, host, port):
        self.buf = deque()
        self.handle = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.handle.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            #self.handle.connect((host, port))
            self.handle.bind(("", port))
        except socket.error, msg:
            pass
        super(UDPDriver, self).__init__(self.handle)
        self._write_lock = threading.Lock()

    def read(self, size):
        if len(self.buf) < size:
            try:
                data, addr = self.handle.recvfrom(4096)
                if not data:
                    raise IOError
                for d in data:
                    self.buf.append(d)
            except socket.error, msg:
                raise IOError

        res = ''.join([self.buf.popleft() for i in xrange(size)])
        return res

    def flush(self):
        pass

    def write(self, s):
        return
        """
        Write wrapper.
        Parameters
        ----------
        s : bytes
        Bytes to write
        """
        try:
            self._write_lock.acquire()
            self.handle.sendall(s)
        except socket.error, msg:
            raise IOError
        finally:
            self._write_lock.release()


#
# # Classes for reading/writing observations to serial/udp
#
class UDPSender(object):
    def __init__(self, host, port):
        self.handle = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = host
        self.port = port

    def send(self, msg, **metadata):
        self.handle.sendto(msg.pack(), (self.address, self.port))

class UDPReceiver(object):
    def __init__(self, host, port, callback):
        self._callback = callback
        self.driver = UDPDriver(host, port)
        self.framer = Framer(self.driver.read, None, verbose=False)
        self.piksi = Handler(self.framer)
        self.piksi.add_callback(self.callback)
        self.piksi.start()
    def callback(self, msg, **metadata):
        self._callback(msg, **metadata)

class SerialSender(object):
    def __init__(self, port, baud_rate):
        import serial
        self.serial = serial.serial_for_url(port)
        self.serial.baudrate = baud
        self.serial.timeout = 1


    def send(self, msg, **metadata):
        self.serial.write(msg.pack())

class SerialReceiver(object):
    def __init__(self, port, baud_rate, callback):
        self._callback = callback
        self.driver = PySerialDriver(port, baud=baud_rate)
        self.framer = Framer(self.driver.read, None, verbose=False)
        self.piksi = Handler(self.framer)
        self.piksi.add_callback(self.callback)
        self.piksi.start()

    def callback(self, msg, **metadata):
        self._callback(msg, **metadata)

rospack = rospkg.RosPack()
PKG_PATH = rospack.get_path(PKG)

GPS_EPOCH = time.mktime((1980, 1, 6, 0, 0, 0, 0, 0, 0))
WEEK_SECONDS = 60*60*24*7

def ros_time_from_sbp_time(msg):
    return rospy.Time(GPS_EPOCH + msg.wn*WEEK_SECONDS + msg.tow * 1E-3 + msg.ns * 1E-9)

def nested_dict_iter(d,res=[],parents=()):
    for k,v in d.iteritems():
        if isinstance(v, dict):
            nested_dict_iter(v, res, parents+(k,))
        else:
            res.append((parents+(k,), v))
    return res

# Maps piksi error levels to ROS logging functions
PIKSI_LOG_LEVELS_TO_ROS = {
    0: rospy.logfatal,
    1: rospy.logerr,
    2: rospy.logerr,
    3: rospy.logerr,
    4: rospy.logwarn,
    5: rospy.loginfo,
    6: rospy.loginfo,
    7: rospy.logdebug
}

# Description text for some piksi flags
FIX_MODE = {0: "Single Point Positioning [0]", 1: "Fixed RTK [1]", 2: "Float RTK [2]", -1: "No fix"}
HEIGHT_MODE = {0: "Height above WGS84 ellipsoid [0]", 1: "Height above mean sea level [1]"}
RAIM_AVAILABILITY = {0: "Disabled/unavailable [0]", 1: "Available [1]"}
RAIM_REPAIR = {0: "No repair [0]", 1: "Solution from RAIM repair [1]"}

# Covariance for non-observed quantities
COV_NOT_MEASURED = 1000.0

class PiksiROS(object):
    def __init__(self):

        rospy.init_node("piksi_ros")

        self.send_observations = False
        self.serial_number = None

        self.last_baseline = None
        self.last_vel = None
        self.last_pos = None
        self.last_dops = None
        self.last_rtk_pos = None

        self.last_spp_time = 0
        self.last_rtk_time = 0
        self.fix_mode = -1
        self.rtk_fix_mode = 2

        self.read_piksi_settings_info()

        self.piksi_settings = tree()

        self.proj = None

        self.disconnect_piksi()

        self.read_params()

        self.setup_comms()

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.rtk_frame_id
        self.odom_msg.child_frame_id = self.child_frame_id

        self.odom_msg.pose.covariance[0] = self.rtk_h_accuracy**2
        self.odom_msg.pose.covariance[7] = self.rtk_h_accuracy**2
        self.odom_msg.pose.covariance[14] = self.rtk_v_accuracy**2
        self.odom_msg.twist.covariance[0] = self.odom_msg.pose.covariance[0] * 4
        self.odom_msg.twist.covariance[7] = self.odom_msg.pose.covariance[7] * 4
        self.odom_msg.twist.covariance[14] = self.odom_msg.pose.covariance[14] * 4

        self.odom_msg.pose.covariance[21] = COV_NOT_MEASURED
        self.odom_msg.pose.covariance[28] = COV_NOT_MEASURED
        self.odom_msg.pose.covariance[35] = COV_NOT_MEASURED
        self.odom_msg.twist.covariance[21] = COV_NOT_MEASURED
        self.odom_msg.twist.covariance[28] = COV_NOT_MEASURED
        self.odom_msg.twist.covariance[35] = COV_NOT_MEASURED

        self.transform = TransformStamped()
        self.transform.header.frame_id = self.utm_frame_id
        self.transform.child_frame_id = self.rtk_frame_id
        self.transform.transform.rotation = Quaternion(x=0,y=0,z=0,w=1)

        # Used to publish transform from rtk_frame_id -> child_frame_id
        self.base_link_transform = TransformStamped()
        self.base_link_transform.header.frame_id = self.rtk_frame_id
        self.base_link_transform.child_frame_id = self.child_frame_id
        self.base_link_transform.transform.rotation = Quaternion(x=0,y=0,z=0,w=1)

        self.diag_updater = diagnostic_updater.Updater()
        self.heartbeat_diag = diagnostic_updater.FrequencyStatus(diagnostic_updater.FrequencyStatusParam({'min':self.diag_heartbeat_freq, 'max':self.diag_heartbeat_freq}, self.diag_freq_tolerance, self.diag_window_size))
        self.diag_updater.add("Piksi status", self.diag)
        self.diag_updater.add(self.heartbeat_diag)

        self.setup_pubsub()

    def read_piksi_settings_info(self):
        self.piksi_settings_info = tree()
        settings_info = yaml.load(open(os.path.join(PKG_PATH, 'piksi_settings.yaml'), 'r'))
        for s in settings_info:

            if s['type'].lower() == 'boolean':
                s['parser'] = lambda x: x.lower()=='true'
            elif s['type'].lower() in ('float', 'double','int'):
                s['parser'] = ast.literal_eval
            elif s['type'] == 'enum':
                s['parser'] = s['enum'].index
            else:
                s['parser'] = lambda x: x

            self.piksi_settings_info[s['group']][s['name']] = s

    def init_proj(self, latlon):
        self.proj = Proj(proj='utm',zone=calculate_utm_zone(*latlon)[0],ellps='WGS84')

    def reconfig_callback(self, config, level):
        for p,v in config.iteritems():
            if p=='groups':
                continue
            n = p.split('__')
            if self.piksi_settings[n[1]][n[2]] != v:
                i = self.piksi_settings_info[n[1]][n[2]]
                p_val = v
                if i['type'] == 'enum':
                    try:
                        p_val = i['enum'][v]
                    except:
                        p_val = i['enum'][0]
                self.piksi_set(n[1],n[2],p_val)
                self.piksi_settings[n[1]][n[2]] = v
        return config

    def read_params(self):
        self.debug = rospy.get_param('~debug', False)
        self.frame_id = rospy.get_param('~frame_id', "piksi")
        self.rtk_frame_id = rospy.get_param('~rtk_frame_id', "rtk_gps")
        self.utm_frame_id = rospy.get_param('~utm_frame_id', "utm")
        self.child_frame_id = rospy.get_param('~child_frame_id', "base_link")
        self.piksi_port = rospy.get_param('~port', "/dev/ttyUSB0")

        self.publish_utm_rtk_tf = rospy.get_param('~publish_utm_rtk_tf', False)
        self.publish_rtk_child_tf = rospy.get_param('~publish_rtk_child_tf', False)

        self.diag_heartbeat_freq = rospy.get_param('~diag/heartbeat_freq', 1.0)
        self.diag_update_freq = rospy.get_param('~diag/update_freq', 10.0)
        self.diag_freq_tolerance = rospy.get_param('~diag/freq_tolerance', 0.1)
        self.diag_window_size = rospy.get_param('~diag/window_size', 10)
        self.diag_min_delay = rospy.get_param('~diag/min_delay', 0.0)
        self.diag_max_delay = rospy.get_param('~diag/max_delay', 0.2)

        # Send/receive observations through udp
        self.obs_udp_send = rospy.get_param('~obs/udp/send', False)
        self.obs_udp_recv = rospy.get_param('~obs/udp/receive', False)
        self.obs_udp_host = rospy.get_param('~obs/udp/host', "")
        self.obs_udp_port = rospy.get_param('~obs/udp/port', 50785)

        # Send/receive observations through serial port
        self.obs_serial_send = rospy.get_param('~obs/serial/send', False)
        self.obs_serial_recv = rospy.get_param('~obs/serial/receive', False)
        self.obs_serial_port = rospy.get_param('~obs/serial/port', None)
        self.obs_serial_baud_rate = rospy.get_param('~obs/serial/baud_rate', 115200)

        self.rtk_h_accuracy = rospy.get_param("~rtk_h_accuracy", 0.04)
        self.rtk_v_accuracy = rospy.get_param("~rtk_v_accuracy", self.rtk_h_accuracy*3)

        self.sbp_log = rospy.get_param('~sbp_log_file', None)

        self.rtk_fix_timeout = rospy.get_param('~rtk_fix_timeout', 0.2)
        self.spp_fix_timeout = rospy.get_param('~spp_fix_timeout', 1.0)

        self.publish_ephemeris = rospy.get_param('~publish_ephemeris', False)
        self.publish_observations = rospy.get_param('~publish_observations', False)

        self.piksi_update_settings = rospy.get_param('~piksi',{})
        self.piksi_save_settings = rospy.get_param('~piksi_save_settings', False)

    def setup_comms(self):
        self.obs_senders = []
        self.obs_receivers = []

        if self.obs_udp_send or self.obs_serial_send:
            self.send_observations = True

        if self.obs_udp_send:
            self.obs_senders.append(UDPSender(self.obs_udp_host, self.obs_udp_port))

        if self.obs_serial_send:
            self.obs_senders.append(SerialSender(self.obs_serial_port, self.obs_serial_baud_rate))

        if self.obs_udp_recv:
            self.obs_receivers.append(UDPReceiver(self.obs_udp_host, self.obs_udp_port, self.callback_external))

        if self.obs_serial_recv:
            self.obs_receivers.append(SerialReceiver(self.obs_serial_port, self.obs_serial_baud_rate, self.callback_external))

    def callback_external(self, msg, **metadata):
        if self.debug:
            rospy.loginfo("Received external SBP msg.")

        if self.piksi_framer:
            self.piksi_framer(msg, **metadata)
        else:
            rospy.logwarn("Received external SBP msg, but Piksi not connected.")

    def setup_pubsub(self):

        freq_params = diagnostic_updater.FrequencyStatusParam({'min':self.diag_update_freq, 'max':self.diag_update_freq}, self.diag_freq_tolerance, self.diag_window_size)
        time_params = diagnostic_updater.TimeStampStatusParam(self.diag_min_delay, self.diag_max_delay)

        self.pub_fix = rospy.Publisher("~fix", NavSatFix, queue_size=1000)

        self.pub_spp_fix = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~spp_fix", NavSatFix, queue_size=1000), self.diag_updater, freq_params, time_params)

        self.pub_rtk_fix = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~rtk_fix", NavSatFix, queue_size=1000), self.diag_updater, freq_params, time_params)
        #self.pub_rtk = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~rtk_odom", Odometry, queue_size=1000), self.diag_updater, freq_params, time_params)
        self.pub_odom = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~odom", Odometry, queue_size=1000), self.diag_updater, freq_params, time_params)
        self.pub_time = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~time", TimeReference, queue_size=1000), self.diag_updater, freq_params, time_params)

        if self.publish_utm_rtk_tf or self.publish_rtk_child_tf:
            self.tf_br = tf2_ros.TransformBroadcaster()

        if self.publish_ephemeris:
            self.pub_eph = rospy.Publisher("~ephemeris", Ephemeris, queue_size=1000)

        if self.publish_observations:
            self.pub_obs = rospy.Publisher('~observations', Observations, queue_size=1000)


    def piksi_start(self):

        self.dr_srv = GroupServer(PiksiDriverConfig, self.reconfig_callback, ns_prefix=('dynamic_reconfigure',), nest_groups=False)

        self.piksi.add_callback(self.callback_sbp_gps_time, msg_type=SBP_MSG_GPS_TIME)
        self.piksi.add_callback(self.callback_sbp_dops, msg_type=SBP_MSG_DOPS)

        self.piksi.add_callback(self.callback_sbp_pos, msg_type=SBP_MSG_POS_LLH)
        self.piksi.add_callback(self.callback_sbp_baseline, msg_type=SBP_MSG_BASELINE_NED)
        self.piksi.add_callback(self.callback_sbp_vel, msg_type=SBP_MSG_VEL_NED)

        #if self.send_observations:
        self.piksi.add_callback(self.callback_sbp_obs, msg_type=SBP_MSG_OBS)
        self.piksi.add_callback(self.callback_sbp_base_pos, msg_type=SBP_MSG_BASE_POS)
        if self.publish_ephemeris:
            self.piksi.add_callback(self.callback_sbp_ephemeris, msg_type=SBP_MSG_EPHEMERIS)

        if self.sbp_log is not None:
            self.sbp_logger = RotatingFileLogger(self.sbp_log, when='M', interval=60, backupCount=0)
            self.piksi.add_callback(self.sbp_logger)

    def connect_piksi(self):
        self.piksi_driver = PySerialDriver(self.piksi_port, baud=1000000)
        self.piksi_framer = Framer(self.piksi_driver.read, self.piksi_driver.write, verbose=False)
        self.piksi = Handler(self.piksi_framer)

        # Only setup log and settings messages
        # We will wait to set up rest until piksi is configured
        self.piksi.add_callback(self.callback_sbp_heartbeat, msg_type=SBP_MSG_HEARTBEAT)
        self.piksi.add_callback(self.callback_sbp_log, msg_type=SBP_MSG_LOG)
        self.piksi.add_callback(self.callback_sbp_settings_read_resp, msg_type=SBP_MSG_SETTINGS_READ_RESP)
        self.piksi.add_callback(self.callback_sbp_settings_read_by_index_resp, msg_type=SBP_MSG_SETTINGS_READ_BY_INDEX_REQ)
        self.piksi.add_callback(self.callback_sbp_settings_read_by_index_resp, msg_type=SBP_MSG_SETTINGS_READ_BY_INDEX_RESP)
        self.piksi.add_callback(self.callback_sbp_settings_read_by_index_done, msg_type=SBP_MSG_SETTINGS_READ_BY_INDEX_DONE)

        self.piksi.add_callback(self.callback_sbp_startup, SBP_MSG_STARTUP)

        self.piksi.start()

    def disconnect_piksi(self):
        try:
            self.piksi.stop()
            self.piksi.remove_callback()
        except:
            pass
        self.piksi = None
        self.piksi_framer = None
        self.piksi_driver = None

    def set_serial_number(self, serial_number):
        rospy.loginfo("Connected to piksi #%d" % serial_number)
        self.serial_number = serial_number
        self.diag_updater.setHardwareID("Piksi %d" % serial_number)

    def read_piksi_settings(self):
        self.settings_index = 0
        self.piksi_framer(MsgSettingsReadByIndexReq(index=self.settings_index))
        #self.piksi_framer(MsgSettingsReadReq(setting='simulator\0enabled\0'))

    def piksi_set(self, section, setting, value):
        m = MsgSettingsWrite(setting="%s\0%s\0%s\0" % (section, setting, value))
        self.piksi_framer(m)

    def update_dr_param(self, section, name, value):
        #print 'set %s:%s to %s' % (section, name, value)
        #print '~dynamic_reconfigure/piksi__%s__%s' % (section, name), value
        rospy.set_param('~dynamic_reconfigure/piksi__%s__%s' % (section, name), value)

    def set_piksi_settings(self):
        save_needed = False
        for s in nested_dict_iter(self.piksi_update_settings):
            cval = self.piksi_settings[s[0][0]][s[0][1]]
            if len(cval) != 0:
                if cval != str(s[1]):
                    rospy.loginfo('Updating piksi setting %s:%s to %s.' % (s[0][0], s[0][1], s[1]))
                    self.piksi_set(s[0][0], s[0][1], s[1])
                    self.update_dr_param(s[0][0], s[0][1], s[1])
                    save_needed = True
                else:
                    rospy.loginfo('Piksi setting %s:%s already set to %s.' % (s[0][0], s[0][1], s[1]))
            else:
                rospy.logwarn('Invalid piksi setting: %s' % ':'.join(s[0]))

        if self.piksi_save_settings and save_needed:
            rospy.loginfo('Saving piksi settings to flash')
            m = MsgSettingsSave()
            self.piksi_framer(m)

    def callback_sbp_startup(self, msg, **metadata):
        rospy.loginfo('Piksi startup packet received: %s' % repr(msg))

    def callback_sbp_gps_time(self, msg, **metadata):
        if self.debug:
            rospy.loginfo("Received SBP_MSG_GPS_TIME (Sender: %d): %s" % (msg.sender, repr(msg)))

        out = TimeReference()
        out.header.frame_id = self.frame_id
        out.header.stamp = rospy.Time.now()
        out.time_ref = ros_time_from_sbp_time(msg)
        out.source = "gps"
        self.pub_time.publish(out)

    def callback_sbp_heartbeat(self, msg, **metadata):
        if self.debug:
            rospy.loginfo("Received SBP_MSG_HEARTBEAT (Sender: %d): %s" % (msg.sender, repr(msg)))

        self.heartbeat_diag.tick()
        if self.serial_number is None:
            self.set_serial_number(msg.sender)
            self.read_piksi_settings()


    def callback_sbp_dops(self, msg, **metadata):
        if self.debug:
            rospy.loginfo("Received SBP_MSG_DOPS (Sender: %d): %s" % (msg.sender, repr(msg)))

        self.last_dops = msg

    def callback_sbp_pos(self, msg, **metadata):
        if self.debug:
            rospy.loginfo("Received SBP_MSG_POS_LLH (Sender: %d): %s" % (msg.sender, repr(msg)))

        out = NavSatFix()
        out.header.frame_id = self.frame_id
        out.header.stamp = rospy.Time.now()

        out.status.service = NavSatStatus.SERVICE_GPS

        out.latitude = msg.lat
        out.longitude = msg.lon
        out.altitude = msg.height

        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        if msg.flags & 0x03:
            self.last_rtk_time = time.time()
            self.rtk_fix_mode = msg.flags & 0x03

            out.status.status = NavSatStatus.STATUS_GBAS_FIX

            # TODO this should probably also include covariance of base fix?
            out.position_covariance[0] = self.rtk_h_accuracy**2
            out.position_covariance[4] = self.rtk_h_accuracy**2
            out.position_covariance[8] = self.rtk_v_accuracy**2

            pub = self.pub_rtk_fix
            self.last_rtk_pos = msg

            # If we are getting this message, RTK is our best fix, so publish this as our best fix.
            self.pub_fix.publish(out)
        else:

            self.last_spp_time = time.time()

            out.status.status = NavSatStatus.STATUS_FIX

            # TODO hack, piksi should provide these numbers
            if self.last_dops:
                out.position_covariance[0] = (self.last_dops.hdop * 1E-2)**2
                out.position_covariance[4] = (self.last_dops.hdop * 1E-2)**2
                out.position_covariance[8] = (self.last_dops.vdop * 1E-2)**2
            else:
                out.position_covariance[0] = COV_NOT_MEASURED
                out.position_covariance[4] = COV_NOT_MEASURED
                out.position_covariance[8] = COV_NOT_MEASURED

            pub = self.pub_spp_fix
            self.last_pos = msg

            # Check if SPP is currently our best available fix
            if self.rtk_fix_mode <= 0:
                self.pub_fix.publish(out)

        pub.publish(out)

    def publish_odom(self):
        if self.last_baseline is None or self.last_vel is None:
            return

        if self.last_baseline.tow == self.last_vel.tow:
            self.odom_msg.header.stamp = rospy.Time.now()

            self.odom_msg.pose.pose.position.x = self.last_baseline.e/1000.0
            self.odom_msg.pose.pose.position.y = self.last_baseline.n/1000.0
            self.odom_msg.pose.pose.position.z = -self.last_baseline.d/1000.0

            self.odom_msg.twist.twist.linear.x = self.last_vel.e/1000.0
            self.odom_msg.twist.twist.linear.y = self.last_vel.n/1000.0
            self.odom_msg.twist.twist.linear.z = -self.last_vel.d/1000.0

            self.pub_odom.publish(self.odom_msg)

    def callback_sbp_vel(self, msg, **metadata):
        if self.debug:
            rospy.loginfo("Received SBP_MSG_VEL_NED (Sender: %d): %s" % (msg.sender, repr(msg)))
        self.last_vel = msg
        self.publish_odom()

    def callback_sbp_baseline(self, msg, **metadata):
        if self.debug:
            rospy.loginfo("Received SBP_MSG_BASELINE_NED (Sender: %d): %s" % (msg.sender, repr(msg)))

        if self.publish_rtk_child_tf:
            self.base_link_transform.header.stamp = rospy.Time.now()
            self.base_link_transform.transform.translation.x = msg.e/1000.0
            self.base_link_transform.transform.translation.y = msg.n/1000.0
            self.base_link_transform.transform.translation.z = -msg.d/1000.0
            self.tf_br.sendTransform(self.base_link_transform)

        self.last_baseline = msg
        self.publish_odom()

    def callback_sbp_ephemeris(self, msg, **metadata):
        if self.debug:
            rospy.loginfo("Received SBP_MSG_EPHEMERIS (Sender: %d): %s" % (msg.sender, repr(msg)))

        if not hasattr(self, 'eph_msg'):
            self.eph_msg = Ephemeris()
        self.eph_msg.header.stamp = rospy.Time.now()
        self.eph_msg.header.frame_id = self.frame_id

        self.eph_msg.tgd = msg.tgd
        self.eph_msg.c_rs = msg.c_rs
        self.eph_msg.c_rc = msg.c_rc
        self.eph_msg.c_uc = msg.c_uc
        self.eph_msg.c_us = msg.c_us
        self.eph_msg.c_ic = msg.c_ic
        self.eph_msg.c_is = msg.c_is
        self.eph_msg.dn = msg.dn
        self.eph_msg.m0 = msg.m0
        self.eph_msg.ecc = msg.ecc
        self.eph_msg.sqrta = msg.sqrta
        self.eph_msg.omega0 = msg.omega0
        self.eph_msg.omegadot = msg.omegadot
        self.eph_msg.w = msg.w
        self.eph_msg.inc = msg.inc
        self.eph_msg.inc_dot = msg.inc_dot
        self.eph_msg.af0 = msg.af0
        self.eph_msg.af1 = msg.af1
        self.eph_msg.af2 = msg.af2
        self.eph_msg.toe_tow = msg.toe_tow
        self.eph_msg.toe_wn = msg.toe_wn
        self.eph_msg.toc_tow = msg.toc_tow
        self.eph_msg.toc_wn = msg.toc_wn
        self.eph_msg.valid = msg.valid
        self.eph_msg.healthy = msg.healthy
        self.eph_msg.sid.sat = msg.sid.sat
        self.eph_msg.sid.band = msg.sid.band
        self.eph_msg.sid.constellation = msg.sid.constellation
        self.eph_msg.iode = msg.iode
        self.eph_msg.iodc = msg.iodc

        self.pub_eph.publish(m)

    def callback_sbp_obs(self, msg, **metadata):
        if self.debug:
            rospy.loginfo("Received SBP_MSG_OBS (Sender: %d): %s" % (msg.sender, repr(msg)))

        if self.send_observations:
            for s in self.obs_senders:
                s.send(msg)

        if self.publish_observations:
            if not hasattr(self, 'obs_msg'):
                self.obs_msg = Observations()

            # Need to do some accounting to figure out how many sbp packets to expect
            num_packets = (msg.header.n_obs >> 4) & 0x0F
            packet_no = msg.header.n_obs & 0x0F

            if self.obs_msg.tow != msg.header.t.tow:

                self.obs_msg.header.stamp = rospy.Time.now()
                self.obs_msg.header.frame_id = self.frame_id

                self.obs_msg.tow = msg.header.t.tow
                self.obs_msg.wn = msg.header.t.wn
                self.obs_msg.n_obs = 0

                self.obs_msg.obs = []

            # lets use this field to count how many packets we have so far
            self.obs_msg.n_obs += 1

            for obs in msg.obs:
                x = Obs()
                x.P = obs.P
                x.L.i = obs.L.i
                x.L.f = obs.L.f
                x.cn0 = obs.cn0
                x.lock = obs.lock
                x.sid.sat = obs.sid.sat
                x.sid.band = obs.sid.band
                x.sid.constellation = obs.sid.constellation
                self.obs_msg.obs.append(x)

            if num_packets == self.obs_msg.n_obs:
                # Now use the field to indicate how many observations we have
                self.obs_msg.n_obs = len(self.obs_msg.obs)

                self.pub_obs.publish(self.obs_msg)


    def callback_sbp_base_pos(self, msg, **metadata):

        if self.debug:
            rospy.loginfo("Received SBP_MSG_BASE_POS (Sender: %d): %s" % (msg.sender, repr(msg)))

        if self.send_observations:
            for s in self.obs_senders:
                s.send(msg)

        # publish tf for rtk frame
        if self.publish_utm_rtk_tf:
            if not self.proj:
                self.init_proj((msg.lat, msg.lon))

            E,N = self.proj(msg.lon,msg.lat, inverse=False)

            self.transform.header.stamp = rospy.Time.now()
            self.transform.transform.translation.x = E
            self.transform.transform.translation.y = N
            self.transform.transform.translation.z = -msg.height
            self.tf_br.sendTransform(self.transform)

    def callback_sbp_settings_read_resp(self, msg, **metadata):
        pass


    def callback_sbp_settings_read_by_index_resp(self, msg, **metadata):
        p = msg.setting.split('\0')
        try:
            i = self.piksi_settings_info[p[0]][p[1]]
            p[2] = i['parser'](p[2])
        except:
            pass

        rospy.set_param('~piksi_original_settings/%s/%s' % (p[0],p[1]), p[2])
        self.piksi_settings[p[0]][p[1]] = p[2]
        self.update_dr_param(p[0], p[1], p[2])


        self.settings_index += 1
        self.piksi_framer(MsgSettingsReadByIndexReq(index=self.settings_index))

    def callback_sbp_settings_read_by_index_done(self, msg, **metadata):
        self.set_piksi_settings()
        self.piksi_start()

    def callback_sbp_log(self, msg, **metadata):
        PIKSI_LOG_LEVELS_TO_ROS[msg.level]("Piksi LOG: %s" % msg.text)

    def check_timeouts(self):
        if time.time() - self.last_rtk_time > self.rtk_fix_timeout:
            if time.time() - self.last_spp_time > self.spp_fix_timeout:
                self.fix_mode = -1
            else:
                self.fix_mode = 0
        else:
            self.fix_mode = self.rtk_fix_mode

    def diag(self, stat):
        fix_mode = self.fix_mode
        num_sats = 0
        last_pos = None

        if self.last_rtk_pos is not None:
            last_pos = self.last_rtk_pos
        elif self.last_pos is not None:
            last_pos = self.last_pos

        if last_pos:
            stat.add("GPS latitude", last_pos.lat)
            stat.add("GPS longitude", last_pos.lon)
            stat.add("GPS altitude", last_pos.height)
            stat.add("GPS #sats", last_pos.n_sats)
            num_sats = last_pos.n_sats

            stat.add("Height mode", HEIGHT_MODE[(last_pos.n_sats & 0x04) >> 2])
            stat.add("RAIM availability", RAIM_AVAILABILITY[(last_pos.n_sats & 0x08) >> 3])
            stat.add("RAIM repair", RAIM_REPAIR[(last_pos.n_sats & 0x10) >> 4])

        stat.add("Fix mode", FIX_MODE[self.fix_mode])

        if self.last_vel:
            stat.add("Velocity N", self.last_vel.n * 1E-3)
            stat.add("Velocity E", self.last_vel.e * 1E-3)
            stat.add("Velocity D", self.last_vel.d * 1E-3)
            stat.add("Velocity #sats", self.last_vel.n_sats)

        if self.last_baseline:
            stat.add("Baseline N", self.last_baseline.n * 1E-3)
            stat.add("Baseline E", self.last_baseline.e * 1E-3)
            stat.add("Baseline D", self.last_baseline.d * 1E-3)
            stat.add("Baseline #sats", self.last_baseline.n_sats)
            stat.add("Baseline fix mode", FIX_MODE[self.last_baseline.flags & 0x03])

        if self.last_dops:
            stat.add("GDOP", self.last_dops.gdop * 1E-2)
            stat.add("PDOP", self.last_dops.pdop * 1E-2)
            stat.add("TDOP", self.last_dops.tdop * 1E-2)
            stat.add("HDOP", self.last_dops.hdop * 1E-2)
            stat.add("VDOP", self.last_dops.vdop * 1E-2)

        if not self.piksi:
            stat.summary(DiagnosticStatus.ERROR, "Piksi not connected")
        elif fix_mode < 0:
            stat.summary(DiagnosticStatus.ERROR, "No fix")
        elif fix_mode < 1:
            stat.summary(DiagnosticStatus.WARN, "No RTK fix")
        elif num_sats < 5:
            stat.summary(DiagnosticStatus.WARN, "Low number of satellites in view")
        else:
            stat.summary(DiagnosticStatus.OK, "Piksi connected")

    def spin(self):
        reconnect_delay = 1.0
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Connecting to SwiftNav Piksi on port %s" % self.piksi_port)
                self.connect_piksi()

                while not rospy.is_shutdown():
                    rospy.sleep(0.05)
                    if not self.piksi.is_alive():
                        raise IOError
                    self.diag_updater.update()
                    self.check_timeouts()

                break # should only happen if rospy is trying to shut down
            except IOError as e:
                rospy.logerr("IOError")
                self.disconnect_piksi()
            except SystemExit as e:
                rospy.logerr("Unable to connect to Piksi on port %s" % self.piksi_port)
                self.disconnect_piksi()
            except: # catch *all* exceptions
                e = sys.exc_info()[0]
                rospy.logerr("Uncaught error: %s" % repr(e))
                self.disconnect_piksi()
            rospy.loginfo("Attempting to reconnect in %fs" % reconnect_delay)
            rospy.sleep(reconnect_delay)

def main():
    node = PiksiROS()
    node.spin()

if __name__ == "__main__":
  main()
