#!/usr/bin/env python

PKG = "piksi_ros"

# TODO add rinex logging (copy from https://github.com/swift-nav/piksi_tools/blob/master/piksi_tools/console/observation_view.py)
# TODO finish obs to/from serial port

# Import system libraries
import time
import sys
import socket
import threading
from collections import deque

# Import ROS libraries
import rospy
import roslib; roslib.load_manifest(PKG)
import diagnostic_updater
import diagnostic_msgs
import tf2_ros

# Import ROS messages
from sensor_msgs.msg import NavSatFix, TimeReference, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from diagnostic_msgs.msg import DiagnosticStatus

# Import SBP libraries
from sbp.client.drivers.base_driver import BaseDriver
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer

# Import SBP message ids
from sbp.system import SBP_MSG_HEARTBEAT
from sbp.observation import SBP_MSG_OBS, SBP_MSG_BASE_POS
from sbp.navigation import SBP_MSG_GPS_TIME, SBP_MSG_BASELINE_NED, SBP_MSG_VEL_NED, SBP_MSG_POS_LLH, SBP_MSG_DOPS

# Import other libraries
from pyproj import Proj

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
        raise NotImplemented
    def send(self, msg, **metadata):
        raise NotImplemented

class SerialReceiver(object):
    def __init__(self, port, baud_rate, callback):
        raise NotImplemented

GPS_EPOCH = time.mktime((1980, 1, 6, 0, 0, 0, 0, 0, 0))
WEEK_SECONDS = 60*60*24*7

def ros_time_from_sbp_time(msg):
    return rospy.Time(GPS_EPOCH + msg.wn*WEEK_SECONDS + msg.tow * 1E-3 + msg.ns * 1E-9)

FIX_MODE = {0: "Single Point Positioning [0]", 1: "Fixed RTK [1]", 2: "Float RTK [2]", -1: "No fix"}
HEIGHT_MODE = {0: "Height above WGS84 ellipsoid [0]", 1: "Height above mean sea level [1]"}
RAIM_AVAILABILITY = {0: "Disabled/unavailable [0]", 1: "Available [1]"}
RAIM_REPAIR = {0: "No repair [0]", 1: "Solution from RAIM repair [1]"}

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
        self.transform.header.frame_id = self.rtk_frame_id

        self.diag_updater = diagnostic_updater.Updater()
        self.heartbeat_diag = diagnostic_updater.FrequencyStatus(diagnostic_updater.FrequencyStatusParam({'min':self.diag_heartbeat_freq, 'max':self.diag_heartbeat_freq}, self.diag_freq_tolerance, self.diag_window_size))
        self.diag_updater.add("Piksi status", self.diag)
        self.diag_updater.add(self.heartbeat_diag)

        self.setup_pubsub()

    def init_proj(self, latlon):
        self.proj = Proj(proj='utm',zone=calculate_utm_zone(*latlon)[0],ellps='WGS84')

    def read_params(self):
        self.debug = rospy.get_param('~debug', False)
        self.frame_id = rospy.get_param('~frame_id', "gps")
        self.rtk_frame_id = rospy.get_param('~rtk_frame_id', "rtk_gps")
        self.utm_frame_id = rospy.get_param('~utm_frame_id', "utm")
        self.child_frame_id = rospy.get_param('~child_frame_id', "base_link")
        self.piksi_port = rospy.get_param('~port', "/dev/ttyUSB0")

        self.publish_tf = rospy.get_param('~publish_tf', False)

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
        self.obs_serial_baud_rate = rospy.get_param('~obs/serial/baud_rate', None)

        self.rtk_h_accuracy = rospy.get_param("~rtk_h_accuracy", 0.04)
        self.rtk_v_accuracy = rospy.get_param("~rtk_h_accuracy", self.rtk_h_accuracy*3)

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

        self.pub_fix = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~spp_fix", NavSatFix, queue_size=1000), self.diag_updater, freq_params, time_params)

        self.pub_rtk_fix = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~fix", NavSatFix, queue_size=1000), self.diag_updater, freq_params, time_params)
        #self.pub_rtk = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~rtk_odom", Odometry, queue_size=1000), self.diag_updater, freq_params, time_params)
        self.pub_odom = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~odom", Odometry, queue_size=1000), self.diag_updater, freq_params, time_params)
        self.pub_time = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~time", TimeReference, queue_size=1000), self.diag_updater, freq_params, time_params)

        if self.publish_tf:
            self.tf_br = tf2_ros.TransformBroadcaster()


    def connect_piksi(self):
        self.piksi_driver = PySerialDriver(self.piksi_port, baud=1000000)
        self.piksi_framer = Framer(self.piksi_driver.read, self.piksi_driver.write, verbose=False)
        self.piksi = Handler(self.piksi_framer)

        self.piksi.add_callback(self.callback_sbp_heartbeat, msg_type=SBP_MSG_HEARTBEAT)
        self.piksi.add_callback(self.callback_sbp_gps_time, msg_type=SBP_MSG_GPS_TIME)
        self.piksi.add_callback(self.callback_sbp_dops, msg_type=SBP_MSG_DOPS)

        self.piksi.add_callback(self.callback_sbp_pos, msg_type=SBP_MSG_POS_LLH)
        self.piksi.add_callback(self.callback_sbp_baseline, msg_type=SBP_MSG_BASELINE_NED)
        self.piksi.add_callback(self.callback_sbp_vel, msg_type=SBP_MSG_VEL_NED)

        #if self.send_observations:
        self.piksi.add_callback(self.callback_sbp_obs, msg_type=SBP_MSG_OBS)
        self.piksi.add_callback(self.callback_sbp_base_pos, msg_type=SBP_MSG_BASE_POS)

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

        if msg.flags & 0x03:
            out.status.status = NavSatStatus.STATUS_GBAS_FIX

            # TODO this should probably also include covariance of base fix?
            out.position_covariance[0] = self.rtk_h_accuracy**2
            out.position_covariance[4] = self.rtk_h_accuracy**2
            out.position_covariance[8] = self.rtk_v_accuracy**2

            pub = self.pub_rtk_fix
            self.last_rtk_pos = msg
        else:
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

            pub = self.pub_fix
            self.last_pos = msg

        #out.position_covariance
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
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
        return

        # out = Odometry()
        # out.header.frame_id = self.frame_id
        # out.header.stamp = rospy.Time.now()
        # out.child_frame_id = self.child_frame_id
        #
        # out.twist.twist.linear.x = msg.e/1000.0
        # out.twist.twist.linear.y = msg.n/1000.0
        # out.twist.twist.linear.z = -msg.d/1000.0
        #
        # self.pub_odom.publish(out)


    def callback_sbp_baseline(self, msg, **metadata):
        if self.debug:
            rospy.loginfo("Received SBP_MSG_BASELINE_NED (Sender: %d): %s" % (msg.sender, repr(msg)))
        self.last_baseline = msg
        self.publish_odom()
        return

        # out = Odometry()
        # out.header.frame_id = self.frame_id
        # out.header.stamp = rospy.Time.now()
        # out.child_frame_id = self.child_frame_id
        #
        # out.pose.pose.position.x = msg.e/1000.0
        # out.pose.pose.position.y = msg.n/1000.0
        # out.pose.pose.position.z = -msg.d/1000.0
        #
        # self.pub_rtk.publish(out)

    def callback_sbp_obs(self, msg, **metadata):
        if self.debug:
            rospy.loginfo("Received SBP_MSG_OBS (Sender: %d): %s" % (msg.sender, repr(msg)))

        if self.send_observations:
            for s in self.obs_senders:
                s.send(msg)


    def callback_sbp_base_pos(self, msg, **metadata):
        print msg
        if self.debug:
            rospy.loginfo("Received SBP_MSG_BASE_POS (Sender: %d): %s" % (msg.sender, repr(msg)))

        if self.send_observations:
            for s in self.obs_senders:
                s.send(msg)

        # publish tf for rtk frame
        if self.publish_tf:
            if not self.proj:
                self.init_proj((msg.lat, msg.lon))

            E,N = self.proj(msg.lon,msg.lat, inverse=False)

            self.transform.header.stamp = rospy.Time.now()
            self.transform.translation.x = E
            self.transform.translation.y = N
            self.transform.translation.z = -msg.height
            self.tf_br.sendTransform(self.transform)

    def fix_mode(self):
        # Check how old messages are

        d = None
        if self.last_rtk_pos is not None:
            d = self.last_rtk_pos
        elif self.last_pos is not None:
            d = self.last_pos
        elif self.last_baseline is not None:
            d = self.last_baseline

        if d:
            return d.flags & 0x03
        else:
            return -1

    def diag(self, stat):
        fix_mode = self.fix_mode()
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

        stat.add("Fix mode", FIX_MODE[self.fix_mode()])

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
                    rospy.sleep(1.0)
                    if not self.piksi.is_alive():
                        raise IOError
                    self.diag_updater.update()

                break # should only happen if rospy is trying to shut down
            except IOError as e:
                rospy.logerr("IOError")
                self.disconnect_piksi()
            except SystemExit as e:
                rospy.logerr("Unable to connect to Piksi on port %s" % self.piksi_port)
                self.disconnect_piksi()
            except: # catch *all* exceptions
                e = sys.exc_info()[0]
                print e
                self.disconnect_piksi()
            rospy.loginfo("Attempting to reconnect in %fs" % reconnect_delay)
            rospy.sleep(reconnect_delay)

def main():
    node = PiksiROS()
    node.spin()

if __name__ == "__main__":
  main()
