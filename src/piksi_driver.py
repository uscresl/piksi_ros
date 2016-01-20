#!/usr/bin/env python

PKG = "piksi_ros"

import time

import rospy
import roslib; roslib.load_manifest(PKG)
import diagnostic_updater
import diagnostic_msgs

from sensor_msgs.msg import NavSatFix, TimeReference, NavSatStatus
from nav_msgs.msg import Odometry

from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
#from sbp.navigation import SBP_MSG_BASELINE_NED, MsgBaselineNED
from sbp.system import SBP_MSG_HEARTBEAT
from sbp.observation import SBP_MSG_OBS, SBP_MSG_BASE_POS
from sbp.navigation import SBP_MSG_GPS_TIME, SBP_MSG_BASELINE_NED, SBP_MSG_VEL_NED, SBP_MSG_POS_LLH, SBP_MSG_DOPS

#
# # Classes for reading/writing observations to serial/udp
#
# class UDPWriter(object):
#     pass
#
# class UDPReader(object):
#     pass
#
# class SerialWriter(object):
#     pass
#
# class SerialReader(object):
#     pass
#
#

GPS_EPOCH = time.mktime((1980, 1, 6, 0, 0, 0, 0, 0, 0))
WEEK_SECONDS = 60*60*24*7

def ros_time_from_sbp_time(msg):
    return rospy.Time(GPS_EPOCH + msg.wn*WEEK_SECONDS + msg.tow * 1E-3 + msg.ns * 1E-9)


class PiksiROS(object):
    def __init__(self):

        rospy.init_node("piksi_ros")

        self.send_observations = True
        self.dops = None
        self.serial_number = None

        self.read_params()

        self.diag_updater = diagnostic_updater.Updater()
        self.heartbeat_diag = diagnostic_updater.FrequencyStatus(diagnostic_updater.FrequencyStatusParam({'min':self.diag_heartbeat_freq, 'max':self.diag_heartbeat_freq}, self.diag_freq_tolerance, self.diag_window_size))
        self.diag_updater.add(self.heartbeat_diag)

        self.setup_pubsub()
        self.setup_piksi()


    def read_params(self):
        self.frame_id = rospy.get_param('~frame_id', "gps")
        self.child_frame_id = rospy.get_param('~child_frame_id', "base_link")
        self.piksi_port = rospy.get_param('~port', "/dev/ttyUSB0")

        self.diag_heartbeat_freq = rospy.get_param('~diag/heartbeat_freq', 1.0)
        self.diag_update_freq = rospy.get_param('~diag/update_freq', 10.0)
        self.diag_freq_tolerance = rospy.get_param('~diag/freq_tolerance', 0.1)
        self.diag_window_size = rospy.get_param('~diag/window_size', 10)
        self.diag_min_delay = rospy.get_param('~diag/min_delay', 0.0)
        self.diag_max_delay = rospy.get_param('~diag/max_delay', 0.2)

    def setup_pubsub(self):

        freq_params = diagnostic_updater.FrequencyStatusParam({'min':self.diag_update_freq, 'max':self.diag_update_freq}, self.diag_freq_tolerance, self.diag_window_size)
        time_params = diagnostic_updater.TimeStampStatusParam(self.diag_min_delay, self.diag_max_delay)

        self.pub_fix = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~spp_fix", NavSatFix, queue_size=1000), self.diag_updater, freq_params, time_params)

        self.pub_rtk_fix = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~fix", NavSatFix, queue_size=1000), self.diag_updater, freq_params, time_params)
        self.pub_rtk = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~rtk_odom", Odometry, queue_size=1000), self.diag_updater, freq_params, time_params)
        self.pub_odom = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~odom", Odometry, queue_size=1000), self.diag_updater, freq_params, time_params)
        self.pub_time = diagnostic_updater.DiagnosedPublisher(rospy.Publisher("~time", TimeReference, queue_size=1000), self.diag_updater, freq_params, time_params)


    def setup_piksi(self):
        self.driver = PySerialDriver(self.piksi_port, baud=1000000)
        self.framer = Framer(self.driver.read, self.driver.write, verbose=False)
        self.source = Handler(self.framer)

        self.source.add_callback(self.callback_sbp_heartbeat, msg_type=SBP_MSG_HEARTBEAT)
        self.source.add_callback(self.callback_sbp_gps_time, msg_type=SBP_MSG_GPS_TIME)
        self.source.add_callback(self.callback_sbp_dops, msg_type=SBP_MSG_DOPS)

        self.source.add_callback(self.callback_sbp_pos, msg_type=SBP_MSG_POS_LLH)
        self.source.add_callback(self.callback_sbp_baseline, msg_type=SBP_MSG_BASELINE_NED)
        self.source.add_callback(self.callback_sbp_vel, msg_type=SBP_MSG_VEL_NED)

        if self.send_observations:
            self.source.add_callback(self.callback_sbp_obs, msg_type=[SBP_MSG_OBS, SBP_MSG_BASE_POS])

    def set_serial_number(self, serial_number):
        rospy.loginfo("Connected to piksi #%d" % serial_number)
        self.serial_number = serial_number
        self.diag_updater.setHardwareID("Piksi %d" % serial_number)

    def callback_sbp_gps_time(self, msg, **metadata):
        out = TimeReference()
        out.header.frame_id = self.frame_id
        out.header.stamp = rospy.Time.now()
        out.time_ref = ros_time_from_sbp_time(msg)
        out.source = "gps"
        self.pub_time.publish(out)

    def callback_sbp_heartbeat(self, msg, **metadata):
        self.heartbeat_diag.tick()
        if self.serial_number is None:
            self.set_serial_number(msg.sender)


    def callback_sbp_dops(self, msg, **metadata):
        self.dops = msg

    def callback_sbp_pos(self, msg, **metadata):

        out = NavSatFix()
        out.header.frame_id = self.frame_id
        out.header.stamp = rospy.Time.now()


        out.status.service = NavSatStatus.SERVICE_GPS

        out.latitude = msg.lat
        out.longitude = msg.lon
        out.altitude = msg.height

        if msg.flags & 0x03:
            out.status.status = NavSatStatus.STATUS_GBAS_FIX
            pub = self.pub_rtk_fix
        else:
            out.status.status = NavSatStatus.STATUS_FIX
            pub = self.pub_fix

        #out.position_covariance
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        pub.publish(out)


    def callback_sbp_vel(self, msg, **metadata):
        out = Odometry()
        out.header.frame_id = self.frame_id
        out.header.stamp = rospy.Time.now()
        out.child_frame_id = self.child_frame_id

        out.twist.twist.linear.x = msg.e/1000.0
        out.twist.twist.linear.y = msg.n/1000.0
        out.twist.twist.linear.z = -msg.d/1000.0

        self.pub_odom.publish(out)


    def callback_sbp_baseline(self, msg, **metadata):
        out = Odometry()
        out.header.frame_id = self.frame_id
        out.header.stamp = rospy.Time.now()
        out.child_frame_id = self.child_frame_id

        out.pose.pose.position.x = msg.e/1000.0
        out.pose.pose.position.y = msg.n/1000.0
        out.pose.pose.position.z = -msg.d/1000.0

        self.pub_rtk.publish(out)

    def callback_sbp_obs(self, msg, **metadata):
        pass

    def spin(self):
        self.source.start()

        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            self.diag_updater.update()

def main():
    node = PiksiROS()
    node.spin()

if __name__ == "__main__":
  main()
