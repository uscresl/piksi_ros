#!/usr/bin/env python

PKG='piksi_ros'

import rospy
import roslib; roslib.load_manifest(PKG)

from piksi_ros.msg import Observations, Obs, Ephemeris, SignalStatus

import numpy as np

import time

def reject_outliers(data, m = 2.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.
    return data[s<m]

class SignalMonitor():

    def __init__(self):
        rospy.init_node("piksi_signal_monitor")

        self.last_obs_time = 0
        self.obs_timeout = 0.75

        self.signal_pub = rospy.Publisher("~signal_status", SignalStatus, queue_size=1000)
        self.obs_sub = rospy.Subscriber("observations", Observations, callback=self.obs_callback)

    def obs_callback(self, msg):
        self.last_obs_time = time.time()
        cn0 = np.array([obs.cn0/4.0 for obs in msg.obs])

        m = SignalStatus()
        m.header.stamp = rospy.Time.now()
        m.mean_cn0 = np.mean(cn0)
        m.median_cn0 = np.median(cn0)
        m.robust_mean_cn0 = np.mean(reject_outliers(cn0))
        m.num_sats = len(msg.obs)
        self.signal_pub.publish(m)

    def send_zero_status(self):
        m = SignalStatus()
        m.header.stamp = rospy.Time.now()
        m.mean_cn0 = 0
        m.median_cn0 = 0
        m.robust_mean_cn0 = 0
        m.num_sats = 0
        self.signal_pub.publish(m)

    def spin(self):
        while not rospy.is_shutdown():
            if time.time() - self.last_obs_time > self.obs_timeout:
                self.send_zero_status()
            rospy.sleep(1.0)

def main():
    node = SignalMonitor()
    node.spin()

if __name__ == "__main__":
    main()
