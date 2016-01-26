#!/usr/bin/env python

PKG='piksi_ros'

import rospy
import roslib; roslib.load_manifest(PKG)

from piksi_ros.msg import Observations, Obs, Ephemeris, SignalStatus

import numpy as np

def reject_outliers(data, m = 2.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.
    return data[s<m]

class SignalMonitor():

    def __init__(self):
        rospy.init_node("piksi_signal_monitor")

        self.signal_pub = rospy.Publisher("~signal_status", SignalStatus, queue_size=1000)
        self.obs_sub = rospy.Subscriber("observations", Observations, callback=self.obs_callback)

    def obs_callback(self, msg):
        cn0 = np.array([obs.cn0/4.0 for obs in msg.obs])

        m = SignalStatus()
        m.header.stamp = rospy.Time.now()
        m.mean_cn0 = np.mean(cn0)
        m.median_cn0 = np.median(cn0)
        m.robust_mean_cn0 = np.mean(reject_outliers(cn0))
        m.num_sats = len(msg.obs)
        self.signal_pub.publish(m)

def main():
    node = SignalMonitor()
    rospy.spin()

if __name__ == "__main__":
    main()
