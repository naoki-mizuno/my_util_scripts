#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.msg import OverlayText

import datetime


class Node(object):
    def __init__(self, rate=50, fmt=None, out_topic="current_time"):
        self._fmt = fmt

        self._pub = rospy.Publisher(out_topic, OverlayText, queue_size=1)

        period = rospy.Duration.from_sec(1.0 / rate)
        self._timer = rospy.Timer(period, self.cb_timer)

    def cb_timer(self, event):
        now_sec = rospy.Time.now().to_sec()
        now = datetime.datetime.fromtimestamp(now_sec)

        if self._fmt is None:
            now_str = now.isoformat()
        else:
            now_str = now.strftime(self._fmt)

        ot = OverlayText()
        ot.text = now_str
        ot.text_size = 14
        ot.height = 25
        ot.width = 320
        ot.fg_color.r = 0.09804
        ot.fg_color.g = 1
        ot.fg_color.b = 0.94117
        ot.fg_color.a = 0.8
        ot.bg_color.r = 0
        ot.bg_color.g = 0
        ot.bg_color.b = 0
        ot.bg_color.a = 0.8
        ot.font = "DejaVu SansMono"

        self._pub.publish(ot)


def main():
    rospy.init_node("current_time")

    rate = rospy.get_param("~rate", 50)
    fmt = rospy.get_param("~fmt", "%Y-%m-%d %H:%M:%S.%f")

    Node(rate, fmt)

    rospy.spin()


if __name__ == "__main__":
    main()
