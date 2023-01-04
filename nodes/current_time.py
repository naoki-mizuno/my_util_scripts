#!/usr/bin/env python

import rospy
from jsk_rviz_plugins.msg import OverlayText

import argparse
import datetime
import sys


def parse_args():
    description = "show current ROS time"
    parser = argparse.ArgumentParser(
        description=description,
        add_help=False,
    )
    parser.add_argument(
        "-h", "--help",
        action="help",
        help="show this help message and exit",
    )
    parser.add_argument(
        "-c",
        "--console",
        action="store_true",
        help="print to console instead of publishing as OverlayText",
    )

    args, _ = parser.parse_known_args()
    return args


class Node(object):
    def __init__(self, rate=50, fmt=None, out_topic="current_time", console=False):
        self._fmt = fmt

        self._pub = rospy.Publisher(out_topic, OverlayText, queue_size=1)

        self._console = console

        period = rospy.Duration.from_sec(1.0 / rate)
        self._timer = rospy.Timer(period, self.cb_timer, reset=True)

    def cb_timer(self, event):
        now_sec = rospy.Time.now().to_sec()
        now = datetime.datetime.fromtimestamp(now_sec)

        if self._fmt is None:
            now_str = now.isoformat()
        else:
            now_str = now.strftime(self._fmt)

        if self._console:
            sys.stdout.write("\r{}".format(now_str))
            sys.stdout.flush()
        else:
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
    args = parse_args()

    rospy.init_node("current_time")

    rate = rospy.get_param("~rate", 50)
    fmt = rospy.get_param("~fmt", "%Y-%m-%d %H:%M:%S.%f")

    Node(rate, fmt, console=args.console)

    rospy.spin()


if __name__ == "__main__":
    main()
