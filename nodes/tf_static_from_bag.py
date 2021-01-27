#!/usr/bin/env python

import rospy
import rosbag
from tf2_msgs.msg import TFMessage

import argparse
import os
import textwrap


def parse_args():
    description = "publish only the /tf_static from a bag"
    parser = argparse.ArgumentParser(
        description=description,
        add_help=False,
    )
    # fmt: off
    parser.add_argument(
        "-h", "--help",
        action="help",
        help="show this help message and exit",
    )
    parser.add_argument(
        "-d", "--delay",
        default=0,
        type=float,
        help="delay before publishing the transforms",
    )
    parser.add_argument(
        "--no-latched",
        dest="latched",
        action="store_false",
        help="do not publish as latched topic",
    )
    parser.add_argument(
        "--topic-in",
        default="/tf_static",
        help="topic to read from the bag file",
    )
    parser.add_argument(
        "--topic-out",
        default="/tf_static",
        help="topic to publish",
    )
    parser.add_argument(
        "bag_in",
        help="input bag file",
    )
    # fmt: on

    args, unknown = parser.parse_known_args()
    return args


def main():
    args = parse_args()

    rospy.init_node("tf_static_from_bag")

    bag_in = rosbag.Bag(args.bag_in)
    tform = TFMessage()
    for topic, msg, t in bag_in.read_messages(topics=[args.topic_in]):
        if hasattr(msg, "transforms"):
            tform.transforms.extend(msg.transforms)
        else:
            rospy.logwarn("`transforms` attribute not found in {}".format(msg))

    fmt = """Publishing {num_transforms} transforms read from {fname} to {topic_out}
      topic-in  : {topic_in}
      topic-out : {topic_out}
      delay     : {delay}
      latched   : {latched}
    """
    rospy.loginfo(
        fmt.format(
            num_transforms=len(tform.transforms),
            fname=os.path.basename(args.bag_in),
            topic_in=args.topic_in,
            topic_out=args.topic_out,
            delay=args.delay,
            latched=args.latched,
        )
    )

    pub = rospy.Publisher(
        args.topic_out,
        TFMessage,
        queue_size=1,
        latch=args.latched,
    )
    rospy.sleep(args.delay)
    pub.publish(tform)

    rospy.spin()


if __name__ == "__main__":
    main()
