#!/usr/bin/env python

import rospy
import tf.transformations as tft
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

import argparse
import glob
import os


def parse_args():
    description = "publish mesh file as visualization_msgs/Marker"
    parser = argparse.ArgumentParser(
        description=description,
        add_help=False,
    )
    parser.add_argument(
        "-h",
        "--help",
        action="help",
        help="show this help message and exit",
    )
    parser.add_argument(
        "mesh_path",
        nargs="+",
        help="files or directories of mesh (searches for *.stl if directory is given)",
    )
    parser.add_argument(
        "-p",
        "--pos",
        default=[0, 0, 0],
        type=float,
        nargs=3,
        metavar=tuple("XYZ"),
        help="x, y, z position (meters)",
    )
    parser.add_argument(
        "-o",
        "--orientation",
        default=[0, 0, 0],
        type=float,
        nargs=3,
        metavar=tuple("RPY"),
        help="roll, pitch, yaw (degrees)",
    )
    parser.add_argument("-f", "--frame", default="world", help="frame ID")
    parser.add_argument(
        "-s",
        "--scale",
        default=0.001,
        type=float,
        help="scaling factor for the mesh",
    )
    color = parser.add_mutually_exclusive_group()
    color.add_argument(
        "-c",
        "--color",
        default=None,
        type=float,
        nargs=3,
        metavar=tuple("RGB"),
        help="RGB color in range [0, 1] (mesh default if unspecified)",
    )
    color.add_argument(
        "--random-colors",
        action="store_true",
        help="use random colors for each mesh",
    )
    parser.add_argument(
        "-a",
        "--alpha",
        nargs="+",
        default=[1.0],
        help="alpha for the mesh (applied to all if single arg, each if multiple args)",
    )
    parser.add_argument(
        "--frame-lock",
        action="store_true",
        help="enable frame-locking",
    )
    parser.add_argument(
        "-r",
        "--rate",
        default=None,
        type=float,
        help="rate in Hz to publish (default: one-shot)",
    )
    parser.add_argument(
        "-t",
        "--topic",
        default="mesh_marker",
        help="topic to publish to",
    )
    parser.add_argument(
        "--one-namespace",
        action="store_true",
        help="put all mesh files in one namespace",
    )

    args, unknown = parser.parse_known_args()
    return args


def make_mesh_marker(file_path, marker_id, args, alpha=None, namespace=""):
    mesh_resource = "file://" + os.path.realpath(file_path)

    m = Marker()
    m.header.frame_id = args.frame
    m.id = marker_id
    m.type = Marker.MESH_RESOURCE
    m.mesh_resource = mesh_resource
    m.frame_locked = args.frame_lock
    m.ns = namespace

    m.pose.position.x = args.pos[0]
    m.pose.position.y = args.pos[1]
    m.pose.position.z = args.pos[2]

    q = tft.quaternion_from_euler(*np.radians(args.orientation))
    m.pose.orientation.x = q[0]
    m.pose.orientation.y = q[1]
    m.pose.orientation.z = q[2]
    m.pose.orientation.w = q[3]

    m.scale.x = args.scale
    m.scale.y = args.scale
    m.scale.z = args.scale

    # Color of the mesh resource (uses color defined in mesh if unspecified)
    if args.color is not None:
        m.color.r = args.color[0]
        m.color.g = args.color[1]
        m.color.b = args.color[2]
    elif args.random_color:
        m.color.r = np.random.random()
        m.color.g = np.random.random()
        m.color.b = np.random.random()
    else:
        m.mesh_use_embedded_materials = True

    if alpha is not None:
        m.color.a = alpha

    return m


def main():
    args = parse_args()

    rospy.init_node("mesh_marker_pub")

    ma = MarkerArray()

    mesh_fpaths = []
    mesh_id = 0
    # Collect all file paths
    for path in args.mesh_path:
        if os.path.isdir(path):
            mesh_fpaths.extend(glob.glob(path + "/*.stl"))
        elif os.path.exists(path):
            mesh_fpaths.append(path)
        else:
            rospy.logerr("No such file or directory : {}".format(path))
            continue

    if len(args.alpha) == 1:
        alphas = args.alpha * len(mesh_fpaths)
    elif len(args.alpha) != len(mesh_fpaths):
        raise RuntimeError(
            "Number of given alpha ({}) does not match the number of mesh files found ({})".format(
                len(args.alpha), len(mesh_fpaths)
            )
        )
    else:
        alphas = args.alpha

    for fpath, alpha in zip(mesh_fpaths, alphas):
        rospy.loginfo("Found {}".format(os.path.basename(fpath)))
        if args.one_namespace:
            namespace = ""
        else:
            namespace = os.path.basename(fpath)

        m = make_mesh_marker(fpath, mesh_id, args, alpha=alpha, namespace=namespace)
        ma.markers.append(m)
        mesh_id += 1

    pub = rospy.Publisher(args.topic, MarkerArray, latch=True, queue_size=1)
    if args.rate is None:
        pub.publish(ma)
        rospy.spin()
    else:
        rate = rospy.Rate(args.rate)
        while not rospy.is_shutdown():
            ma.markers[0].header.stamp = rospy.Time.now()
            pub.publish(ma)
            rate.sleep()


if __name__ == "__main__":
    main()
