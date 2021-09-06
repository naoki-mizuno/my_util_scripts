#!/usr/bin/env python

from __future__ import print_function

import rospy
import tf.transformations as tft
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl

import numpy as np

import argparse
from collections import OrderedDict
import os
import sys
import yaml


class IMarkerHelper(object):
    INTERACTION_MODE_DICT = {
        "none": InteractiveMarkerControl.NONE,
        "menu": InteractiveMarkerControl.MENU,
        "button": InteractiveMarkerControl.BUTTON,
        "move": InteractiveMarkerControl.MOVE_AXIS,
        "move_plane": InteractiveMarkerControl.MOVE_PLANE,
        "rotate": InteractiveMarkerControl.ROTATE_AXIS,
        "move_rotate": InteractiveMarkerControl.MOVE_ROTATE,
        "move_3d": InteractiveMarkerControl.MOVE_3D,
        "rotate_3d": InteractiveMarkerControl.ROTATE_3D,
        "move_rotate_3d": InteractiveMarkerControl.MOVE_ROTATE_3D,
    }
    ORIENTATION_MODE_DICT = {
        "inherit": InteractiveMarkerControl.INHERIT,
        "fixed": InteractiveMarkerControl.FIXED,
        "view_facing": InteractiveMarkerControl.VIEW_FACING,
    }
    AXES = {
        # Quaternion order: x, y, z, w
        "x": [0, 0, 0, 1],
        "y": [0, 0, 0.707, 0.707],
        "z": [0, 0.707, 0, 0.707],
    }

    @staticmethod
    def make_control(
        name,
        desc="",
        markers=[],
        is_always_visible=False,
        omode="inherit",
        imode="none",
        orientation=None,
        rpy_deg=None,
        axis=None,
    ):
        control = InteractiveMarkerControl()

        control.name = name
        control.description = desc
        control.markers = markers
        control.always_visible = is_always_visible
        control.interaction_mode = IMarkerHelper.INTERACTION_MODE_DICT[imode]
        control.orientation_mode = IMarkerHelper.ORIENTATION_MODE_DICT[omode]

        if orientation is not None:
            control.orientation = orientation
        elif rpy_deg is not None:
            q = tft.quaternion_from_euler(
                np.radians(rpy_deg[0]),
                np.radians(rpy_deg[1]),
                np.radians(rpy_deg[2]),
                "rxyz",
            )
            control.orientation.x = q[0]
            control.orientation.y = q[1]
            control.orientation.z = q[2]
            control.orientation.w = q[3]
        elif axis is not None:
            q = IMarkerHelper.AXES[axis]
            control.orientation.x = q[0]
            control.orientation.y = q[1]
            control.orientation.z = q[2]
            control.orientation.w = q[3]
        else:
            control.orientation.w = 1

        return control

    @staticmethod
    def make_3dof_controls(omode="inherit", imode="move", is_always_visible=False):
        controls = []
        for axis in IMarkerHelper.AXES.keys():
            controls.append(
                IMarkerHelper.make_control(
                    imode + "_" + axis,
                    axis=axis,
                    omode=omode,
                    imode=imode,
                    is_always_visible=is_always_visible,
                )
            )
        return controls

    @staticmethod
    def make_6dof_controls(omode="inherit", is_always_visible=False):
        controls = []
        for imode in ("move", "rotate"):
            controls.extend(
                IMarkerHelper.make_3dof_controls(omode, imode, is_always_visible)
            )
        return controls

    @staticmethod
    def make_menu(handler, menu, parent=None):
        """
        :type handler: MenuHandler
        :type menu: dict[str,function|dict]
        """
        # Dictionary that maps menu's ID -> title
        menu_ids = {}
        for menu_title, callback in menu.items():
            if isinstance(callback, dict):
                submenu_id = handler.insert(menu_title)
                # Recursively insert the submenus ("callback" is a dict)
                sub_ids = IMarkerHelper.make_menu(handler, callback, parent=submenu_id)
                # Merge (flatten)
                for key, val in sub_ids.items():
                    menu_ids[key] = val
            else:
                menu_id = handler.insert(menu_title, callback=callback, parent=parent)
                menu_ids[menu_id] = menu_title
        return menu_ids


def parse_args():
    description = "publish mesh file as interactive marker"
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
        "mesh_fpath",
        help="path to the mesh file",
    )
    parser.add_argument(
        "-ipf",
        "--init-pose-file",
        dest="init_pose_fpath",
        metavar="PATH",
        help="yaml file that contains the initial pose (- for stdin)",
    )
    parser.add_argument(
        "-ip",
        "--init-pos",
        dest="init_pos",
        default=None,
        type=float,
        nargs=3,
        metavar=tuple("XYZ"),
        help="initial x, y, z position (meters). Takes precedence over init-pose-file",
    )
    parser.add_argument(
        "-p",
        "--pos",
        default=[0, 0, 0],
        type=float,
        nargs=3,
        metavar=tuple("XYZ"),
        help="x, y, z position offset (meters)",
    )
    parser.add_argument(
        "-o",
        "--orientation",
        default=[0, 0, 0],
        type=float,
        nargs=3,
        metavar=tuple("RPY"),
        help="roll, pitch, yaw offset (degrees)",
    )
    parser.add_argument(
        "--rotation-axes",
        default="rxyz",
        help="rotation axes to convert euler angles to quaternion",
    )
    parser.add_argument(
        "-f",
        "--frame",
        default="world",
        help="frame ID",
    )
    parser.add_argument(
        "-s",
        "--scale",
        default=0.001,
        type=float,
        help="scaling factor for the mesh",
    )
    parser.add_argument(
        "--axes-scale",
        default=0.5,
        type=float,
        help="scaling factor for the axes",
    )
    parser.add_argument(
        "-c",
        "--color",
        default=None,
        type=float,
        nargs=4,
        metavar=tuple("RGBA"),
        help="RGBA color in range [0, 1] (mesh default if unspecified)",
    )
    parser.add_argument(
        "--server-name",
        default="imarker_server",
        help="name of the interactive server",
    )
    parser.add_argument(
        "--marker-name",
        default="my_marker",
        help="name of the interactive marker",
    )

    args = parser.parse_args()
    return args


def make_mesh_marker(args):
    m = Marker()
    m.header.frame_id = args.frame
    m.type = Marker.MESH_RESOURCE
    m.mesh_resource = "file://" + os.path.realpath(args.mesh_fpath)

    m.pose.position.x = args.pos[0]
    m.pose.position.y = args.pos[1]
    m.pose.position.z = args.pos[2]

    q = tft.quaternion_from_euler(
        *np.radians(args.orientation), axes=args.rotation_axes
    )
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
        m.color.a = args.color[3]
    else:
        m.mesh_use_embedded_materials = True

    return m


def cb_marker(feedback):
    # Nothing to do here
    pass


def rotate_marker(
    server, fb, name, roll=0, pitch=0, yaw=0, axes="rxyz", is_delta=True, random=False
):
    if random:
        new_q = np.random.random(4)
        new_q = new_q / np.linalg.norm(new_q)
    else:
        o = fb.pose.orientation
        current_q = [o.x, o.y, o.z, o.w]
        q = tft.quaternion_from_euler(roll, pitch, yaw, axes=axes)
        # Absolute or delta given?
        new_q = tft.quaternion_multiply(current_q, q) if is_delta else q
    new_pose = fb.pose
    new_pose.orientation.x = new_q[0]
    new_pose.orientation.y = new_q[1]
    new_pose.orientation.z = new_q[2]
    new_pose.orientation.w = new_q[3]
    # Note: update=None means update pose
    server.doSetPose(update=None, name=name, pose=new_pose, header=fb.header)
    server.applyChanges()


def main():
    args = parse_args()

    rospy.init_node("interactive_mesh")

    marker = make_mesh_marker(args)

    imarker = InteractiveMarker()
    imarker.name = args.marker_name
    imarker.header.frame_id = args.frame
    imarker.scale = args.axes_scale

    if args.init_pose_fpath is not None:
        # Set initial pose of the marker from a file (or stdin)
        if args.init_pose_fpath == "-":
            doc = sys.stdin.read()
            print("Read initial pose from stdin")
        else:
            doc = open(args.init_pose_fpath).read()
            print("Read initial pose from file")
        pose = yaml.safe_load(doc)
        if "position" in pose:
            imarker.pose.position.x = pose["position"]["x"]
            imarker.pose.position.y = pose["position"]["y"]
            imarker.pose.position.z = pose["position"]["z"]
        if "orientation" in pose:
            imarker.pose.orientation.x = pose["orientation"]["x"]
            imarker.pose.orientation.y = pose["orientation"]["y"]
            imarker.pose.orientation.z = pose["orientation"]["z"]
            imarker.pose.orientation.w = pose["orientation"]["w"]
    if args.init_pos is not None:
        # Set initial position of the marker
        imarker.pose.position.x = args.init_pos[0]
        imarker.pose.position.y = args.init_pos[1]
        imarker.pose.position.z = args.init_pos[2]
    marker.pose = imarker.pose

    imarker.controls = [
        IMarkerHelper.make_control(
            "box", markers=[marker], imode="button", is_always_visible=True
        ),
    ]
    imarker.controls.extend(IMarkerHelper.make_6dof_controls("inherit", False))
    menu_handler = MenuHandler()
    IMarkerHelper.make_menu(
        menu_handler,
        OrderedDict(
            [
                [
                    "Show Pose",
                    lambda fb: print("Frame: {}\n{}".format(args.frame, fb.pose)),
                ],
                [
                    "Rotate 90 degrees (red)",
                    lambda fb: rotate_marker(
                        server,
                        fb,
                        roll=np.pi / 2,
                        name=args.marker_name,
                        axes=args.rotation_axes,
                    ),
                ],
                [
                    "Rotate 90 degrees (green)",
                    lambda fb: rotate_marker(
                        server,
                        fb,
                        pitch=np.pi / 2,
                        name=args.marker_name,
                        axes=args.rotation_axes,
                    ),
                ],
                [
                    "Rotate 90 degrees (blue)",
                    lambda fb: rotate_marker(
                        server,
                        fb,
                        yaw=np.pi / 2,
                        name=args.marker_name,
                        axes=args.rotation_axes,
                    ),
                ],
                [
                    "Reset orientation",
                    lambda fb: rotate_marker(
                        server,
                        fb,
                        name=args.marker_name,
                        is_delta=False,
                        axes=args.rotation_axes,
                    ),
                ],
            ],
        ),
    )

    # Create InteractiveMarker server
    server = InteractiveMarkerServer(args.server_name)
    server.insert(imarker, cb_marker)
    server.applyChanges()
    menu_handler.apply(server, imarker.name)

    rospy.spin()


if __name__ == "__main__":
    main()
