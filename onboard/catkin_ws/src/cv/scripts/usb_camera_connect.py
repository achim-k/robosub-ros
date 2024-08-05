#!/usr/bin/env python3

import rospy
import roslaunch
import yaml
import resource_retriever as rr

from usb_camera import USBCamera

CAMERA_CONFIG_PATH = 'package://cv/configs/usb_cameras.yaml'


def connect_all():

    rospy.init_node("usb_camera_connect_all", anonymous=True)

    with open(rr.get_filename(CAMERA_CONFIG_PATH, use_protocol=False)) as f:
        cameras = yaml.safe_load(f)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    roslaunch_files = []
    for camera_name in cameras:

        camera = cameras[camera_name]
        channel = camera["channel"]
        topic = camera["topic"]

        cli_args = ["cv", "usb_camera.launch", f"topic:={topic}", f"channel:={channel}", "framerate:=-1"]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
        roslaunch_files.append((roslaunch_file, cli_args[2:]))

    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files)
    parent.start()


if __name__ == "__main__":
    rospy.init_node("usb_camera_connect")
    connect_all()
