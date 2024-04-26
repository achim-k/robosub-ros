import rospy
import math

from task import Task, task
import move_tasks
import cv_tasks
from utils import geometry_utils

# from interface.controls import Controls
from interface.state import State


@task
async def prequal_task(self: Task) -> Task[None, None, None]:
    """
    Complete the prequalification task by moving to a series of local poses. Returns when the robot is at the final pose
    with zero velocity, within a small tolerance, completing the prequalifiacation task.
    """

    directions = [
        (0, 0, -0.7),
        (2.5, 0, -0.2),
        (4.75, 0, -0.2),
        # (4.75, 0, -0.2),
        # (0, 0.6, -0.1),
        # (3.5, 0.1, -0.1),
        # (0, -1.2, -0.1),
        # (-3.5, -0.1, -0.1),
        # (0, 0.6, -0.1),
        # (-7.5, 0, -0.3),
        # (-7.5, 0, -0.3)
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")

    cv_task = cv_tasks.move_to_cv_obj("buoy_earth_cetus", parent=self)
    cv_pose = cv_task.send(None)
    while not geometry_utils.at_pose(geometry_utils.create_pose(0, 0, 0, 0, 0, 0), cv_pose, linear_tol=0.5,
                                     angular_tol=0.5):
        cv_pose = cv_task.send(None)
        rospy.loginfo(cv_pose.position)

    rospy.loginfo("Moved to buoy")

    move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, -math.pi/10, 0, 0), parent=self)
    rospy.loginfo("Rolled -30 deg")

    directions = [
        (0, 2, -0.1),
        (-3, 0, -0.1),
        (0, -1, 0),
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")

    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, math.pi), parent=self)
    rospy.loginfo("Yawed 180")

    await move_tasks.move_to_pose_local(geometry_utils.create_pose(4.75, 0, -0.2, 0, 0, 0), parent=self)
    rospy.loginfo("Moved (4.75, 0, -0.2)")

    cv_task = cv_tasks.move_to_cv_obj("gate_abydos", parent=self)
    cv_pose = cv_task.send(None)
    while not geometry_utils.at_pose(geometry_utils.create_pose(0, 0, 0, 0, 0, 0), cv_pose, linear_tol=1,
                                     angular_tol=0.5):
        cv_pose = cv_task.send(None)
        rospy.loginfo(cv_pose.position)

    rospy.loginfo("Moved to gate")

    move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, -math.pi/10, 0, 0), parent=self)
    rospy.loginfo("Rolled -30 deg")

    directions = [
        (0, 0, -0.5),
        (3, 0, -0.15)
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        rospy.loginfo(f"Moved to {direction}")

