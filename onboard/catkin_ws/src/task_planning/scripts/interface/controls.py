import rospy

from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from custom_msgs.msg import ControlTypes, ThrusterAllocs
from custom_msgs.srv import SetControlTypes
import resource_retriever as rr
import os
import yaml


class ControlsInterface:
    STATE_TOPIC = 'state'


    CONTROL_TYPES_SERVICE = 'controls/set_control_types'
    RESET_PID_LOOPS_SERVICE = 'controls/reset_pid_loops'
    DESIRED_POSITION_TOPIC = 'controls/desired_position'
    DESIRED_VELOCITY_TOPIC = 'controls/desired_velocity'
    THRUSTER_ALLOCS_TOPIC = 'controls/thruster_allocs'


    def __init__(self, listener):
        self.listener = listener


        rospy.wait_for_service(self.CONTROL_TYPES_SERVICE)
        self._set_control_types = rospy.ServiceProxy(self.CONTROL_TYPES_SERVICE, SetControlTypes)
        # Note: if this variable gets out of sync with the actual control types,
        # bad things may happen
        self._all_axes_control_type = None


        rospy.wait_for_service(self.RESET_PID_LOOPS_SERVICE)
        self._reset_pid_loops = rospy.ServiceProxy(self.RESET_PID_LOOPS_SERVICE, Trigger)


        rospy.Subscriber(self.STATE_TOPIC, Odometry, self._on_receive_state)


        self._desired_position_pub = rospy.Publisher(self.DESIRED_POSITION_TOPIC, Pose, queue_size=1)
        self._desired_velocity_pub = rospy.Publisher(self.DESIRED_VELOCITY_TOPIC, Twist, queue_size=1)
        self._state = None


        self._read_config = None


        self.num_thrusters
        self.thruster_dict
        self.get_thruster_dict()
        self._thruster_pub = rospy.Publisher(self.THRUSTER_ALLOCS_TOPIC, ThrusterAllocs, queue_size=1)
   
    def get_thruster_dict(self):
        CONFIG_FILE_PATH = 'package://controls/config/%s.config'
        f = rr.get_filename(CONFIG_FILE_PATH
                        % os.getenv("ROBOT_NAME", "oogway"), use_protocol=False)
        full_thruster_dict = yaml.safe_load(f)
        num_thrusters = len(full_thruster_dict)
        i = 0
       
        thruster_dict = {}
        for t_dict in full_thruster_dict['thrusters']:
            thruster_name = t_dict['name']
            thruster_dict[thruster_name] = i
            i += 1


        self.num_thrusters = num_thrusters
        self.thruster_dict = thruster_dict
        return thruster_dict




    @property
    def state(self):
        return self._state


    def _set_all_axes_control_type(self, type):
        if self._all_axes_control_type == type:
            return
        # TODO what if this doesn't return success?
        self._set_control_types(ControlTypes(
            x=type,
            y=type,
            z=type,
            roll=type,
            pitch=type,
            yaw=type
        ))
        self._all_axes_control_type = type
        self.start_new_move()


    # Resets the PID loops. Should be called for every "new" movement
    def start_new_move(self):
        self._reset_pid_loops()


    # In global coordinates
    def publish_desired_position(self, pose):
        self._set_all_axes_control_type(ControlTypes.DESIRED_POSE)
        self._desired_position_pub.publish(pose)


    # In local coordinates
    def publish_desired_velocity(self, twist):
        self._set_all_axes_control_type(ControlTypes.DESIRED_TWIST)
        self._desired_velocity_pub.publish(twist)


    def _on_receive_state(self, state):
        self._state = state


    def get_state(self):
        return self._state
   
    def publish_thruster_allocs(self, **kwargs):
        thruster_allocs = [0] * len(self.num_thrusters)


        for kwarg_name, kwarg_value in kwargs.items():
            if kwarg_name in self.thruster_dict:
                thruster_allocs[self.thruster_dict[kwarg_name]] = kwarg_value
            else:
                raise ValueError("Thruster name not in thruster_dict", kwarg_name)
        self._thruster_pub.publish(thruster_allocs)


