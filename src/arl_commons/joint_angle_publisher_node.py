#!/usr/bin/env python

import os
import xmlrpclib
import yaml

import rospy
import roslib
import rospkg
from arl_hw_msgs.msg import AnalogInputArray, AnalogInput


class Publisher:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self._name = name
        self._ros_master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])

        proj_path =  rospkg.RosPack().get_path('arl_commons')
        with open(proj_path + '/config/potentiometer_settings.yaml', 'rt') as f:
            text = f.read()
        self._settings = yaml.safe_load(text)

        rospy.Subscriber('/sensor_readings/analog_inputs', AnalogInputArray, self._input_cb)
        self._joint_angle_publisher = rospy.Publisher('/joint_angle', AnalogInputArray, queue_size=10)

        rospy.spin()

    def convert_to_deg(self, voltage, joint_name):
        setting = self._settings[joint_name]
        deg = (voltage - setting['voltage_zero_angle']) * setting['effective_angle'] * setting['gear_ratio'] / setting['voltage_max']
        return deg

    def _input_cb(self, data):
        joint_angles = AnalogInputArray()
        joint_angles.header = data.header
        for i, input in enumerate(data.inputs):
            joint_angle = AnalogInput()
            joint_angle.name = 'joint_' + input.name.split('_')[-1]
            joint_angle.voltage = self.convert_to_deg(input.voltage, joint_angle.name)
            joint_angles.inputs.append(joint_angle)
        self._joint_angle_publisher.publish(joint_angles)


if __name__ == '__main__':
    try:
        pub = Publisher('joint_angle_publisher_node')
    except rospy.ROSInterruptException:
        pass
