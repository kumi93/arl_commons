#!/usr/bin/env python
import os
import xmlrpclib

import rospy
import roslib
from arl_hw_msgs.msg import Muscle, MuscleCommand, MusculatureState, MusculatureCommand
from std_msgs.msg import Float64
from rqt_py_common import topic_helpers

class Muxer:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self._ros_master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        self._muscle_mux_state_publisher = rospy.Publisher('/muscle_muxer/musculature_state', MusculatureState, queue_size=10)
        rospy.Subscriber('/muscle_muxer/musculature_command', MusculatureCommand, self._muscle_command_mux_callback)
        self._muscle_pressure_command_publishers = {}
        self._muscle_activation_command_publishers = {}
        self._muscle_states = {}
        self._publishing_musculature_state = False

    def run(self):
        self._update_muscle_controller_comm()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self._publish_muxed_musculature_state()
            rate.sleep()

    def _publish_muxed_musculature_state(self):
        self._publishing_musculature_state = True

        musculature_state = MusculatureState()
        musculature_state.header.stamp = rospy.get_rostime()
        musculature_state.header.frame_id = '0'
        for muscle in self._muscle_states:
            musculature_state.muscle_states.append(self._muscle_states[muscle])
        self._muscle_mux_state_publisher.publish(musculature_state)

        self._publishing_musculature_state = False

    def _register_subscribers(self, topics):
        for topic in topics:
            rospy.Subscriber('/' + topic + '/state', Muscle, self._muscle_state_callback)

    def _register_publishers(self, topics):
        pressure_publishers = {}
        activation_publishers = {}
        for topic in topics:
            pressure_publishers[topic] = rospy.Publisher('/' + topic + '/pressure_command', Float64, queue_size=10)
            activation_publishers[topic] = rospy.Publisher('/' + topic + '/activation_command', Float64, queue_size=10)
        self._muscle_pressure_command_publishers = pressure_publishers
        self._muscle_activation_command_publishers = activation_publishers

    def _get_topic_properties(self, topic_name):
        topic_type, real_topic, _ = topic_helpers.get_topic_type(topic_name)
        if topic_type is None:
            message = "topic %s does not exist" % (topic_name)
            return [], message

        slot_type, is_array, array_size = roslib.msgs.parse_type(topic_type)
        field_class = roslib.message.get_message_class(slot_type)

        return field_class, slot_type, is_array, array_size

    def _update_muscle_controller_comm(self):
        try:
            code, msg, val = self._ros_master.getPublishedTopics('/arl_muscle_muxer_script', "")
            if code == 1:
                published_topics = dict(val)
                controller_topics = []

                for topic in published_topics:
                    field_class, _, _, _ = self._get_topic_properties(topic)
                    if topic.split('/')[-1] == 'state' and field_class._type == 'arl_hw_msgs/Muscle':
                        controller_topics.append(topic.split('/')[1])

                self._register_publishers(sorted(set(controller_topics)))
                self._register_subscribers(sorted(set(controller_topics)))

            else:
                rospy.logerr("Communication with ROS Master failed")
        except:
            rospy.logerr("Something went wrong while discovering muscles")

    def _muscle_state_callback(self, data):
        if not self._publishing_musculature_state:
            self._muscle_states[data.name] = data

    def _muscle_command_mux_callback(self, data):
        for command in data.muscle_commands:
            controller_topic = command.name + '_controller'
            try:
                if command.control_mode == 0 and controller_topic in self._muscle_pressure_command_publishers:
                    self._muscle_pressure_command_publishers[controller_topic].publish(command.pressure)
                elif command.control_mode == 1 and controller_topic in self._muscle_pressure_command_publishers:
                    self._muscle_activation_command_publishers[controller_topic].publish(command.activation)
            except:
                pass

if __name__ == '__main__':
    try:
        mux = Muxer('muscle_muxer_node')
        mux.run()
    except rospy.ROSInterruptException:
        pass
