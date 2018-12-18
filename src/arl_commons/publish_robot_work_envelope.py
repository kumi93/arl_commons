#!/usr/bin/env python
import os
import glob
import numpy as np
from scipy.spatial import ConvexHull
import xmlrpclib
from stl import mesh
import yaml
from rosbag.bag import Bag

import rospy
import rosbag
import roslib
import tf
import rospkg
from visualization_msgs.msg import Marker


class WorkEnvelopePublisher:

    def __init__(self, mesh_path):
        rospy.on_shutdown(self._shutdown_hook)
        self._ros_master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        self._work_envelope_publisher = rospy.Publisher('/robot/markers', Marker, queue_size=10)
        self._tf_broadcaster = tf.TransformBroadcaster()

        self._marker = Marker()
        self._marker.pose.position.x = .0
        self._marker.pose.position.y = .0
        self._marker.pose.position.z = .0

        self._marker.pose.orientation.x = .0
        self._marker.pose.orientation.y = .0
        self._marker.pose.orientation.z = .0
        self._marker.pose.orientation.w = 1.0

        self._marker.header.frame_id = 'robot_work_envelope'
        self._marker.header.stamp = rospy.Time.now()

        self._marker.id = 0
        self._marker.type = Marker.MESH_RESOURCE
        self._marker.action = Marker.ADD

        self._marker.scale.x = 1.0
        self._marker.scale.y = 1.0
        self._marker.scale.z = 1.0

        self._marker.color.a = 0.5
        self._marker.color.r = 1.0
        self._marker.color.g = .0
        self._marker.color.b = .0

        self._marker.mesh_resource = mesh_path

    def display(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self._work_envelope_publisher.publish(self._marker)
            self._tf_broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(),
                                               'base_link', 'robot_work_envelope')
            rate.sleep()

    def _shutdown_hook(self):
        self._marker.action = Marker.DELETE
        self._work_envelope_publisher.publish(self._marker)


class MeshGenerator:

    def __init__(self, bag_path):
        self._bag_path = bag_path

    def generate_mesh(self, save_path):
        points = self._bag_reader()
        rospy.loginfo('Calculating convex hull')
        hull = ConvexHull(points)

        work_envelope_mesh = mesh.Mesh(np.zeros(hull.simplices.shape[0], dtype=mesh.Mesh.dtype))
        for i, s in enumerate(hull.simplices):
            for j in range(3):
                work_envelope_mesh.vectors[i][j] = hull.points[s[j], :]

        rospy.loginfo('Saving convex hull as stl')
        work_envelope_mesh.save(save_path)

    def _bag_reader(self):
        path_regex = self._bag_path + '/*.bag'
        bags = glob.glob(path_regex)
        positions = list()
        for b in bags:
            info_dict = yaml.load(Bag(b, 'r')._get_yaml_info())
            messages_num = info_dict['messages']
            bag = rosbag.Bag(b)
            count = 0
            for topic, msg, t in bag.read_messages(topics=['/training/state_data']):
                positions.append([msg.hand_pose.pose.position.x, msg.hand_pose.pose.position.y,
                                  msg.hand_pose.pose.position.z])
                if count % 10000 is 0:
                    rospy.loginfo(str((count / messages_num) * 100) + '% of messages processed')
                count += 1

            bag.close()
        return np.array(positions)


if __name__ == '__main__':
    rospy.init_node('work_envelope_publisher_node', anonymous=True)

    try:
        mesh_uri = 'package://arl_commons/config/work_envelope_gen.stl'
        bag_path = '/home/user/tmp/bags'
        generate_mesh = True
        mesh_path = rospkg.RosPack().get_path('arl_commons') + '/config/work_envelope_gen.stl'

        if generate_mesh:
            rospy.logwarn('Generation of work envelope can take a long time.')
            mesh_gen = MeshGenerator(bag_path)
            mesh_gen.generate_mesh(mesh_path)

        node = WorkEnvelopePublisher(mesh_uri)

        rospy.loginfo('Publishing convex hull')
        node.display()
    except rospy.ROSInterruptException:
        pass
