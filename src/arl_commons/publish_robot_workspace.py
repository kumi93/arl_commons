#!/usr/bin/env python
import os
import numpy as np
from scipy.spatial import ConvexHull
import xmlrpclib
from collada import *

import rospy
import roslib
import tf
import rospkg
from visualization_msgs.msg import Marker
from shape_msgs.msg import Mesh


class WorkspacePublisher:

    def __init__(self, name, mesh_path):
        rospy.init_node(name, anonymous=True)
        rospy.on_shutdown(self._shutdown_hook)
        self._ros_master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        self._workspace_publisher = rospy.Publisher('/robot/markers', Marker, queue_size=10)
        self._tf_broadcaster = tf.TransformBroadcaster()

        self._marker = Marker()
        self._marker.pose.position.x = .0
        self._marker.pose.position.y = .0
        self._marker.pose.position.z = .0

        self._marker.pose.orientation.x = .0
        self._marker.pose.orientation.y = .0
        self._marker.pose.orientation.z = .0
        self._marker.pose.orientation.w = 1.0

        self._marker.header.frame_id = 'robot_workspace'
        self._marker.header.stamp = rospy.Time.now()

        self._marker.id = 0
        self._marker.type = Marker.MESH_RESOURCE
        self._marker.action = Marker.ADD

        self._marker.scale.x = 0.01
        self._marker.scale.y = 0.01
        self._marker.scale.z = 0.01

        self._marker.color.a = 1.0
        self._marker.color.r = 1.0
        self._marker.color.g = .0
        self._marker.color.b = .0

        self._marker.mesh_resource = mesh_path

    def display(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self._workspace_publisher.publish(self._marker)
            self._tf_broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), 'base_link', 'robot_workspace')
            rate.sleep()

    def _shutdown_hook(self):
        self._marker.action = Marker.DELETE
        self._workspace_publisher.publish(self._marker)


class MeshGenerator:

    def __init__(self, bag_path):
        self._bag_path = bag_path

    def generate_mesh(self, save_path):
        points = np.random.rand(300, 3)
        hull = ConvexHull(points)
        edges = zip(*points)

        mesh = Collada()
        mesh_effect = material.Effect("effect0", [], "phong", diffuse=(1, 0, 0), specular=(0, 1, 0))
        mesh_material = material.Material("material0", "mymaterial", mesh_effect)
        mesh.effects.append(mesh_effect)
        mesh.materials.append(mesh_material)

        vert_floats = [-1000, 1000, 1000, 1000, 1000, 1000, -1000, -1000, 1000, 1000, -1000, 1000, -50, 50, -50, 50, 50, -50, -50, -50, -50, 50, -50, -50]
        normal_floats = [0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, -1, 0, 0, -1, 0, 0, -1, 0, -1, 0, 0, -1, 0, 0, -1, 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, -1, 0, 0, -1, 0, 0, -1]
        vert_src = source.FloatSource("cubeverts-array", np.array(vert_floats), ('X', 'Y', 'Z'))
        normal_src = source.FloatSource("cubenormals-array", np.array(normal_floats), ('X', 'Y', 'Z'))
        geom = geometry.Geometry(mesh, "geometry0", "mycube", [vert_src, normal_src])
        input_list = source.InputList()
        input_list.addInput(0, 'VERTEX', "#cubeverts-array")
        input_list.addInput(1, 'NORMAL', "#cubenormals-array")
        indices = np.array([0, 0, 2, 1, 3, 2, 0, 0, 3, 2, 1, 3, 0, 4, 1, 5, 5, 6, 0, 4, 5, 6, 4, 7, 6, 8, 7, 9, 3, 10, 6, 8, 3, 10, 2, 11, 0, 12, 4, 13, 6, 14, 0, 12, 6, 14, 2, 15, 3, 16, 7, 17, 5, 18, 3, 16, 5, 18, 1, 19, 5, 20, 7, 21, 6, 22, 5, 20, 6, 22, 4, 23])
        triset = geom.createTriangleSet(indices, input_list, "materialref")
        geom.primitives.append(triset)
        mesh.geometries.append(geom)
        matnode = scene.MaterialNode("materialref", mesh_material, inputs=[])
        geomnode = scene.GeometryNode(geom, [matnode])
        node = scene.Node("node0", children=[geomnode])
        myscene = scene.Scene("myscene", [node])
        mesh.scenes.append(myscene)
        mesh.scene = myscene
        mesh.write(save_path)



    def _bag_reader(self):
        pass


if __name__ == '__main__':
    try:
        mesh_uri = 'package://arl_commons/config/workspace__.dae'
        bag_path = '/home/user/tmp/bags'
        generate_mesh = True
        mesh_path = rospkg.RosPack().get_path('arl_commons') + '/config/workspace_gen.dae'

        if generate_mesh:
            mesh_uri = 'package://arl_commons/config/workspace_gen.dae'
            mesh_gen = MeshGenerator(bag_path)
            mesh_gen.generate_mesh(mesh_path)

        node = WorkspacePublisher('workspace_publisher_node', mesh_uri)
        node.display()
    except rospy.ROSInterruptException:
        pass
