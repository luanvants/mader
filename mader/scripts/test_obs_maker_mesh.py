#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker

rospy.init_node('test_mesh_marker')

pub = rospy.Publisher('/test_mesh', Marker, queue_size=1, latch=True)

marker = Marker()
marker.header.frame_id = "world"
marker.ns = "test"
marker.id = 1
marker.type = Marker.MESH_RESOURCE
marker.action = Marker.ADD
marker.pose.position.x = 0.0
marker.pose.position.y = 0.0
marker.pose.position.z = 0.0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0
marker.scale.x = 1.0
marker.scale.y = 1.0
marker.scale.z = 1.0
marker.mesh_resource = "file:///home/nvhung/ws_mader/src/mader/mader/meshes/tretuarORTA/grass.dae"
marker.mesh_use_embedded_materials = True

rospy.sleep(1)
pub.publish(marker)
rospy.spin()