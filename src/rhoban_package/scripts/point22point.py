#!/usr/bin/env python3
import rospy
import pcl
from sensor_msgs.msg import PointCloud2, PointCloud
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np

def callback(data):
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    p = pcl.PointCloud(np.array(points, dtype=np.float32))
    rospy.loginfo(p)
    pub.publish(p)


rospy.init_node('listener', anonymous=True)
pub = rospy.Publisher('/scan', PointCloud, queue_size=10)
rospy.Subscriber("/rslidar_points", PointCloud2, callback)
rospy.spin()