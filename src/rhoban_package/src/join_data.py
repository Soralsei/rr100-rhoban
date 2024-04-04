#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2


def callback(data):
    pub = rospy.Publisher("velodyne_points", PointCloud2, queue_size=10)
    # rospy.loginfo(str(i)+ " height :  "+str(data.height))
    # rospy.loginfo(str(i)+ " width :  "+str(data.width))
    # rospy.loginfo(str(i)+ " is_bigendian :  "+str(data.is_bigendian))
    # rospy.loginfo(str(i)+ " point_step :  "+str(data.point_step))
    # rospy.loginfo(str(i)+ " row_step :  "+str(data.row_step))
    # rospy.loginfo(str(i)+ " is_dense :  "+str(data.is_dense))
    # rospy.loginfo(f"size :  {len(data)}, shape : {[len(i) for i in data]}")
    pub.publish(data)


def listener():
    rospy.init_node("join_data", anonymous=True)
    # rospy.Subscriber("rslidar_points", PointCloud2, callback)
    rospy.Subscriber("camera/depth/color/points", PointCloud2, callback)
    rospy.spin()


if __name__ == "__main__":
    listener()
