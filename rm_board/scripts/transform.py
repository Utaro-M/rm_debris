#! /usr/bin/env python
##https://python5.com/q/oxlqykvy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import rospy
import time

def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    time.sleep(1)
    print type(gen)
    for p in gen:
        print " x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2])
    # pub=rospy.Publisher('/pointcloud2topointcloud', PointCloud, queue_size=10)

def main():
    rospy.init_node('pcl_listener', anonymous=True)
    rospy.Subscriber('/multisense_local/hsi_filter/output', PointCloud2, callback_pointcloud)
    rospy.spin()

if __name__ == "__main__":
    main()
