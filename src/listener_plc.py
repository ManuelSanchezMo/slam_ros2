#!/usr/bin/python
from pypcd import pypcd
import rospy
from sensor_msgs.msg import PointCloud2


def cb(msg):
    pc = pypcd.PointCloud.from_msg(msg)
    #pc.save('foo.pcd', compression='binary_compressed')
    # maybe manipulate your pointcloud
    pc.pc_data['x'] *= -1
    print(pc)
    outmsg = pc.to_msg()
    # you'll probably need to set the header
    outmsg.header = msg.header
    pub.publish(outmsg)

# ...
rospy.init_node('pypcd_node')
sub = rospy.Subscriber('/velodyne_points', PointCloud2,cb)
pub = rospy.Publisher('outcloud', PointCloud2,queue_size=1)

rospy.spin()
