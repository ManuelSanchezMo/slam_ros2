#!/usr/bin/python
from pypcd import pypcd
import rospy
from sensor_msgs.msg import PointCloud2
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
import numpy
def msg_reader( msg):

    '''Reads a PointCloud2 message type message and returns an array of lenght
    the number of points contained in the message and width, the number of 
    data labels.'''
    cloud_points = []
    # Iterate over the cloud and store the data of the points
    for p in pc2.read_points(msg):
        cloud_points.append(p)
    # Array of results
    array = numpy.array(cloud_points, dtype=numpy.float32)
    print(array.shape)
    return array
def cb(msg):
    map_points=msg_reader(msg)
    plot1 = plt. figure(1)
    plt.ion()
    plt.clf()
    ax = Axes3D(plot1)
    ax.scatter(map_points[:,0], map_points[:,1], map_points[:,2],color='green',s=1)
    plt.show()
    plt.pause(5.0)
# ...
rospy.init_node('pypcd_node')
sub = rospy.Subscriber('/map', PointCloud2,cb)


rospy.spin()
