#!/usr/bin/python3
import os
import copy
import numpy
import open3d
import pyquaternion
from pypcd import pypcd
from std_msgs.msg import Header
import rospy
from sensor_msgs.msg import PointCloud2
import pprint
import sensor_msgs.point_cloud2 as pc2
def change_background_to_black(vis):
    opt = vis.get_render_option()
    opt.background_color = numpy.asarray([0, 0, 0])
    opt.point_size = 1.
    opt.light_on = True
    opt.show_coordinate_frame = True
    #     opt.load_from_json('./opt.json')
    return False

key_to_callback = {ord("K"): change_background_to_black}


class Keyframe(object):
	def __init__(self, id, cloud, odom):

		self.id = id
		self.odom = copy.deepcopy(odom)
		self.cloud = copy.deepcopy(cloud)
		self.transformed = copy.deepcopy(cloud)

		self.node = open3d.pipelines.registration.PoseGraphNode(odom)
		self.update_transformed()

	def update_transformed(self):
		self.transformed.points = self.cloud.points
		self.transformed.normals = self.cloud.normals
		self.transformed.transform(self.node.pose)


class GraphSLAM(object):
	def __init__(self):
		self.graph = open3d.pipelines.registration.PoseGraph()
		self.map_pub = rospy.Publisher('/map', PointCloud2, queue_size=10)
		self.keyframes = []
		self.last_frame_transformation = numpy.identity(4)
		self.sub = rospy.Subscriber('/velodyne_points', PointCloud2,self.cb)
		self.keyframe_angle_thresh_deg = 15.0
		self.keyframe_trans_thresh_m = 1.0
		self.count=0

		#self.vis = open3d.visualization.Visualizer()
		#self.vis.create_window()
		rospy.spin()
	def update(self, cloud):

		cloud = cloud.voxel_down_sample(0.1)

		if not len(self.keyframes):
			self.keyframes.append(Keyframe(0, cloud, numpy.identity(4)))
			#self.vis.add_geometry(self.keyframes[-1].transformed)
			self.graph.nodes.append(self.keyframes[-1].node)
			return

		if not self.update_keyframe(cloud):
			return

		print('optimizing...')
		option = open3d.pipelines.registration.GlobalOptimizationOption(max_correspondence_distance=1.0,
								      edge_prune_threshold=0.25, reference_node=0)
		print(open3d.pipelines.registration.global_optimization(self.graph,
							      open3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
							      open3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
							      option))

		for keyframe in self.keyframes:
			keyframe.update_transformed()

		#self.vis.poll_events()
		#self.vis.update_renderer()

	def update_keyframe(self, cloud):
		criteria = open3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
		reg = open3d.pipelines.registration.registration_icp(cloud,
							   self.keyframes[-1].cloud,
							   1.0,
							   self.last_frame_transformation,
							   open3d.pipelines.registration.TransformationEstimationPointToPlane(),
							   criteria=criteria)

		angle = pyquaternion.Quaternion(matrix=reg.transformation[:3, :3]).degrees
		trans = numpy.linalg.norm(reg.transformation[:3, 3])

		if abs(angle) < self.keyframe_angle_thresh_deg and abs(trans) < self.keyframe_trans_thresh_m:
			self.last_frame_transformation = reg.transformation
			return False

		odom = numpy.dot(self.keyframes[-1].odom, reg.transformation)
		self.keyframes.append(Keyframe(len(self.keyframes), cloud, odom))
		self.graph.nodes.append(self.keyframes[-1].node)
		#self.vis.add_geometry(self.keyframes[-1].transformed)

		self.last_frame_transformation = numpy.identity(4)

		information = open3d.pipelines.registration.get_information_matrix_from_point_clouds(self.keyframes[-1].cloud,
											   self.keyframes[-2].cloud,
											   1.0,
											   reg.transformation)
		edge = open3d.pipelines.registration.PoseGraphEdge(self.keyframes[-1].id,
							 self.keyframes[-2].id,
							 reg.transformation,
							 information,
							 uncertain=False)

		self.graph.edges.append(edge)

		return True


	def msg_reader(self, msg):
		'''Reads a PointCloud2 message type message and returns an array of lenght
		the number of points contained in the message and width, the number of 
		data labels.'''
		cloud_points = []
		# Iterate over the cloud and store the data of the points
		for p in pc2.read_points(msg):
			cloud_points.append(p)
		# Array of results
		array = numpy.array(cloud_points, dtype=numpy.float32)
		return array

	def points2pcd(self,points):
		#Storage path
		PCD_DIR_PATH=os.path.join(os.path.abspath('.'),'')
		PCD_FILE_PATH=os.path.join(PCD_DIR_PATH,'cache.pcd')
		if os.path.exists(PCD_FILE_PATH):
			os.remove(PCD_FILE_PATH)
    
		#Write file handle
		handle = open(PCD_FILE_PATH, 'a')
		#Get point cloud points
		point_num=points.shape[0]

		#pcdHead (important)
		handle.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1')
		string = '\nWIDTH ' + str(point_num)
		handle.write(string)
		handle.write('\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0')
		string = '\nPOINTS ' + str(point_num)
		handle.write(string)
		handle.write('\nDATA ascii')

		# Write points sequentially
		for i in range(point_num):
			string = '\n' + str(points[i, 0]) + ' ' + str(points[i, 1]) + ' ' + str(points[i, 2])
			handle.write(string)
		handle.close()
	def create_msg(self, points):
		'''Builds a PointCloud2 message with the coordinates (x,y,z) of the points
		depending on whether they are accessible (green) or not (red).
		Points is a 2-D numpy array and label a 1-D numpy array.
		'''
		# Create header of the message
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'velodyne'
		# Select the accessible or unaccessible points
  
		points_list = points.tolist()

		# Build point cloud message
		msg = pc2.create_cloud_xyz32(header, points_list)
		self.map_pub.publish(msg)
		print('pubmap')
		return msg
	def generate_map(self):
		map_cloud = open3d.geometry.PointCloud()

		for keyframe in self.keyframes:
			transformed = copy.deepcopy(keyframe.cloud)
			transformed.transform(keyframe.node.pose)
			map_cloud += transformed
		map_cloud = map_cloud.voxel_down_sample(0.1)
		map_points=numpy.asarray(map_cloud.points)
		self.create_msg(map_points)

		return map_cloud.voxel_down_sample(voxel_size=0.05)

	def cb(self,msg):
		points= self.msg_reader(msg)
		self.points2pcd(points)
		cloud = open3d.io.read_point_cloud("cache.pcd")
		cloud.estimate_normals()
		self.update(cloud)
		self.count=self.count+1
		print(self.count)
		if self.count%10==0:
			self.generate_map()	
             


def main():
	rospy.init_node('slam_ROS')
	graph_slam = GraphSLAM()
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
	main()
