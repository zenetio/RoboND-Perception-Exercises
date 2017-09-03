#!/usr/bin/env python

# Import modules
from pcl_helper import *
# import rospy

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Initialization

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    # TODO: Voxel Grid Downsampling
    vox = pcl_data.make_voxel_grid_filter()
    # try a voxel size and set to cloud
    LEAF_SIZE = 0.002
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # call filter function to obtain the resultant downsampled PC
    cloud_filtered = vox.filter()
    # save file
    filename = 'voxel_sampled.pcd'
    pcl.save(cloud_filtered, filename)
    # TODO: PassThrough Filter to isolate the table and objects
    passthrough = cloud_filtered.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    # use filter function to obtain only table and objects
    cloud_filtered = passthrough.filter()
    # save result
    filename = 'pass_through_filtered.pcd'
    pcl.save(cloud_filtered, filename)
    # TODO: RANSAC Plane Segmentation, will table and objects
    seg = cloud_filtered.make_segmenter()
    # TODO: Extract inliers and outliers
    # set the model we wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    # set max distance for a point to be considered fitting the model
    seg.set_distance_threshold(max_distance)
    # call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    # extracted inliers - table
    pcl_table = cloud_filtered.extract(inliers, negative=False)
    # save file
    filename = 'pcl_table.pcd'
    pcl.save(pcl_table, filename)
    # extracted outliers - objects
    pcl_objects = cloud_filtered.extract(inliers, negative=True)
    # save file
    filename = 'pcl_objects.pcd'
    pcl.save(pcl_objects, filename)
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(pcl_objects)
    tree = white_cloud.make_kdtree()
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(1)
    ec.set_MaxClusterSize(20000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(pcl_objects)
    ros_cloud_table = pcl_to_ros(pcl_table)
    # now publish ros_cluster_cloud
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()