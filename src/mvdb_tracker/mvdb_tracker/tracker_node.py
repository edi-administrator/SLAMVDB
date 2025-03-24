import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
import open3d as o3d
from mvdb_py.utils import pcd_to_pointcloud, pointcloud_to_pcd, matrix_to_transform_msg
from .config import Config
from .tracker import Tracker

class TrackerNode(Node):

    def __init__(self):

        self.config = Config()
        super().__init__(self.config.TRACKER_NODE_NAME)
        self.tf_broadcaster = TransformBroadcaster(self)

        for k,v in vars(self.config).items():
            self.get_logger().info(f"config.{k} : {v}")

        self.tracker = Tracker(self.config)

        self.pose_pub = self.create_publisher(TransformStamped, self.config.OUTPUT_TOPIC, 10)
        self.points_pub = self.create_publisher(PointCloud2, self.config.VISUALIZATION_TOPIC, 10)
        self.input_sub = self.create_subscription(PointCloud2, self.config.INPUT_TOPIC, self.input_cb, 10)

        self.get_logger().info("tracker node started")
    
    
    def input_cb(self, msg: PointCloud2):

        scan = pointcloud_to_pcd(msg, crop=True, crop_radius=self.config.CROP_RADIUS)
        self.tracker.insert_scan(o3d.geometry.PointCloud(scan))
        
        scan.transform(self.tracker.T_latest)
        self.publish_scan(scan, msg)
        
    def publish_scan(self, cloud: o3d.geometry.PointCloud, msg: PointCloud2):

        drift_msg = matrix_to_transform_msg(self.tracker.T_latest)
        drift_msg.header.stamp = msg.header.stamp
        drift_msg.header.frame_id = self.config.TRACKER_FRAME
        drift_msg.child_frame_id = self.config.ROBOT_FRAME
        self.pose_pub.publish(drift_msg)

        drift_msg.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(drift_msg)

        msg = pcd_to_pointcloud(cloud)
        msg.header.frame_id = self.config.TRACKER_FRAME
        self.points_pub.publish(msg)


def main():

    rclpy.init()

    exec = rclpy.executors.MultiThreadedExecutor()

    tracker_node = TrackerNode()
    exec.add_node(tracker_node)

    while rclpy.ok():
        exec.spin()
    for node in exec.get_nodes():
        node.destroy_node()

    rclpy.shutdown()
