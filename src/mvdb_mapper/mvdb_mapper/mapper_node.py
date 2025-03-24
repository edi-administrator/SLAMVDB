import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
import numpy as np
from mvdb_py.utils import pointcloud_to_pcd, matrix_to_transform_msg, stamp_to_int, transform_msg_to_matrix, int_to_stamp, dump_pts
from mvdb_interface.msg import LoopMessage
from .config import Config
from .mapper import Mapper, LoopEdge

from os.path import exists


class MapperNode(Node):

    def __init__(self):

        self.config = Config()
        super().__init__(self.config.MAPPER_NODE_NAME)
        self.tf_broadcaster = TransformBroadcaster(self)

        for k,v in vars(self.config).items():
            self.get_logger().info(f"config.{k} : {v}")

        self.mapper = Mapper(self.config, self.get_logger())

        self.mapper_frame = self.config.MAPPER_FRAME
        self.tracker_frame = self.config.TRACKER_FRAME

        self.cb_group_timer = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self.correction_pub = self.create_publisher(TransformStamped, self.config.MAPPER_CORRECTION_RUNNING_TOPIC, 10)
        self.loop_pub = self.create_publisher(LoopMessage, self.config.MAPPER_LOOP_TOPIC, 10)
        self.scan_sub = self.create_subscription(PointCloud2, self.config.INPUT_POINTS_TOPIC, self.points_sub, 10, callback_group=self.cb_group_timer)
        self.tracker_frame_sub = self.create_subscription(TransformStamped, self.config.TRACKER_POSE_TOPIC, self.tf_cb, 10)

        self.correction_timer = self.create_timer(1.0, self.timer_cb, self.cb_group_timer)

        self.stamp_to_scan = {}
        self.stamp_to_pose = {}

        self.get_logger().info("mapper node up!")

    def points_sub(self, msg: PointCloud2):

        self.stamp_to_scan[stamp_to_int(msg.header.stamp)] = pointcloud_to_pcd(msg)

        removable_stamps = []

        for stamp, scan in self.stamp_to_scan.items():

            if stamp in self.stamp_to_pose:

                removable_stamps.append(stamp)

                tracker_pose = self.stamp_to_pose[stamp]
                scan = self.stamp_to_scan[stamp]

                self.mapper.put_scan(stamp, scan, tracker_pose)
        
        for rstamp in removable_stamps:
            self.stamp_to_pose.pop(rstamp)
            self.stamp_to_scan.pop(rstamp)

    def tf_cb(self, msg: TransformStamped):

        self.stamp_to_pose[stamp_to_int(msg.header.stamp)] = transform_msg_to_matrix(msg)

        drift_msg = matrix_to_transform_msg(self.mapper.tracker_drift)
        drift_msg.child_frame_id = "tracker_frame"
        drift_msg.header.frame_id = "mapper_frame"
        drift_msg.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(drift_msg)

    def timer_cb(self):

        for submap in self.mapper.submap_list[-self.config.MAPPER_PUBLISH_MAXCOUNT:]:
            self.publish_correction(submap.mapper_pose, submap.timestamp)

            if self.config.DUMP_POSES:

                np.savetxt(
                    f"refactor_test_data/{submap.timestamp}.txt", 
                    (np.linalg.inv(submap.mapper_pose) @ submap.tracker_pose).reshape(-1,16), 
                    delimiter=",")
                np.savetxt(f"refactor_test_data/{submap.timestamp}.chk", submap.tracker_pose.reshape(-1,16), delimiter=",")
                np.savetxt(f"refactor_test_data/{submap.timestamp}.mpp", submap.mapper_pose.reshape(-1,16), delimiter=",")
        
        for i, loop in enumerate(self.mapper.loop_edges[-self.config.MAPPER_PUBLISH_MAXCOUNT:]):

            if self.config.DUMP_LOOPS:
                
                s_a = self.mapper.submap_list[loop.n_a]
                s_b = self.mapper.submap_list[loop.n_b]
                T_a_b = loop.T_a_b
                T_a = s_a.tracker_pose
                T_b = s_b.tracker_pose

                np.savetxt(f"loops/loop_{i}_{loop.conf:04f}_a:{loop.n_a:04d}->b:{loop.n_b:04d}_Tab.csv", T_a_b, delimiter=",")
                np.savetxt(f"loops/loop_{i}_{loop.conf:04f}_a:{loop.n_a:04d}->b:{loop.n_b:04d}_T_a.csv", T_a, delimiter=",")
                np.savetxt(f"loops/loop_{i}_{loop.conf:04f}_a:{loop.n_a:04d}->b:{loop.n_b:04d}_T_b.csv", T_b, delimiter=",")

                path_cloud_a = f"loops/loop_{i}_{loop.conf:04f}_a:{loop.n_a:04d}->b:{loop.n_b:04d}_pts_a.ply"
                path_cloud_b = f"loops/loop_{i}_{loop.conf:04f}_a:{loop.n_a:04d}->b:{loop.n_b:04d}_pts_b.ply"

                if not exists(path_cloud_a):
                    dump_pts(path_cloud_a, s_a.cloud)

                if not exists(path_cloud_b):
                    dump_pts(path_cloud_b, s_b.cloud)

            self.publish_loop(loop)
    
    def publish_correction(self, pose: np.ndarray, stamp: int) -> TransformStamped:

        drift_msg = matrix_to_transform_msg(pose)
        drift_msg.header.stamp = int_to_stamp(stamp)

        drift_msg.header.frame_id = self.mapper_frame
        drift_msg.child_frame_id = self.tracker_frame

        self.correction_pub.publish(drift_msg)

        return drift_msg

    def publish_loop(self, loop: LoopEdge) -> LoopMessage:

        msg = LoopMessage()
        msg.t_a = int_to_stamp(loop.t_a)
        msg.t_b = int_to_stamp(loop.t_b)
        msg._transform_a_b = matrix_to_transform_msg(loop.T_a_b)

        self.loop_pub.publish(msg)

        return msg

def main():

    rclpy.init()
    exec = rclpy.executors.MultiThreadedExecutor()

    mapper_node = MapperNode()
    exec.add_node(mapper_node)

    while rclpy.ok():
        exec.spin()
    for node in exec.get_nodes():
        node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()