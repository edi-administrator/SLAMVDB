import numpy as np
import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from mvdb_py.utils import stamp_to_int, transform_msg_to_matrix, matrix_to_transform_msg, SE3Interpolate, delta_t, normalized
from .config import Config

def grav_stationary(T_a, T_b, T_c, t_a, t_b, t_c, a_locals, w_norms, imu_stamps, config: Config):

    t_a = float(t_a) / 1e9
    t_b = float(t_b) / 1e9
    t_c = float(t_c) / 1e9

    v_ab = delta_t(T_a, T_b) / (t_b - t_a + np.finfo(float).eps)
    v_bc = delta_t(T_b, T_c) / (t_c - t_b + np.finfo(float).eps)

    dt_centers = (t_c - t_a) / 2
    a_avg = (v_bc - v_ab) / ( dt_centers + np.finfo(float).eps )
    a_avg_norm = np.linalg.norm(a_avg)

    v_ab_norm = np.linalg.norm(v_ab)
    v_bc_norm = np.linalg.norm(v_bc)

    print(f"\na_avg = {a_avg} v_diff = {v_bc - v_ab}")
    print(f"accel_norm = {a_avg_norm}")
    # print(f"v_ab = {v_ab} v_bc = {v_bc} diff = {v_ab - v_bc}")
    print(f"v_abn = {v_ab_norm} v_bcn = {v_bc_norm} diff = {np.linalg.norm(v_ab - v_bc)}")

    if a_avg_norm > config.GRAVITY_A_CUTOFF:
        return False, 0, np.zeros(3)
    
    interp = SE3Interpolate([t_a, t_b, t_c], [T_a, T_b, T_c])
    accels = []
    for a_local, w_norm, imu_stamp in zip(a_locals, w_norms, imu_stamps):
        stamp_s = float(imu_stamp) / 1e9
        
        if interp.in_bounds(stamp_s) and w_norm < config.GRAVITY_W_CUTOFF:
            print(f"w_norm = {w_norm} a_local = {a_local}")
            T_interp = interp.interpolate(stamp_s)
            accels.append(T_interp[:3,:3] @ a_local)
    
    print(f"len(accels) = {len(accels)}")
    if len(accels) > 5:
        a_w_mean = np.mean(np.stack(accels), axis=0)
        a_w_sub = a_w_mean - a_avg
        print(f"a_w_mean = {a_w_mean}")
        print(f"a_w_sub  = {a_w_sub}")
        return True, len(accels), a_w_sub

    else:
        return False, 0, np.zeros(3)
    
def T_local_gravity(g: np.ndarray) -> np.ndarray:

    dz = normalized(g)
    dx = normalized(np.cross(np.array([0,1,0]), dz))
    dy = normalized(np.cross(dz, dx))

    T = np.identity(4)
    T[:3,:3] = np.stack([dx, dy, dz]).T

    return T

class GravityNode(Node):

    def __init__(self):

        self.config = Config()
        super().__init__("gravity_node")
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cb_groub_pose = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self.pose_sub = self.create_subscription(TransformStamped, "/mvdb_tracker/tracker_node/pose", self.pose_cb, 10, callback_group=self.cb_groub_pose)
        self.active = not self.config.GRAVITY_BYPASS

        if self.active:
            self.cb_groub_imu = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
            self.imu_sub = self.create_subscription(Imu, self.config.IMU_TOPIC, self.imu_cb, 10, callback_group=self.cb_groub_imu)

            self.timer = self.create_timer(1, self.publish_pose)
            self.timer.cancel()

        self.imus = []
        self.poses = []

        self.g = np.array([0,0,10])
        self.T = np.identity(4)
        self.count = 0

        self.get_logger().info(f"gravity node started in state 'active = {self.active}'")

    def imu_cb(self, msg: Imu):

        if self.active:
            a = np.array(
                [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ])
            w = np.array(
                [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ])
            stamp = stamp_to_int(msg.header.stamp)
            self.imus.append((stamp, a, w))

    def pose_cb(self, msg: TransformStamped):

        if self.active:
            stamp = stamp_to_int(msg.header.stamp)
            pose = transform_msg_to_matrix(msg)

            self.poses.append((stamp, pose))

            self.get_logger().info(f"poses = {len(self.poses)} imus = {len(self.imus)}")

            if len(self.imus) > self.config.GRAVITY_PATH_CUTOFF:
                self.active = False
                self.set_g()
                self.get_logger().info(f"\n\n\nposes = {len(self.poses)} imus = {len(self.imus)}")
                self.get_logger().info(f"g = {self.g} count = {self.count} norm = {np.linalg.norm(self.g)}")
        
        else:
            self.publish_pose()

    def set_g(self):

        had_success = False
        count = 0

        T_as = self.poses[:-2]
        T_bs = self.poses[1:-1]
        T_cs = self.poses[2:]


        a_locals = [a for _,a,__ in self.imus]
        w_norms  = [np.linalg.norm(w) for _,__,w in self.imus]
        stamps   = [s for s,_,__ in self.imus]

        for (t_a, T_a), (t_b, T_b), (t_c, T_c) in zip(T_as[::2], T_bs[::2], T_cs[::2]):
            success, n, g = grav_stationary(T_a, T_b, T_c, t_a, t_b, t_c, a_locals, w_norms, stamps, self.config)
            if success:
                self.g =  float( count )  / float( count + n ) * self.g + n / float( count + n ) * g
                self.get_logger().info(f"succeeded setting gravity from stationary sequence to {self.g}")
                had_success = success
                count += n

                self.T = T_local_gravity(self.g)

        if not had_success:
            self.get_logger().info(f"failed setting gravity from stationary sequence! defaulting to {self.g}")
        
        self.count = count
        self.timer.reset()
    

    def publish_pose(self):
        drift_msg = matrix_to_transform_msg(self.T)
        drift_msg.header.stamp = self.get_clock().now().to_msg()
        drift_msg.header.frame_id = self.config.TRACKER_FRAME
        drift_msg.child_frame_id = "gravity"
        self.tf_broadcaster.sendTransform(drift_msg)

        drift_msg = matrix_to_transform_msg(np.linalg.inv(self.T))
        drift_msg.header.frame_id = "gravity_aligned"
        drift_msg.child_frame_id = "mapper_frame"
        drift_msg.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(drift_msg)

def main():

    rclpy.init()

    exec = rclpy.executors.MultiThreadedExecutor()

    gravity_node = GravityNode()

    exec.add_node(gravity_node)

    while rclpy.ok():
        exec.spin()
    for node in exec.get_nodes():
        node.destroy_node()

    rclpy.shutdown()
