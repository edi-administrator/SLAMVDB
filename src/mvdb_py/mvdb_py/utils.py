import numpy as np
import open3d as o3d
from typing import Tuple
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped

def stamp_to_int(stamp: Time) -> int:
    return int(1e9) * stamp.sec + stamp.nanosec

def int_to_stamp(nsec_epoch: int) -> Time:
    stamp = Time()
    stamp.sec = nsec_epoch // int(1e9)
    stamp.nanosec = nsec_epoch % int(1e9)
    return stamp

def delta_T(T_a: np.ndarray, T_b: np.ndarray) -> np.ndarray:
    return np.linalg.inv(T_a) @ T_b

def delta_t(T_a: np.ndarray, T_b: np.ndarray) -> np.ndarray:
    return np.linalg.norm(delta_T(T_a, T_b)[:3,3])

def pointcloud_to_np(msg: PointCloud2) -> Tuple[np.ndarray, np.ndarray]:
    '''
    This only works with pcd's that have only points, height 1 and dtype float32
    '''
    fields = msg.fields

    step = msg.point_step
    size = 4
    x_offs = next(filter(lambda f: f.name == "x", fields)).offset
    y_offs = next(filter(lambda f: f.name == "y", fields)).offset
    z_offs = next(filter(lambda f: f.name == "z", fields)).offset

    has_color = step == 6 * 4

    if has_color:
        r_offs = next(filter(lambda f: f.name == "r", fields)).offset
        g_offs = next(filter(lambda f: f.name == "g", fields)).offset
        b_offs = next(filter(lambda f: f.name == "b", fields)).offset

    buf = np.frombuffer(msg._data, np.uint8).reshape(-1,step)
    x = buf[:,x_offs:x_offs+size]
    y = buf[:,y_offs:y_offs+size]
    z = buf[:,z_offs:z_offs+size]
    ars = [x,y,z]

    if has_color:
        r = buf[:,r_offs:r_offs+size]
        g = buf[:,g_offs:g_offs+size]
        b = buf[:,b_offs:b_offs+size]
        ars += [r,g,b]


    for ar in ars:
        ar.dtype = np.float32

    points = np.concatenate(ars[:3], axis=-1)
    colors = None
    if has_color:
        colors = np.concatenate(ars[3:], axis=-1)

    return points, colors

def pointcloud_to_pcd(msg: PointCloud2, crop = False, crop_radius = 1.5) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pts, colors = pointcloud_to_np(msg)
    if crop:
        mask = np.linalg.norm(pts, axis=-1) > crop_radius
        pts = pts[mask]
        if not colors is None:
            colors = colors[mask]
    pcd.points = o3d.utility.Vector3dVector(pts)
    if not colors is None:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

def pcd_to_points(pcd: o3d.geometry.PointCloud, ds = None) -> np.ndarray:
    if not ds is None:
        pcd = pcd.voxel_down_sample(ds)
    return np.asarray(pcd.points)

def pcd_to_pointcloud(pcd: o3d.geometry.PointCloud, color: bool = False) -> PointCloud2:

    msg = PointCloud2()
    points = np.asarray(pcd.points).astype(np.float32)
    if color:
        colors = np.asarray(pcd.colors).astype(np.float32)

    idx_range, tags = (6, "xyzrgb") if color else (3, "xyz")
    for offs,c in zip(range(idx_range),tags):
        field = PointField()
        field.count = 1
        field.datatype = PointField.FLOAT32
        field.offset = offs * 4
        field.name = c
        msg.fields.append(field)

    msg.height = 1
    msg.width = points.shape[0]
    msg.point_step = idx_range * 4
    msg.row_step = msg.width * msg.point_step

    if color:
        buf_pts = points.view(np.uint8).reshape(-1,12)
        buf_clr = colors.view(np.uint8).reshape(-1,12)
        buf = np.concatenate([buf_pts, buf_clr], axis=-1).tobytes()
    else:
        buf = points.tobytes()
    msg._data = buf

    return msg

def matrix_to_transform_msg(T: np.ndarray) -> TransformStamped:

    msg = TransformStamped()
    q = Rotation.from_matrix(T[:3,:3].copy()).as_quat()
    t = T[:3,3].copy()

    msg.transform._rotation.x = q[0]
    msg.transform._rotation.y = q[1]
    msg.transform._rotation.z = q[2]
    msg.transform._rotation.w = q[3]
    msg.transform._translation.x = t[0]
    msg.transform._translation.y = t[1]
    msg.transform._translation.z = t[2]

    return msg

def transform_msg_to_matrix(msg: TransformStamped) -> np.ndarray:

    T = np.identity(4)
    R = Rotation.from_quat([
        msg.transform.rotation.x, 
        msg.transform.rotation.y, 
        msg.transform.rotation.z, 
        msg.transform.rotation.w]).as_matrix()
    
    t = np.array([
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z]
        )
    
    T[:3,:3] = R
    T[:3,3] = t

    return T

def dump_pts(path: str, pts: o3d.geometry.PointCloud):
    o3d.io.write_point_cloud(path, pts)

def main():
    print(f"package reachable!")