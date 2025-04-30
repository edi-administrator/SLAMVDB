import numpy as np
import open3d as o3d
from typing import Tuple, List, Any, SupportsFloat
from scipy.spatial.transform import Rotation, Slerp
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
    return delta_T(T_a, T_b)[:3,3]

def delta_t_norm(T_a: np.ndarray, T_b: np.ndarray) -> np.ndarray:
    return np.linalg.norm(delta_t(T_a, T_b))

def normalized(v: np.ndarray) -> np.ndarray:
    return v / ( np.linalg.norm(v) + np.finfo(float).eps )

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

def points_to_pcd(points: np.ndarray, colors: np.ndarray | None = None, crop_radius: float | None = 1.5) -> o3d.geometry.PointCloud:
    
    if not crop_radius is None:

        mask = np.linalg.norm(points, axis=-1) > crop_radius

        points = points[mask]
        
        if not colors is None:
            colors = colors[mask]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if not colors is None:
        pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd

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

class Linterp:

    def __init__(self, values: List[np.ndarray], stamps: List[SupportsFloat]):
        self.values = [v.copy() for v in values]
        self.times = np.array([self.sec(t) for t in stamps])
        self.indices = np.arange(len(self.values))
    
    @staticmethod
    def sec(stamp_ns: int):
        return float(stamp_ns) / 1e9
    
    def in_bounds(self, t):
        return self.times[0] <= self.sec(t) <= self.times[-1]

    def bounds(self):
        return self.times[0], self.times[-1]
    
    def interpolate(self, t):
        
        t_s = self.sec(t)

        lbound = self.indices[(self.times <= t_s)][-1]
        ubound = self.indices[(self.times >= t_s)][0]

        if lbound == ubound:
            return self.values[lbound]

        coeff = ( t_s - self.times[lbound] ) / ( self.times[ubound] - self.times[lbound] + np.finfo(float).eps )

        return ( 1 - coeff ) * self.values[lbound] + coeff * self.values[ubound]


class SE3Interpolate:

    def __init__(self, stamps: List[int], poses: List[np.ndarray]):
        
        assert(all(x.shape == (4,4) for x in poses))

        ts = [T[:3,3] for T in poses]
        self.t_interp = Linterp(ts, stamps)

        Rs = Rotation.from_matrix([T[:3,:3] for T in poses])
        self.slerp = Slerp(stamps, Rs)

    def in_bounds(self, t: int):
        return self.t_interp.in_bounds(t)
    
    def interpolate(self, t: int):
        
        if not self.t_interp.in_bounds(t):
            raise ArithmeticError(f"Tried to interpolate out of bounds! lbound = {self.t_interp.bounds()[0]} t = {self.t_interp.sec(t)} ubound = {self.t_interp.bounds()[-1]}")

        T = np.identity(4)
        T[:3,:3] = self.slerp(t).as_matrix()
        T[:3,3] = self.t_interp.interpolate(t)
        
        return T

def _pose_cloud(scale: float = 1.0, count: float = 10.):

    points = []
    colors = []

    for axis in range(3):
        for da in range(int(count)):
            pt = np.zeros(3)
            cl = np.zeros(3)
            cl[axis] = 1.0
            pt[axis] = da
            points.append(pt * scale / count)
            colors.append(cl)
    
    return np.stack(points), np.stack(colors)

class PoseMarker:

    def __init__(self, scale = 1.0, count = 10.0):
        points, colors = _pose_cloud(scale, count)
        self._cloud = points_to_pcd(points, colors, crop_radius=None)
    
    def at(self, pose: np.ndarray):
        out = o3d.geometry.PointCloud(self._cloud)
        out.transform(pose)
        return out