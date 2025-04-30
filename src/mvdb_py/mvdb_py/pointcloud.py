import numpy as np
import rclpy
from rclpy.node import Node
import open3d as o3d
from typing import Set, Iterable, Callable, AnyStr
from sensor_msgs.msg import PointCloud2, PointField

class PointCloudNP:

    field_dtype_map = {
        1 : np.int8,
        2 : np.uint8,
        3 : np.int16,
        4 : np.uint16,
        5 : np.int32,
        6 : np.uint32,
        7 : np.float32,
        8 : np.float64
    }

    def __init__(
            self, 
            msg: PointCloud2, 
            row_step: int = 1, 
            col_step: int = 1,
            col_major: bool = False, 
            field_filter: Iterable[AnyStr] = ["x", "y", "z", "r", "g", "b"]
        ):
        
        rows = msg.height
        cols = msg.width
        point_step = msg.point_step

        size = ( rows // row_step ) * ( cols // col_step )

        fields = self.parse_fields(msg.fields, field_filter)

        if col_major:
            data_buffer = np.frombuffer(msg._data, np.uint8).reshape(cols,rows,point_step).transpose(1,0,2)
        else:
            data_buffer = np.frombuffer(msg._data, np.uint8).reshape(rows,cols,point_step)

        assert "x" in fields
        assert "y" in fields
        assert "z" in fields

        for key, (dtype, offset) in fields.items():
            field_array = self.arbuffer(data_buffer, offset, dtype.itemsize, dtype, row_step, col_step)
            setattr(self, key, field_array.reshape(size))
        

        self.points = np.stack([self.x, self.y, self.z], axis=-1)

        if "r" in fields and "g" in fields and "b" in fields:
            self.colors = np.stack([self.r, self.g, self.b], axis=-1)
            self.has_color = True
        else:
            self.colors = None
            self.has_color = False


    @staticmethod
    def parse_fields(msg_fields: Iterable[PointField], filter_set: Set):
        fields = {}
        for field in msg_fields:
            if field.name in filter_set:
                fields[field.name] = [
                    np.dtype(PointCloudNP.field_dtype_map[field.datatype]),
                    field.offset,
                ]
        return fields

    
    @staticmethod
    def arbuffer(buf: np.ndarray, offset: int, size: int, dtype: np.dtype, row_step: int = 1, col_step: int = 1):
        assert buf.dtype == np.uint8
        ar = buf[::row_step,::col_step,offset:offset+size]
        ar.dtype = dtype
        return ar
    
    def _mask(self, mask: np.ndarray):

        for attr in self.__dict__.keys():
            item = getattr(self, attr)
            if type(item) is np.ndarray:
                ar = item[mask]
                setattr(self, attr, ar)

        return self

    def crop(self, radius: float = 0.5):
        norms = np.linalg.norm(self.points, axis=-1)
        return self._mask(norms > radius)

    def azimuth_range(self, theta_start: float | None, theta_end: float | None):

        if theta_start is None or theta_end is None:
            return self

        assert -np.pi <= theta_start <= np.pi
        assert -np.pi <= theta_end <= np.pi

        reverse = theta_end < theta_start
        
        thetas = np.arctan2(self.points[:,1], self.points[:,0])
        
        if reverse:
            theta_end %= 2 * np.pi
            thetas %= 2 * np.pi
        
        mask = (thetas >= theta_start) & (thetas <= theta_end)

        return self._mask(mask)    
    

    def o3d(self, color_by_field: str | None = None, color_cb: Callable | None = None) -> o3d.geometry.PointCloud:

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points.astype(np.float64))

        if self.has_color:
            pcd.colors = o3d.utility.Vector3dVector(self.colors.astype(np.float64))
        
        elif not color_by_field is None:

            color_by_field: str
            assert not color_cb is None

            color_cb: Callable

            color_array = color_cb(getattr(self, color_by_field))

            pcd.colors = o3d.utility.Vector3dVector(color_array.astype(np.float64))

        return pcd


class Vis:

    def __init__(self):
        self.pcdnp = None

    def parse_msg(self, msg: PointCloud2) -> PointCloudNP:
        self.pcdnp = PointCloudNP(msg, row_step=1, field_filter=set(["x", "y", "z", "intensity", "t"])).crop().azimuth_range(1, -1)


def intensity_cb(ar: np.ndarray) -> np.ndarray:

    M = ar.max()
    m = ar.min()
    s = (M - m)

    return ( ( ar.astype(float) - m )/ s ).reshape(-1,1).repeat(3, -1)


if __name__ == "__main__":

    rclpy.init()
    node = Node("subnode")
    v = Vis()
    sub = node.create_subscription(PointCloud2, "/points", v.parse_msg, 10)

    while rclpy.ok() and v.pcdnp is None:
         rclpy.spin_once(node)

    o3d.visualization.draw_geometries([v.pcdnp.o3d(color_by_field="t", color_cb=intensity_cb)])