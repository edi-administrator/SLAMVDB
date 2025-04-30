import numpy as np
import glob
import open3d as o3d
import rclpy
from os.path import exists
from os import remove
from rclpy.node import Node
from mvdb_py.utils import SE3Interpolate, points_to_pcd
from .config import Config
from .mapper import Mapper
# from config import Config
# from mvdb_mapper.mapper import Mapper

def stamp_from_path(path: str, ext: str):
    return int(path.split("_")[-1].replace(ext, ""))

def map_in_files():

    rclpy.init()
    logger_node = Node("file_mapper_logger_node")

    config = Config()
    mapper = Mapper(config, logger_node.get_logger())

    existing_pose_files = glob.glob(f"{config.MAP_FILE_POSES_OUT}/poses*csv")
    for exf in existing_pose_files:
        if exists(exf):
            print(f"removing existing pose at: {exf}")
            remove(exf)

    point_files = sorted(glob.glob(f"{config.MAP_FILE_POINTS_IN}/points*.npz"))
    pose_files = sorted(glob.glob(f"{config.MAP_FILE_POSES_IN}/poses*.csv"))

    point_stamps = np.array([stamp_from_path(p, ".npz") for p in point_files], dtype=np.uint64)
    pose_stamps = np.array([stamp_from_path(p, ".csv") for p in pose_files], dtype=np.uint64)

    assert(len(pose_files) == len(point_files))
    assert((point_stamps == pose_stamps).all())

    tracker_poses = []
    for pose_file in pose_files:
        tracker_poses.append(np.loadtxt(pose_file, delimiter=",").reshape(4,4))

    for point_file, tracker_pose, stamp in zip(point_files, tracker_poses, pose_stamps):
        # logger_node.get_logger().info(f"putting scan: {stamp}")
        points = points_to_pcd(np.load(point_file)["points"])
        mapper.put_scan(stamp, points, tracker_pose)

    mapper_poses = []
    mapper_stamps = []
    for sbmp in mapper.submap_list:
        mapper_poses.append(sbmp.mapper_pose)
        mapper_stamps.append(sbmp.timestamp)
    
    mapper_stamps = np.stack(mapper_stamps)
    mapper_poses = np.stack(mapper_poses)


    tracker_interp = SE3Interpolate(pose_stamps, tracker_poses)

    for tracker_pose, tracker_stamp in zip(tracker_poses, pose_stamps):

        prior_mapper_stamps = (mapper_stamps <= tracker_stamp)

        if prior_mapper_stamps.any():

            last_mapper_pose = mapper_poses[prior_mapper_stamps][-1]
            last_mapper_stamp = mapper_stamps[prior_mapper_stamps][-1]

            last_tracker_pose = tracker_interp.interpolate(last_mapper_stamp)

            relative_tracker_pose = np.linalg.inv(last_tracker_pose) @ tracker_pose
            propagated_mapper_pose = last_mapper_pose @ relative_tracker_pose

            path = f"{config.MAP_FILE_POSES_OUT}/poses_{tracker_stamp:018d}.csv"

            # print(f"saving propagated mapper pose: {path}")
            # print(f"current tracker stamp {tracker_stamp:018d}")
            # print(f"previous mapper stamp {last_mapper_stamp:018d}")

            np.savetxt(path, propagated_mapper_pose, delimiter=",")    


if __name__ == "__main__":
    map_in_files()