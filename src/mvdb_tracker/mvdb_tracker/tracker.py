import numpy as np
import open3d as o3d
from typing import List
from .config import Config

class Submap:

    def __init__(self, seq_id: int, pose: np.ndarray, scan: o3d.geometry.PointCloud, track_distance: float):
        self.seq_id = seq_id
        self.track_distance = track_distance
        self.tracker_pose = pose
        self.cloud = scan

class Tracker():

    def __init__(self, config: Config):

        self.config = config

        self.voxel_size = config.TRACKER_VOXELS
        self.submap_thresh = config.TRACKER_SUBMAP_THRESH
        self.history_size = config.TRACKER_HISTORY_SIZE
        self.icp_max_corr_distance = config.ICP_MAX_CORR_DIST

        self.T_latest = np.identity(4)
        self.submap_list: List[Submap] = []
        self.map_initialized = False
        self.map_updated = False
        self.reference_points = None
        self.cooldown = 0

        self.fitness = 0.0

    def insert_scan(self, scan: o3d.geometry.PointCloud):

        if not self.map_initialized:

            submap = Submap(0, self.T_latest, scan, 0)

            self.submap_list.append(submap)
            self.map_initialized = True
            self.map_updated = True

            self.reference_points = scan.voxel_down_sample(self.voxel_size)

        else:

            downsampled_scan = scan.voxel_down_sample(self.voxel_size)

            reg = o3d.pipelines.registration.registration_icp(
                downsampled_scan, 
                self.reference_points, 
                self.icp_max_corr_distance, 
                self.T_latest, 
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
            )

            T_new_corrected_icp = reg.transformation
            self.fitness = reg.fitness
            
            self.T_latest = T_new_corrected_icp

            delta = np.linalg.norm(self.T_latest[:3,3] - self.submap_list[-1].tracker_pose[:3,3])
            track_distance = self.submap_list[-1].track_distance + delta
            seq_id = self.submap_list[-1].seq_id + 1

            if delta > self.submap_thresh:
                
                submap = Submap(
                    seq_id, 
                    self.T_latest, 
                    scan, 
                    track_distance
                )

                self.submap_list.append(submap)
                self.map_updated = True
                self.submap_list = self.submap_list[-self.history_size:]

                self.reference_points.clear()
                for submap in self.submap_list:
                    downsample = submap.cloud.voxel_down_sample(self.voxel_size)
                    downsample.transform(submap.tracker_pose)                    
                    self.reference_points.points.extend(downsample.points)
            
    def submap_list_updated(self) -> bool:
        return self.map_updated

    def get_submap_list(self) -> List[Submap]:
        self.map_updated = False
        return self.submap_list
            
    def get_reference_points(self) -> o3d.geometry.PointCloud:
        return o3d.geometry.PointCloud(self.reference_points)

    def get_fitness(self) -> float:
        return self.fitness
