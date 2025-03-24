import numpy as np
import open3d as o3d
import g2opy
from dataclasses import dataclass
from rclpy.impl.rcutils_logger import RcutilsLogger
from typing import List, Tuple, Generator
from mvdb_py.utils import pcd_to_points, delta_t
from .scd import SCD
from .config import Config

class Submap:

    def __init__(self, seq_id: int, timestamp: int, tracker_pose: np.ndarray, mapper_pose: np.ndarray, scan: o3d.geometry.PointCloud, track_distance: float):
        self.seq_id = seq_id
        self.timestamp = timestamp
        self.tracker_pose = tracker_pose
        self.mapper_pose = mapper_pose
        self.cloud = scan
        self.track_distance = track_distance

@dataclass
class ScdMatchResult:
    has_loop: bool = False
    res_idx: int = -1
    alignment: int = 0
    full_sim: float = 0
    T_init: np.ndarray = np.identity(4)

@dataclass
class LoopEdge:
    conf: float = .0
    n_a: int = -1
    n_b: int = -1
    t_a: int = -1
    t_b: int = -1
    T_a_b: np.ndarray = np.identity(4)

def lskip(l: List, idx: int) -> Generator:
    return ( l[x] for x in filter( lambda i: i!=idx, range( len(l) ) ) )

class Mapper():

    def __init__(self, config: Config, logger: RcutilsLogger):

        self.config = config
        self.logger = logger
        self.loop_max_dist = 8
        self.loop_min_path_dist = config.MAPPER_LOOP_MIN_DIST
        self.loop_window_length = 5

        self.voxel_size_fgr = config.MAPPER_LOOP_VOXELS
        self.voxel_size_icp = 0.125
        self.fitness_thresh = config.MAPPER_LOOP_FITNESS_THRESH
        
        self.tracker_drift = np.identity(4)
        self.icp_max_corr_distance = self.config.ICP_MAX_CORR_DIST

        self.loop_edges: List[LoopEdge] = []
        self.submap_list: List[Submap] = []

        self.scds = []
        self.scd_retrieve_keys_ar = None
        self.scd_cart = False
        self.scd_theta_res = config.SCD_THETA_R
        self.scd_r_res = config.SCD_R_R
        self.scd_r_scale = config.SCD_R_SCALE
        self.scd_voxels = config.SCD_VOXELS
        self.scd_ret_thresh = config.SCD_RETRIEVAL_THRESH
        self.scd_sim_thresh = config.SCD_SIM_THRESH

    def log(self, s: str):
        self.logger.info(s)

    def put_scan(self, timestamp: int, scan: o3d.geometry.PointCloud, tracker_pose: np.ndarray):

        if len(self.submap_list) == 0:

            track_distance = 0
            count = 0
            submap = Submap(count, timestamp, tracker_pose, tracker_pose, scan, track_distance)
            self.update_submaps_and_find_loops(submap)

        else:
            
            count = self.submap_list[-1].seq_id + 1
            T_last = self.submap_list[-1].tracker_pose
            stamp_last = self.submap_list[-1].timestamp

            if ( timestamp > stamp_last and delta_t(T_last, tracker_pose) > self.config.MAPPER_SUBMAP_THRESH ):
                track_distance = self.submap_list[-1].track_distance + delta_t(T_last, tracker_pose)
                submap = Submap(count, timestamp, tracker_pose, tracker_pose, scan, track_distance)
                self.update_submaps_and_find_loops(submap)
    
    def test_scan_for_scd_match(self, scan: o3d.geometry.PointCloud, mask: slice = slice(0), thresh_override: float = -1) -> ScdMatchResult:
        
        if thresh_override == -1:
            thresh_override = self.scd_ret_thresh

        result = ScdMatchResult()

        if len(self.scd_retrieve_keys_ar) > 0:

            scd = SCD(pcd_to_points(scan, self.scd_voxels), self.scd_cart, self.scd_r_res, self.scd_theta_res, self.scd_r_scale)
            ret_sim = self.ret_sim(scd.get_retrieve_key())
            ret_sim[mask] = 0
            max_idx = np.argmax(ret_sim)

            if ret_sim[max_idx] > thresh_override:
                if thresh_override > self.scd_ret_thresh:
                    self.log(f"[scd] modified ret thresh = {thresh_override:.4f} exceed; full check at ret = {ret_sim[max_idx]:.4f}")
                result.alignment = scd.align(self.scds[max_idx])
                result.full_sim = scd.full_compare(self.scds[max_idx], result.alignment)
                result.has_loop = result.full_sim > self.scd_sim_thresh
                if result.has_loop:
                    result.T_init = scd.get_transform(result.alignment)
                    result.res_idx = max_idx
        
        return result
    
    def check_loop(self, pcd: o3d.geometry.PointCloud, seq_id: int, idx_ref: int, T_init: np.ndarray, mode: str = "[odom]", voxel_override = -1, iter_override = 30):
        
        if voxel_override == -1:
            voxel_override = self.voxel_size_fgr

        loop_edge = None
        new_loop = False
        transform = np.identity(4)

        candidate_submap = self.submap_list[idx_ref]

        start_idx = max( idx_ref - self.loop_window_length // 2, 0 )
        stop_idx = min( start_idx + self.loop_window_length, len(self.submap_list) - 1 )

        latest_pcd = pcd.voxel_down_sample(voxel_override)

        loop_reference_points = o3d.geometry.PointCloud()
        for submap in self.submap_list[start_idx:stop_idx]:
            alignment_pose = np.linalg.inv(candidate_submap.mapper_pose) @ submap.mapper_pose
            pcd = o3d.geometry.PointCloud(submap.cloud)
            pcd.transform(alignment_pose)
            loop_reference_points.points.extend(pcd.points)
        loop_reference_pcd = loop_reference_points.voxel_down_sample(voxel_override)

        latest_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_override * 2, max_nn=30))
        loop_reference_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_override * 2, max_nn=30))

        reg = o3d.pipelines.registration.registration_generalized_icp(
            latest_pcd, 
            loop_reference_pcd, 
            self.icp_max_corr_distance,
            T_init, 
            estimation_method=o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iter_override)
        )

        self.log(f"\n{mode} attempting loop closure between submaps {seq_id} -> {idx_ref}; fitness = {reg.fitness:.4f}")
        if reg.fitness > self.fitness_thresh:
            self.log(f"{mode} Performing loop closure between submaps {seq_id} -> {idx_ref}; fitness = {reg.fitness:.4f}")
            loop_edge: Tuple[np.ndarray, int, int] = (reg.transformation, idx_ref, seq_id)
            new_loop = True
            transform = reg.transformation
            
        return new_loop, loop_edge, transform, reg.fitness
    
    def find_loop_candidate_odom(self, idx: int, T_init = np.identity(4)) -> Tuple[bool,int]:

        candidate_found = False
        candidate_idx = -1
        candidate_dist = np.inf

        latest_submap = self.submap_list[idx]

        for other_submap in lskip(self.submap_list, idx):

            path_dist = abs(latest_submap.track_distance - other_submap.track_distance)
            relative_dist = np.linalg.norm((T_init @ latest_submap.mapper_pose)[:3,3] - other_submap.mapper_pose[:3,3])

            if path_dist > self.loop_min_path_dist and relative_dist < self.loop_max_dist:
                candidate_found = True                
                if candidate_dist > relative_dist:
                    candidate_idx = other_submap.seq_id
                    candidate_dist = relative_dist

        return candidate_found, candidate_idx
    
    def ret_sim(self, lookup_key: np.ndarray) -> np.ndarray:
        ret_dot = np.tensordot(lookup_key, self.scd_retrieve_keys_ar, [[0], [-1]])
        ret_norm = np.linalg.norm(self.scd_retrieve_keys_ar, axis=-1) * np.linalg.norm(lookup_key) + np.finfo(float).eps
        ret_sim = ret_dot / ret_norm
        return ret_sim
    
    def add_loop(self, loop_edge: Tuple[np.ndarray, int, int], fitness: float):
        T_a_b, n_a, n_b = loop_edge
        s_a = self.submap_list[n_a]
        s_b = self.submap_list[n_b]
        t_a = s_a.timestamp
        t_b = s_b.timestamp
        self.loop_edges.append(LoopEdge(fitness, n_a, n_b, t_a, t_b, T_a_b))
    
    def update_submaps_and_find_loops(self, latest_submap: Submap):

        latest_submap.mapper_pose = self.tracker_drift @ latest_submap.tracker_pose

        self.submap_list.append(latest_submap)
        self.scds.append(
            SCD(pcd_to_points(latest_submap.cloud, self.scd_voxels), self.scd_cart, self.scd_r_res, self.scd_theta_res, self.scd_r_scale)
        )
        if self.scd_retrieve_keys_ar is None:
            self.scd_retrieve_keys_ar = np.array([self.scds[-1].get_retrieve_key()])
        else:
            self.scd_retrieve_keys_ar = np.concatenate([self.scd_retrieve_keys_ar, [self.scds[-1].get_retrieve_key()]], axis=0)

        new_loop = False
        new_loop_idx = -1
        updated_poses = [x.mapper_pose for x in self.submap_list]

        masked_range = int(self.loop_min_path_dist // self.config.MAPPER_SUBMAP_THRESH) + 1
        idx = latest_submap.seq_id

        scd_result = self.test_scan_for_scd_match(latest_submap.cloud, slice(idx-masked_range, idx+masked_range))
        candidate_found_scd = scd_result.has_loop
        candidate_idx_scd = scd_result.res_idx
        candidate_sim_scd = scd_result.full_sim
        candidate_T_scd = scd_result.T_init

        if candidate_found_scd and candidate_idx_scd != new_loop_idx:
            self.log(f"found scd candidate {latest_submap.seq_id} -> {candidate_idx_scd}; similarity = {candidate_sim_scd}")
            loop_found, loop_edge, transform, fitness = self.check_loop(latest_submap.cloud, latest_submap.seq_id, candidate_idx_scd, candidate_T_scd, "[scd ]")
            if loop_found:
                new_loop = True
                new_loop_idx = candidate_idx_scd
                self.add_loop(loop_edge, fitness)

        if not new_loop:
            candidate_found_odom, candidate_idx_odom = self.find_loop_candidate_odom(latest_submap.seq_id)
            if candidate_found_odom and candidate_found_odom != new_loop_idx:
                self.log(f"found odm candidate {latest_submap.seq_id} -> {candidate_idx_odom}")
                T_init = np.linalg.inv(self.submap_list[candidate_idx_odom].mapper_pose) @ latest_submap.mapper_pose
                loop_found, loop_edge, transform, fitness = self.check_loop(latest_submap.cloud, latest_submap.seq_id, candidate_idx_odom, T_init)
                if loop_found:
                    new_loop = True
                    new_loop_idx = candidate_idx_odom
                    self.add_loop(loop_edge, fitness)
        

        if new_loop:
            updated_poses = self.optimize_submaps(self.submap_list, self.loop_edges)

            for pose, submap in zip(updated_poses, self.submap_list):

                transform = np.linalg.inv(pose) @ submap.mapper_pose
                submap.mapper_pose = pose

                if np.linalg.norm(transform[:3,3]) > self.loop_max_dist:

                    featured = list(filter(lambda x: x.n_b == submap.seq_id, self.loop_edges))

                    if len(featured) == 0:
                        candidate_found_retrace, candidate_idx_retrace = self.find_loop_candidate_odom(submap.seq_id)

                        if candidate_found_retrace:
                            T_init = np.linalg.inv(self.submap_list[candidate_idx_retrace].mapper_pose) @ submap.mapper_pose
                            another_loop, loop_edge, transform, fitness = self.check_loop(submap.cloud, submap.seq_id, candidate_idx_retrace, T_init)

                            self.log(f"Found another loop candidate {submap.seq_id} -> {candidate_idx_retrace}; fitness = {fitness}; accepted = {another_loop}")

                            if another_loop:
                                self.log(f"Loop inserted {submap.seq_id} -> {candidate_idx_retrace}")
                                self.add_loop(loop_edge, fitness)

        self.tracker_drift = updated_poses[-1] @ np.linalg.inv(latest_submap.tracker_pose)

        return updated_poses
 
    def optimize_submaps(self, submap_list: List[Submap], loop_edges: List[LoopEdge]) -> List[np.ndarray]:

        opt = g2opy.SparseOptimizer()
        solver = g2opy.OptimizationAlgorithmLevenberg(
            g2opy.BlockSolverSE3(g2opy.LinearSolverCholmodSE3())
            )
        opt.set_algorithm(solver)
        LIDAR_COVARIANCE = np.identity(6)

        for i, submap in enumerate(submap_list):
            submap_pose_vertex = g2opy.VertexSE3()
            submap_pose_vertex.set_estimate(g2opy.Isometry3d(submap.mapper_pose))
            submap_pose_vertex.set_id(i)
            if i == 0:
                submap_pose_vertex.set_fixed(True)
            opt.add_vertex(submap_pose_vertex)

        for i, (prev_submap, next_submap) in enumerate(zip(submap_list[:-1], submap_list[1:])):
            odom_edge = g2opy.EdgeSE3()
            odom_constraint = np.linalg.inv(prev_submap.tracker_pose) @ next_submap.tracker_pose
            odom_edge.set_measurement(g2opy.Isometry3d(odom_constraint))
            odom_edge.set_information(LIDAR_COVARIANCE)
            odom_edge.set_vertex(0, opt.vertex(i))
            odom_edge.set_vertex(1, opt.vertex(i+1))
            opt.add_edge(odom_edge)

        for _edge in loop_edges:

            constraint = _edge.T_a_b
            submap_idx_a = _edge.n_a
            submap_idx_b = _edge.n_b

            loop_edge = g2opy.EdgeSE3()
            loop_edge.set_measurement(g2opy.Isometry3d(constraint))
            odom_edge.set_information(LIDAR_COVARIANCE)
            loop_edge.set_vertex(0, opt.vertex(submap_idx_a))
            loop_edge.set_vertex(1, opt.vertex(submap_idx_b))
            opt.add_edge(loop_edge)
        
        opt.initialize_optimization()
        opt.optimize(1000)

        updated_poses = []
        for i in range(len(submap_list)):
            updated_poses.append(opt.vertex(i).estimate().matrix())
        
        return updated_poses