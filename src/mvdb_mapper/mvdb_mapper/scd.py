import numpy as np

class SCD():

    def __init__(self, points: np.ndarray, cartesian = False, r_res = 30, theta_res = 60, r_scale = 2.5, sqrt_radius = False):
        self.cartesian = cartesian
        self.r_scale = r_scale
        self.r_res = r_res
        self.theta_res = theta_res
        self.mat_theta_r = np.zeros((theta_res + 1, r_res + 1))
        self.sqrt_radius = sqrt_radius

        self.populate_mat(points)
        self.align_key = self.create_align_key()
        self.retrieve_key = self.create_retrieve_key()

    def populate_mat(self, points: np.ndarray):

        if self.cartesian:

            points_theta = (points[:,1] / self.r_scale).astype(int) + self.theta_res // 2
            points_r = (points[:,0] / self.r_scale).astype(int) + self.r_res // 2

            points_theta = np.where(points_theta >= self.theta_res, self.theta_res, points_theta)
            points_theta = np.where(points_theta < 0, 0, points_theta)

            points_r = np.where(points_r >= self.r_res, self.r_res, points_r)
            points_r = np.where(points_r < 0, 0, points_r)

        else:

            points_theta = ( np.arctan2(points[:,1], points[:,0]) * self.theta_res / ( 2 * np.pi ) ).astype(int)

            if self.sqrt_radius:
                points_r = (np.sqrt(np.linalg.norm(points[:,:2], axis=-1) / self.r_scale)).astype(int)
            else:
                points_r = (np.linalg.norm(points[:,:2], axis=-1) / self.r_scale).astype(int)
            points_r = np.where(points_r >= self.r_res, self.r_res, points_r)

        points_z = points[:,2].astype(int)

        for point_theta, point_r, point_z in zip(points_theta, points_r, points_z):
            self.mat_theta_r[point_theta, point_r] = max(point_z, self.mat_theta_r[point_theta, point_r])

        self.mat_theta_r = self.mat_theta_r[:self.theta_res,:self.r_res]

    def create_retrieve_key(self) -> np.ndarray:
        return np.sum(np.abs(self.mat_theta_r), axis=0)

    def create_align_key(self) -> np.ndarray:
        key = np.sum(np.abs(self.mat_theta_r), axis=-1)
        permutations = []
        for i in range(len(key)):
            permutations.append(np.roll(key, i))
        return np.stack(permutations)

    def get_align_key(self) -> np.ndarray:
        return self.align_key[0]
    
    def get_retrieve_key(self) -> np.ndarray:
        return self.retrieve_key
    
    def get_scd(self) -> np.ndarray:
        return self.mat_theta_r
    
    def get_transform(self, shift: int) -> np.ndarray:
        theta = ( shift / self.theta_res ) * 2 * np.pi
        transform = np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta),  np.cos(theta), 0, 0],
            [            0,              0, 1, 0],
            [            0,              0, 0, 1],
        ])
        return transform

    def retrieve_compare(self, other) -> float:
        other_key = other.get_retrieve_key()
        return np.dot(self.retrieve_key, other_key) / ( np.linalg.norm(self.retrieve_key) * np.linalg.norm(other_key) + np.finfo(float).eps )

    def align(self, other) -> int:
        other_key = other.get_align_key()
        return np.argmax(np.tensordot(self.align_key, other_key, [[-1], [-1]]))

    def full_compare(self, other, shift: int) -> float:

        rolled = np.roll(self.mat_theta_r, shift, axis=0)
        other_scd = other.get_scd()

        good_idxs_reduced = np.sum(rolled, axis=-1) * np.sum(other_scd, axis=-1) > 0

        dots = np.tensordot(rolled, other_scd, [[-1], [-1]]).diagonal()
        norms = np.linalg.norm(rolled, axis=-1) * np.linalg.norm(other_scd, axis=-1) + np.finfo(float).eps
        column_wise_cosine_similarity = dots / norms 

        sim_reduced = np.sum( column_wise_cosine_similarity[good_idxs_reduced] ) / good_idxs_reduced.astype(int).sum()

        return sim_reduced