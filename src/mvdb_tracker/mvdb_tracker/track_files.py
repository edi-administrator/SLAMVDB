import numpy as np
import glob
import open3d as o3d
from os import remove, makedirs
from os.path import exists
from mvdb_py.utils import points_to_pcd
from .config import Config
from .tracker import Tracker


def track_in_files():

    config = Config()
    tracker = Tracker(config)

    point_files = sorted(glob.glob(f"{config.TRACK_FILES_IN}/points*.npz"))

    existing_out_files = glob.glob(f"{config.TRACK_FILES_OUT}/poses_*.csv")
    for exf in existing_out_files:
        if exists(exf):
            print(f"removing existing pose at: {exf}")
            remove(exf)

    if not exists(config.TRACK_FILES_OUT):
        print(f"creating output directory: {config.TRACK_FILES_OUT}")
        makedirs(config.TRACK_FILES_OUT)

    for point_file in point_files:

        stamp = point_file.split("_")[-1].replace(".npz", "")

        points = points_to_pcd(np.load(point_file)["points"])

        tracker.insert_scan(points)
        pose = tracker.T_latest

        path = f"{config.TRACK_FILES_OUT}/poses_{stamp}.csv"
        np.savetxt(path, pose, delimiter=",")
        print(f"saving pose to {path}")


if __name__ == "__main__":
    track_in_files()