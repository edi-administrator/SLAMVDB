from mvdb_py.config import ConfigBase

class Config(ConfigBase):

    def __init__(self, package_name = "mvdb_tracker"):
        super().__init__(package_name=package_name)

    def instance_defaults(self):
        self.test_key: str = "not set!"
        self.TRACKER_NODE_NAME: str = "tracker_node"
        self.INPUT_TOPIC: str = "/points"
        self.TRACKER_FRAME: str = "tracker_frame"
        self.ROBOT_FRAME: str = "robot"
        self.OUTPUT_TOPIC: str = "tracker_node/pose"
        self.VISUALIZATION_TOPIC: str = "tracker_node/transformed_points"
        self.TRACKER_VOXELS: float = 0.125
        self.TRACKER_SUBMAP_THRESH: float = 2.0
        self.TRACKER_HISTORY_SIZE: int = 6
        self.ICP_MAX_CORR_DIST: float = 1.0
        self.CROP_RADIUS: float = 1.5
