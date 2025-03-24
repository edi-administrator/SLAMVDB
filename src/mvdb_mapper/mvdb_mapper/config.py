from mvdb_py.config import ConfigBase

class Config(ConfigBase):

    def __init__(self, package_name = "mvdb_mapper"):
        super().__init__(package_name=package_name)

    def instance_defaults(self):
        self.MAPPER_PUBLISH_MAXCOUNT: int = 1000
        self.MAPPER_NODE_NAME: str = "mapper_node"
        self.INPUT_POINTS_TOPIC: str = "/points"
        self.TRACKER_POSE_TOPIC: str = "/mvdb_tracker/tracker_node/pose"
        self.MAPPER_CORRECTION_RUNNING_TOPIC: str = "mapper_node/corrected_pose"
        self.MAPPER_LOOP_TOPIC: str = "mapper_node/loops"
        self.MAPPER_FRAME: str = "mapper_frame"
        self.TRACKER_FRAME: str = "tracker_frame"
        self.MAPPER_LOOP_VOXELS: float = 0.25
        self.MAPPER_LOOP_FITNESS_THRESH: float = 0.985
        self.MAPPER_SUBMAP_THRESH: float = 4.0
        self.MAPPER_LOOP_MIN_DIST: float = 30.0
        self.ICP_MAX_CORR_DIST: float = 1.0
        self.CROP_RADIUS: float = 1.5
        self.SCD_THETA_R: float = 120
        self.SCD_R_R: float = 40
        self.SCD_R_SCALE: float = 0.5
        self.SCD_VOXELS: float = 0.5
        self.SCD_RETRIEVAL_THRESH: float = 0.94
        self.SCD_SIM_THRESH: float = 0.30
        self.DUMP_POSES: bool = False
        self.DUMP_LOOPS: bool = False
