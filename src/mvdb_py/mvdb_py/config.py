import yaml
from os.path import exists
from ament_index_python.packages import get_package_share_directory

class ConfigBase:

    def __init__(self, package_name: str = ""):

        self.instance_defaults()

        base_directory = get_package_share_directory(package_name)

        paths = [
            f"{base_directory}/config/config.yaml",
            f"./{package_name}_config.yaml"
        ]

        for path in paths:

            if exists(path):

                print(f"{package_name} config reading from: {path}")

                with open(path, 'r') as f:
                    config = yaml.safe_load(f)
                
                if not config is None:
                    for k,v in config.items():
                        if hasattr(self, k):
                            setattr(self, k , v)
                        else:
                            raise NotImplementedError(f"Config for package '{package_name}' has no attribute {k}")
            else:
                print(f"{package_name} config path not found: {path}")
    
    def instance_defaults():
        raise NotImplementedError("ConfigBase cannot be used directly, need to set defaults")


class ConfigPCD(ConfigBase):

    def instance_defaults(self):
        self.CROP_RADIUS: float = 1.5
        self.CROP_THETA_START: float | None = None
        self.CROP_THETA_END: float | None = None
        self.CROP_ROW_STEP: int = 1
        self.CROP_COL_STEP: int = 1
        self.CROP_COLMAJOR: bool = False