import yaml
from ament_index_python.packages import get_package_share_directory

class ConfigBase:

    def __init__(self, package_name: str = ""):

        self.instance_defaults()

        base_directory = get_package_share_directory(package_name)
        
        with open(f"{base_directory}/config/config.yaml", 'r') as f:
            config = yaml.safe_load(f)
        
        if not config is None:
            for k,v in config.items():
                if hasattr(self, k):
                    setattr(self, k , v)
                else:
                    raise f"Config has no attribute {k}"
    
    def instance_defaults():
        raise NotImplementedError("ConfigBase cannot be used directly, need to set defaults")