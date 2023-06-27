import os
from ament_index_python.packages import get_package_share_directory

def get_paths():
    package_share_dir = get_package_share_directory('scara_robot')
    model_path = os.path.join(package_share_dir, 'models')
    plugin_path = os.path.join(package_share_dir, 'plugins')
    media_path = os.path.join(package_share_dir, 'media')

    return model_path, plugin_path, media_path

