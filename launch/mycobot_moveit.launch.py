import os
import yaml
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.joint(package_path, file_path)
    
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None
    