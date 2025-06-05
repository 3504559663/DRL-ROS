import os
from ament_index_python.packages import get_package_share_directory # We will use this directly
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare # We will avoid this for a moment

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ.get("TURTLEBOT3_MODEL", "burger") 
    use_sim_time_cfg = LaunchConfiguration("use_sim_time", default="true")
    xacro_file_name_in_project = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    
    urdf_xacro_path_str = "COULD_NOT_FIND_PKG_PATH" # Default to a bad path
    try:
        # Get the path string directly at parse time
        description_package_actual_path = get_package_share_directory('turtlebot3_description')
        urdf_xacro_path_str = os.path.join(
            description_package_actual_path, # This is already .../share/turtlebot3_description
            'urdf',
            xacro_file_name_in_project
        )
        print(f"DEBUG: Using direct path for URDF: {urdf_xacro_path_str}")
    except Exception as e:
        print(f"DEBUG: get_package_share_directory('turtlebot3_description') FAILED at parse time: {e}")
        # The launch will likely fail later if xacro can't find the file, 
        # or robot_state_publisher will complain about empty robot_description.

    # robot_description_content still needs to be a Substitution for Command
    # Command expects its arguments to be Substitutions or strings that can be converted.
    # Here, urdf_xacro_path_str is a Python string, which Command can handle.
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), # This is a Substitution
        ' ',
        urdf_xacro_path_str # This is now a concrete Python string
    ])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time_cfg},
                    {"robot_description": robot_description_content}
                ],
            ),
        ]
    )