import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# Same accepted arm_type spellings as demo.launch.py
VALID_ARM_TYPES = {
    "v1.0", "v10", "v1_0", "openarm_v1.0", "openarm_v10", "openarm_v1_0",
    "v2.0", "v20", "v2_0", "openarm_v2.0", "openarm_v20", "openarm_v2_0",
}


def resolve_arm_config(arm_type_str: str) -> tuple[str, str]:
    if arm_type_str not in VALID_ARM_TYPES:
        raise ValueError(f"Invalid arm_type: '{arm_type_str}'")
    if any(x in arm_type_str for x in ("1.0", "10", "1_0")):
        return "openarm_v1.0", "openarm_v10.urdf.xacro"
    return "openarm_v2.0", "openarm_v20.urdf.xacro"


def mtc_node_spawner(context: LaunchContext, description_package, arm_type, use_fake_hardware):
    description_package_str = context.perform_substitution(description_package)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)

    description_pkg_path = get_package_share_directory(description_package_str)
    moveit_pkg_path = get_package_share_directory("openarm_bimanual_moveit_config")

    config_dir, xacro_file = resolve_arm_config(arm_type_str)
    xacro_path = os.path.join(description_pkg_path, "assets", "robot", config_dir, "urdf", xacro_file)

    # Identical construction to moveit_nodes_spawner() in your demo.launch.py --
    # the MTC node needs the exact same robot_description/semantic/kinematics
    # parameters move_group is using, not a simplified guess at them.
    moveit_config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={
                "arm_type": arm_type_str,
                "bimanual": "true",
                "use_fake_hardware": use_fake_hardware_str,
                "ros2_control": "true",
            },
        )
        .robot_description_semantic(file_path=f"config/{config_dir}/openarm_bimanual.srdf")
        .robot_description_kinematics(file_path=f"config/{config_dir}/kinematics.yaml")
        .joint_limits(file_path=f"config/{config_dir}/joint_limits.yaml")
        .trajectory_execution(file_path=f"config/{config_dir}/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()

    # Same Pilz cartesian limits merge as demo.launch.py, in case any stage ends up using Pilz
    pilz_path = os.path.join(moveit_pkg_path, "config", config_dir, "pilz_cartesian_limits.yaml")
    if os.path.exists(pilz_path):
        import yaml
        with open(pilz_path, "r") as f:
            config_data = yaml.safe_load(f)
            if "cartesian_limits" in config_data:
                moveit_params.setdefault("robot_description_planning", {}).update(config_data)

    return [
        Node(
            package="openarm_mtc_pick_place",
            executable="mtc_node_air_pickplace",
            output="screen",
            parameters=[moveit_params],
        )
    ]


def generate_launch_description():
    description_package = LaunchConfiguration("description_package")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    return LaunchDescription([
        DeclareLaunchArgument("description_package", default_value="openarm_description"),
        DeclareLaunchArgument("arm_type", default_value="openarm_v1.0"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        OpaqueFunction(
            function=mtc_node_spawner,
            args=[description_package, arm_type, use_fake_hardware],
        ),
    ])
