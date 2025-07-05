from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dual_dobot")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/nova5_robot.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    ).to_dict()

    # Dual Robot Pick and Place Demo
    # This node will control both Dobot robots to perform a pick and place task.
    # It assumes that the MoveIt configuration is set up correctly for both robots.
    # The robots will be controlled in a synchronized manner to perform the task.
    pick_place_demo = Node(
        package="dual_dobot_control",
        executable="dual_robot_dobot_control",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])