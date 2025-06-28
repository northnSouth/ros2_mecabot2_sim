import os 
import xacro 
import math
from launch import LaunchDescription, SomeEntitiesType
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory 
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

	package_name = 'ros2_mecabot2_sim_base'

	gazebo_launch_file = os.path.join(
        get_package_share_directory('ros_gz_sim'), 
        'launch/gz_sim.launch.py'
    )

	gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'gz_args': 'empty.sdf',
            'on_exit_shutdown': 'true'
        }.items()
    )

	gazebo_unpause = ExecuteProcess(
      cmd=[[
        'ros2',
        ' service call ',
        ' /world/empty/control ',
        ' ros_gz_interfaces/srv/ControlWorld ',
        ' "{world_control: {pause: false}}" '
      ]],
      name='unpause_gazebo',
      shell=True
    )

	ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
        '/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/world/empty/control@ros_gz_interfaces/srv/ControlWorld'
      ],
      remappings=[('/world/empty/clock', '/clock')]
    )

	robot_desc = xacro.process_file(
    	os.path.join(get_package_share_directory(package_name), 'robot_desc/mecabot2_assemble.xacro')
    	).toprettyxml(indent='  ')

	robot_state_publisher = Node(
    	package='robot_state_publisher',
    	executable='robot_state_publisher',
    	parameters=[{
     		'use_sim_time': True, 
     		'robot_description': robot_desc
    	}]
  	)

	control_activate = Node(
  	  	package="controller_manager",
  	  	executable="spawner",
  	  	arguments=
  	  	[
  	    	"JSB",
  	    	"velo_c"
  	  	]
  	)

	spawn_robot = Node(
    	package='ros_gz_sim',
    	executable='create',
    	parameters=[{
    		'name': package_name,
    		'world': 'empty',
    		'topic': '/robot_description',
    		'x': 0.0,
    		'y': 0.0,
    		'z': 0.2,
    		'Y': 0.0
    	}]
  	)

	rviz_config_file = os.path.join(
    	get_package_share_directory(package_name), 
    	'rviz2', 'main_view.rviz'
    )

	rviz2_run = Node(
    	package='rviz2',
    	executable='rviz2',
    	output='screen',
    	arguments=['-d', rviz_config_file],
    	parameters=[{ 'use_sim_time': True }]
    )

	return LaunchDescription([
    	gazebo_launch, 
    	ros_gz_bridge, 
    	robot_state_publisher,
		gazebo_unpause,

    	RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_unpause,
                on_exit=[spawn_robot, control_activate]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=control_activate,
                on_exit=rviz2_run
            )
        )
  	])