Run structure is as follows:
ros2 launch husky_gazebo custom_world.launch.py world_name:='world_name.world'
ros2 launch simulation simulation_data_collection.launch.py 'world_name:=world' 'dataset_type:=training' 'number_tries:=5

Using a teleop_keyboard program works with the gazebo world file. I may have imported the controls over incorrectly, I used the code from the example shown in the other github: https://github.com/romarcg/traversability_estimation
I tried to convert the ROS1 format to the ROS2 format as much as I could. The file would run, but there is an issue with publishing the twist command to /husky_velocity_controller/cmd_vel_unstamped and issue with retrieving data from /odom.
When I echoed /odom, I received data, when I directly published to /husky_velocity_controller/cmd_vel_unstamped, the robot moved in gazebo.

I was planning to redo everything from **PubsSubsManager** onwards as I think that the issue originates from that point onwards.

ros2 services from ros2 service list after launching gazebo world:

/controller_manager/configure_and_start_controller

/controller_manager/configure_controller

/controller_manager/describe_parameters

/controller_manager/get_parameter_types

/controller_manager/get_parameters

/controller_manager/list_controller_types

/controller_manager/list_controllers

/controller_manager/list_hardware_interfaces

/controller_manager/list_parameters

/controller_manager/load_and_configure_controller


/controller_manager/load_and_start_controller

/controller_manager/load_controller

/controller_manager/reload_controller_libraries

/controller_manager/set_parameters

/controller_manager/set_parameters_atomically

/controller_manager/switch_controller

/controller_manager/unload_controller

/delete_entity

/ekf_node/describe_parameters

/ekf_node/get_parameter_types

/ekf_node/get_parameters

/ekf_node/list_parameters

/ekf_node/set_parameters

/ekf_node/set_parameters_atomically

/gazebo/describe_parameters

/gazebo/gazebo_ros_state/describe_parameters

/gazebo/gazebo_ros_state/get_parameter_types

/gazebo/gazebo_ros_state/get_parameters

/gazebo/gazebo_ros_state/list_parameters

/gazebo/gazebo_ros_state/set_parameters

/gazebo/gazebo_ros_state/set_parameters_atomically

/gazebo/get_entity_state

/gazebo/get_parameter_types

/gazebo/get_parameters

/gazebo/list_parameters

/gazebo/set_entity_state

/gazebo/set_parameters

/gazebo/set_parameters_atomically

/gazebo_ros2_control/describe_parameters

/gazebo_ros2_control/get_parameter_types

/gazebo_ros2_control/get_parameters

/gazebo_ros2_control/list_parameters

/gazebo_ros2_control/set_parameters

/gazebo_ros2_control/set_parameters_atomically

/get_model_list

/gps_plugin/describe_parameters

/gps_plugin/get_parameter_types

/gps_plugin/get_parameters

/gps_plugin/list_parameters

/gps_plugin/set_parameters

/gps_plugin/set_parameters_atomically

/husky_velocity_controller/describe_parameters

/husky_velocity_controller/get_parameter_types

/husky_velocity_controller/get_parameters

/husky_velocity_controller/list_parameters

/husky_velocity_controller/set_parameters

/husky_velocity_controller/set_parameters_atomically

/imu_plugin/describe_parameters

/imu_plugin/get_parameter_types

/imu_plugin/get_parameters

/imu_plugin/list_parameters

/imu_plugin/set_parameters

/imu_plugin/set_parameters_atomically

/joint_state_broadcaster/describe_parameters

/joint_state_broadcaster/get_parameter_types

/joint_state_broadcaster/get_parameters

/joint_state_broadcaster/list_parameters

/joint_state_broadcaster/set_parameters

/joint_state_broadcaster/set_parameters_atomically

/pause_physics

/reset_simulation

/reset_world

/robot_state_publisher/describe_parameters

/robot_state_publisher/get_parameter_types

/robot_state_publisher/get_parameters

/robot_state_publisher/list_parameters

/robot_state_publisher/set_parameters

/robot_state_publisher/set_parameters_atomically

/spawn_entity

/twist_mux/describe_parameters

/twist_mux/get_parameter_types

/twist_mux/get_parameters

/twist_mux/list_parameters

/twist_mux/set_parameters

/twist_mux/set_parameters_atomically

/twist_server/get_interactive_markers

/twist_server_node/describe_parameters

/twist_server_node/get_parameter_types

/twist_server_node/get_parameters

/twist_server_node/list_parameters

/twist_server_node/set_parameters

/twist_server_node/set_parameters_atomically

/unpause_physics

**
It says that ekf_node and application marker_services have stopped unexpectedly (from husky side, not sure if it is affecting the simulation, but i doubt it)

ros2 topic list:**

/clock

/cmd_vel

/dynamic_joint_states

/e_stop

/gazebo/link_states

/gazebo/model_states

/gps/data

/husky_velocity_controller/cmd_vel_unstamped

/imu/data_raw

/joint_states

/joy_teleop/cmd_vel

/odom

/parameter_events

/performance_metrics

/robot_description

/rosout

/tf

/tf_static

/twist_marker_server/cmd_vel

/twist_server/update
