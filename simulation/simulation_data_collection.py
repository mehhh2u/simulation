#!/usr/bin/env python3

import sys
import os
import time
import math
import numpy as np

import pandas as pd

from std_msgs.msg import Header
#from gazebo_msgs.msg import *
#from gazebo_msrs.srv import *
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

import tf_transformations as transformations

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

## Constant definitions for training/evaluation map simulations
# In m, heightmaps (in simulation) are assumed 'squared' so min_hm_x is equal to -max_hm_x
max_hm_x = 64.5
max_hm_y = 64.5
# Max map height, 0 is the min
max_hm_z = 10
# In pixels, heightmaps (images) are assumed squared, gazebo requires sizes 2^n+1 x 2^n+1
im_hm_x = 513
im_hm_y = 513

# Adjust output folder accordingly
output_folder = "/home/charlie/traversability_analysis_ws_ros2/src/simulation/csv/"

## For gazebo communication
model_name = "husky"
root_relative_entity_name = '' # This is the full model and not only the base_link
fixed_twist = Twist()
fixed_twist.linear.x = 0.15 # Moving forward at .15m/s
fixed_twist.linear.y = 0.0
fixed_twist.linear.z = 0.0
fixed_twist.angular.x = 0.0
fixed_twist.angular.y = 0.0
fixed_twist.angular.z = 0.0

sim_duration = Duration(seconds = 2)

def generate_random_pose():
    # x,y (z will be fixed as the max_hm_z so that the robot will drop down), gamma as orientation
    rn = np.random.random_sample((3,))
    random_pose = Pose()
    random_pose.position.x = 2 * max_hm_x * rn[0] - max_hm_x
    random_pose.position.y = 2 * max_hm_y * rn[1] - max_hm_y
    random_pose.position.z = max_hm_z * 0.5 # Not spawning at max height z to reduce overtuning chances
    qto = transformations.quaternion_from_euler(0, 0, 2*math.pi * rn[1], axes = 'sxyz')
    random_pose.orientation.x = qto[0]
    random_pose.orientation.y = qto[1]
    random_pose.orientation.z = qto[2]
    random_pose.orientation.w = qto[3]    
    return random_pose

def generate_random_model_state():
    random_pose = generate_random_pose()
    model_state = EntityState()
    model_state.name = model_name
    model_state.pose = random_pose
    model_state.twist = fixed_twist
    model_state.reference_frame = root_relative_entity_name
    return model_state


def get_model_state(node, model_name, root_relative_entity_name):
    client = node.create_client(GetEntityState, '/gazebo/get_entity_state')

    if not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().error('Service /gazebo/get_entity_state not available.')
        return False

    try:
        request = GetEntityState.Request()
        request.name = model_name
        request.reference_frame = root_relative_entity_name

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        return future.result()
    
    except Exception as e:
        node.get_logger().error('Service call failed: %s' % str(e))
        return False
    
def set_model_state(node, model_state):
    client = node.create_client(SetEntityState, '/gazebo/set_entity_state')
    
    if not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().error('Service /gazebo/set_entity_state not available.')
        return False
    
    try:
        request = SetEntityState.Request()
        request.state = model_state

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        return future.result()
    
    except Exception as e:
        node.get_logger().error('Service call failed: %s' % str(e))
        return False
        
def is_pose_in_map(pose):
    little_offset = 0.1 # to avoid waiting until the robot is really on the edge
    if pose.position.x > max_hm_x - little_offset or pose.position.x < -max_hm_x + little_offset:
        return False
    if pose.position.y > max_hm_y - little_offset or pose.position.y < -max_hm_y + little_offset:
        return False
    if pose.position.z < 0.1: # Spawning husky in empty world, the z position pose is z=0.13227300333847308, adjusted accordingly, test for more verification
        return False
    return True

def toScreenFrame (s_x, s_y, s_z):
    # from simulation frame (gazebo) x right, y up, z out of the screen, center is at the middle of the map
    # to x right , y down, ignoring ; xs,ys need to be multiplied by the image size
    xs = s_x + max_hm_x
    ys = -s_y + max_hm_y
    xs = xs / (max_hm_x - (-max_hm_x))
    ys = ys / (max_hm_y - (-max_hm_y))
    return xs, ys

#def usage():
#    return "%s [world_name] [number_of_tries]"%sys.argv[0]

## Class to manage the publishers and subscribers to send and recover data from simulation using rostopics instead of services
class PubsSubsManager(Node):
    def __init__(self):
        super(PubsSubsManager, self).__init__('pubs_subs_manager') 
        self.send_cmd_vel = False
        self.process_odom = False
        self.odom_topic = '/odom'
        self.cmd_vel_topic = '/husky_velocity_controller/cmd_vel_unstamped'
        self.subscription = self.create_subscription(Odometry, self.odom_topic, self.callback_odom, 10)
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.world_name = self.declare_parameter('world_name', 'default_world').get_parameter_value().string_value
        self.dataset_type = self.declare_parameter('dataset_type', 'training').get_parameter_value().string_value
        self.i_sim = self.declare_parameter('number_tries', 10).get_parameter_value().integer_value
        self.starting_time = self.get_clock().now()
        self.initial_pose = None
        self.data_buffer = 0
        
    def set_flag_cmd_vel(self, val):
        self.send_cmd_vel = val
    
    def set_flag_odom(self, val):
        self.process_odom = val
    
    def get_flag_cmd_vel(self):
        return self.send_cmd_vel

    def get_flag_odom(self):
        return self.process_odom
    
    def publish_cmd_vel(self, msg):
        if self.send_cmd_vel:
            self.publisher.publish(msg)

    # Here we read the odom and store it in a csv file
    def callback_odom(self, msg):
        current_time = self.get_clock().now()
        if not is_pose_in_map(msg.pose.pose):
            self.process_odom = False

        if self.process_odom:
            orientation_euler = transformations.euler_from_quaternion([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])
            ii_position = toScreenFrame(self.initial_pose.position.x, self.initial_pose.position.y, self.initial_pose.position.z)
            ic_position = toScreenFrame(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

            # Create a new row as a list
            new_row = [
                current_time,
                ii_position[0] * im_hm_x,
                ii_position[1] * im_hm_y,
                self.initial_pose.position.x,
                self.initial_pose.position.y,
                self.initial_pose.position.z,
                ic_position[0] * im_hm_x,
                ic_position[1] * im_hm_y,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                orientation_euler[0],
                orientation_euler[1],
                orientation_euler[2],
                0,  # Assuming this is a placeholder for a future value
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ]

            # Insert the new row at the end of the DataFrame
            self.data_buffer.loc[len(self.data_buffer)] = new_row
        else:
            self.process_odom = False

    def callback_odom(self, msg):
        self.get_logger().info("callback_odom called")
        current_time = self.get_clock().now()
        if not is_pose_in_map(msg.pose.pose):
            self.get_logger().info("Pose is out of map")
            self.process_odom = False

        if self.process_odom:
            self.get_logger().info("Processing odom data")
            orientation_euler = transformations.euler_from_quaternion([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])
            ii_position = toScreenFrame(self.initial_pose.position.x, self.initial_pose.position.y, self.initial_pose.position.z)
            ic_position = toScreenFrame(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            new_row = [
                current_time,
                ii_position[0] * im_hm_x,
                ii_position[1] * im_hm_y,
                self.initial_pose.position.x,
                self.initial_pose.position.y,
                self.initial_pose.position.z,
                ic_position[0] * im_hm_x,
                ic_position[1] * im_hm_y,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                orientation_euler[0],
                orientation_euler[1],
                orientation_euler[2],
                0,  # Assuming this is a placeholder for a future value
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ]
            self.data_buffer.loc[len(self.data_buffer)] = new_row
            self.get_logger().info(f"New data added: {new_row}")
        else:
            self.get_logger().info("Process odom is false")
        
    def start_new_simulation_try(self, world_name, dataset_type, i_sim, initial_pose):
        self.get_logger().info(f"Starting new simulation try: {world_name}, {dataset_type}, {i_sim}")
        self.world_name = world_name
        self.dataset_type = dataset_type
        self.i_sim = i_sim
        self.initial_pose = initial_pose
        # I_RIP imageframe robot initial position, S_ simulation frame, RCP robot current position, TLV twist linear vel, S_RCO_A simulation robot current orientation alpha/beta/gamma (orientation is given in euler angles
        columns = [
            "TIMESTAMP",
            "I_RIP_X",
            "I_RIP_Y",
            "S_RIP_X",
            "S_RIP_Y",
            "S_RIP_Z",
            "I_RCP_X",
            "I_RCP_Y",
            "S_RCP_X",
            "S_RCP_Y",
            "S_RCP_Z",
            "S_RCO_A",
            "S_RCO_B",
            "S_RCO_G",
            "S_RC_STEER",
            "S_RC_TLV_X",
            "S_RC_TLV_Y",
            "S_RC_TLV_Z",
            "S_RC_TAV_X",
            "S_RC_TAV_Y",
            "S_RC_TAV_Z"
        ]        
        self.data_buffer = pd.DataFrame(columns = columns)
        self.starting_time = self.get_clock().now()
        self.set_flag_cmd_vel(True)
        self.publish_cmd_vel(fixed_twist)
        self.set_flag_odom(True)
        
    # save the buffer to a csv and stop the try simulation    
    def stop_simulation_try(self):
        if not isinstance(self.data_buffer, pd.DataFrame):
            self.get_logger().error("Data buffer is not initialized")
            return None
        
        # Ensure output folder exists or create it
        os.makedirs(output_folder, exist_ok=True)
        
        file_name = f"{self.world_name}_{self.dataset_type}_{str(self.i_sim)}.csv"
        file_path = os.path.join(output_folder, file_name)
        
        self.data_buffer.to_csv(file_path)
        self.get_logger().info(f"Saving simulation data to {file_path}")
        
        return file_path
        
def main(args=None):
    rclpy.init(args=args)
    node = Node("run_simulation_manager")

#    if len(sys.argv) >= 4:
#        world_name = sys.argv[1]
#        dataset_type = sys.argv[2]
#        number_tries = int(sys.argv[3])
#    else:
#        node.get_logger().error("Usage: ros2 run [package_name] [node_name] [world_name] [dataset_type] [number_tries]")
#        sys.exit(1)

    # Declare and get parameters
    world_name = node.declare_parameter('world_name', 'default_world').get_parameter_value().string_value
    dataset_type = node.declare_parameter('dataset_type', 'training').get_parameter_value().string_value
    number_tries = node.declare_parameter('number_tries', 10).get_parameter_value().integer_value

    # Initialize PubsSubsManager
    pubssubs_manager = PubsSubsManager()
    
    # Wait for Gazebo and the robot model to be up
    while not get_model_state(node, model_name, root_relative_entity_name).success:
        node.get_logger().info(f"Waiting for {model_name} model to be up in Gazebo")
        rclpy.spin_once(node, timeout_sec=1.0)
        node.get_logger().info(f"saved")

    meta_csv_buffer = pd.DataFrame(columns=["CSV", "HEIGHTMAP"])

    for i_sim in range(number_tries):
        node.get_logger().info(f"=== Simulation {i_sim}/{number_tries} for map {world_name}")
        random_model_state = generate_random_model_state()
        node.get_logger().info(f"-- Spawning robot at {random_model_state.pose}")
        res = set_model_state(node, random_model_state)
        node.get_logger().info(f"-- {res}")

        rclpy.spin_once(node, timeout_sec=0.5)
        pubssubs_manager.start_new_simulation_try(world_name, dataset_type, i_sim, random_model_state.pose)
        start_t = node.get_clock().now()

        while pubssubs_manager.get_flag_odom():
            current_t = node.get_clock().now()
            if (current_t - start_t).nanoseconds / 1e9 >= sim_duration.nanoseconds / 1e9:
                pubssubs_manager.set_flag_odom(False)

        csv_file_name = pubssubs_manager.stop_simulation_try()
        meta_csv_buffer.loc[len(meta_csv_buffer)] = [csv_file_name, world_name + ".png"]

    meta_csv_buffer.to_csv(f"{output_folder}/meta_{world_name}_{dataset_type}.csv")
    node.destroy_node()

if __name__ == '__main__':
    main()