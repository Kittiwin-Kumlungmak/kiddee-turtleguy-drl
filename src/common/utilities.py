from queue import Empty
from turtlebot3_msgs.srv import DrlStep
from turtlebot3_msgs.srv import Goal
from std_srvs.srv import Empty
import os
import time
import rclpy
import torch
import numpy

import xml.etree.ElementTree as ET

with open('/tmp/drlnav_current_stage.txt', 'r') as f:
    test_stage = int(f.read())

def check_gpu():
    print("gpu torch available: ", torch.cuda.is_available())
    if (torch.cuda.is_available()):
        print("device name: ", torch.cuda.get_device_name(0))
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")

def step(agent_self, action, previous_action):
    req = DrlStep.Request()
    req.action = action
    req.previous_action = previous_action

    while not agent_self.step_comm_client.wait_for_service(timeout_sec=1.0):
        agent_self.get_logger().info('env step service not available, waiting again...')
    future = agent_self.step_comm_client.call_async(req)

    while rclpy.ok():
        rclpy.spin_once(agent_self)
        if future.done():
            if future.result() is not None:
                res = future.result()
                return res.state, res.reward, res.done, res.success, res.distance_traveled
            else:
                agent_self.get_logger().error(
                    'Exception while calling service: {0}'.format(future.exception()))
                print("ERROR getting step service response!")

def init_episode(agent_self):
    state, _, _, _, _ = step(agent_self, [], [0.0, 0.0])
    return state

def get_goal_status(agent_self):
    req = Goal.Request()
    while not agent_self.goal_comm_client.wait_for_service(timeout_sec=1.0):
        agent_self.get_logger().info('new goal service not available, waiting again...')
    future = agent_self.goal_comm_client.call_async(req)

    while rclpy.ok():
        rclpy.spin_once(agent_self)
        if future.done():
            if future.result() is not None:
                res = future.result()
                return res.new_goal
            else:
                agent_self.get_logger().error(
                    'Exception while calling service: {0}'.format(future.exception()))
                print("ERROR getting   service response!")

def wait_new_goal(agent_self):
    while(get_goal_status(agent_self) == False):
        print("Waiting for new goal...")
        time.sleep(1.0)

def pause_simulation(agent_self):
    while not agent_self.gazebo_pause.wait_for_service(timeout_sec=1.0):
        agent_self.get_logger().info('pause gazebo service not available, waiting again...')
    future = agent_self.gazebo_pause.call_async(Empty.Request())
    while rclpy.ok():
        rclpy.spin_once(agent_self)
        if future.done():
            return

def unpause_simulation(agent_self):
    while not agent_self.gazebo_unpause.wait_for_service(timeout_sec=1.0):
        agent_self.get_logger().info('unpause gazebo service not available, waiting again...')
    future = agent_self.gazebo_unpause.call_async(Empty.Request())
    while rclpy.ok():
        rclpy.spin_once(agent_self)
        if future.done():
            return

def translate_outcome(outcome):
    if outcome == 1:
        return "SUCCESS"
    elif outcome == 2:
        return "COLLISION_WALL"
    elif outcome == 3:
        return "COLLISION_OBSTACLE"
    elif outcome == 4:
        return "TIMEOUT"
    elif outcome == 5:
        return "TUMBLE"
    else:
        return f"UNKNOWN: {outcome}"

# --- Environment ---

def euler_from_quaternion(quat):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quat = [x, y, z, w]
    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    sinr_cosp = 2 * (w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = numpy.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w*y - z*x)
    if sinp < -1:
        sinp = -1
    if sinp > 1:
        sinp = 1
    pitch = numpy.arcsin(sinp)

    siny_cosp = 2 * (w*z + x*y)
    cosy_cosp = 1 - 2 * (y*y + z*z)
    yaw = numpy.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def get_scan_count():
    tree = ET.parse(os.getenv('DRLNAV_BASE_PATH') + '/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf')

    root = tree.getroot()
    for link in root.find('model').findall('link'):
        if link.get('name') == 'base_scan':
            return int(link.find('sensor').find('ray').find('scan').find('horizontal').find('samples').text)

def get_simulation_speed(stage):
    tree = ET.parse(os.getenv('DRLNAV_BASE_PATH') + '/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_drl_stage' + str(stage) + '/burger.model')
    root = tree.getroot()
    return int(root.find('world').find('physics').find('real_time_factor').text)
    
    

import numpy as np
from scipy import signal as s
def ray_resam(lidar):
    x = s.resample_poly(np.array(lidar),4,45)
    return x

def ray_renorm(lidar_measurements):
    # 450 LiDAR measurements, with angles in degrees and distances in meters
    # Define the desired number of output rays
    num_output_rays = 40
    hutty = [0, 11, 22, 33, 45, 56, 67, 78, 90, 101, 112, 123, 135, 146, 157, 168, 180, 191, 202, 213, 225, 236, 247, 258, 270 ,281, 292, 303, 315, 326, 337, 348, 360, 
    	371, 382, 393, 405, 416, 427, 438]
    # Create a list to store the downsampled rays
    downsampled_rays = []
    len_dar = len(lidar_measurements)
    # Iterate over the desired number of output rays
    for i in hutty:
        # hut = ceil(len_dar/num_output_rays)%len_dar
        # Add the closest measurement to the downsampled rays list
        downsampled_rays.append(lidar_measurements[i])

    # Convert the downsampled rays list to a numpy array
    downsampled_rays = np.array(downsampled_rays)
    #print(len(downsampled_rays))
    return downsampled_rays

