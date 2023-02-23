import math
import numpy
import sys
import copy
import os
import time
import numpy as np
from numpy.core.numeric import Infinity
from geometry_msgs.msg import Pose, Twist
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from turtlebot3_msgs.srv import DrlStep, Goal, RingGoal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from turtlebot3_msgs.srv import DrlStep, Goal
import rclpy
from rclpy.node import Node

from common import utilities as util
from common.settings import ENABLE_BACKWARD, EPISODE_TIMEOUT_SECONDS, ENABLE_MOTOR_NOISE
from common import reward as rw
from common.settings import ENABLE_VISUAL, ENABLE_STACKING, OBSERVE_STEPS, MODEL_STORE_INTERVAL
from common.storagemanager import StorageManager
from common.graph import Graph
from common.logger import Logger
if ENABLE_VISUAL:
    from common.visual import DrlVisual
from common import utilities as util
#from common.dqn import DQN
from common.ddpg import DDPG
from common.td3 import TD3
from common.reward import UNKNOWN, SUCCESS, COLLISION_WALL, COLLISION_OBSTACLE, TIMEOUT, TUMBLE
from common.replaybuffer import ReplayBuffer



#-------------------------------------------------------------#
class DrlAgent(Node):
    def __init__(self, algorithm, training, load_session="", load_episode=0, train_stage=util.test_stage):
        super().__init__(algorithm + '_agent')
        self.algorithm = algorithm
        self.is_training = int(training)
        self.load_session = load_session
        self.episode = int(load_episode)
        self.train_stage = train_stage
        if (not self.is_training and not self.load_session):
            quit("ERROR no test agent specified")
        self.device = util.check_gpu()
        self.sim_speed = util.get_simulation_speed(self.train_stage)
        print(f"{'training' if (self.is_training) else 'testing' } on stage: {util.test_stage}")

        self.total_steps = 0
        self.observe_steps = OBSERVE_STEPS

        if self.algorithm == 'dqn':
            self.model = DQN(self.device, self.sim_speed)
        elif self.algorithm == 'ddpg':
            self.model = DDPG(self.device, self.sim_speed)
        elif self.algorithm == 'td3':
            self.model = TD3(self.device, self.sim_speed)
        else:
            quit(f"invalid algorithm specified: {self.algorithm}, chose one of: ddpg, td3, td3conv")

        self.replay_buffer = ReplayBuffer(self.model.buffer_size)
        self.graph = Graph()

        # ===================================================================== #
        #                             Model loading                             #
        # ===================================================================== #

        self.sm = StorageManager(self.algorithm, self.train_stage, self.load_session, self.episode, self.device)

        if self.load_session:
            del self.model
            self.model = self.sm.load_model()
            self.model.device = self.device
            self.sm.load_weights(self.model.networks)
            if self.is_training:
                self.replay_buffer.buffer = self.sm.load_replay_buffer(self.model.buffer_size, os.path.join(self.load_session, 'stage'+str(self.train_stage)+'_latest_buffer.pkl'))
            self.total_steps = self.graph.set_graphdata(self.sm.load_graphdata(), self.episode)
            print(f"global steps: {self.total_steps}")
            print(f"loaded model {self.load_session} (eps {self.episode}): {self.model.get_model_parameters()}")
        else:
            self.sm.new_session_dir()
            self.sm.store_model(self.model)

        self.graph.session_dir = self.sm.session_dir
        self.logger = Logger(self.is_training, self.sm.machine_dir, self.sm.session_dir, self.sm.session, self.model.get_model_parameters(), self.model.get_model_configuration(), str(util.test_stage), self.algorithm, self.episode)
        if ENABLE_VISUAL:
            self.visual = DrlVisual(self. model.state_size, self.model.hidden_size)
            self.model.attach_visual(self.visual)
        # ===================================================================== #
        #                             Start Process                             #
        # ===================================================================== #

        self.step_comm_client = self.create_client(DrlStep, 'step_comm')
        self.goal_comm_client = self.create_client(Goal, 'goal_comm')
        self.process()


    def process(self):

        while (True):
            episode_done = False
            step, reward_sum, loss_critic, loss_actor = 0, 0, 0, 0
            action_past = [0.0, 0.0]
            state = util.init_episode(self)

            if ENABLE_STACKING:
                frame_buffer = [0.0] * (self.model.state_size * self.model.stack_depth * self.model.frame_skip)
                state = [0.0] * (self.model.state_size * (self.model.stack_depth - 1)) + list(state)
                next_state = [0.0] * (self.model.state_size * self.model.stack_depth)

            time.sleep(0.5)
            episode_start = time.perf_counter()

            while not episode_done:
                if self.is_training and self.total_steps < self.observe_steps:
                    action = self.model.get_action_random()
                else:
                    action = self.model.get_action(state, self.is_training, step, ENABLE_VISUAL)

                action_current = action
                if self.algorithm == 'dqn':
                    action_current = self.model.possible_actions[action]

                # Take a step
                next_state, reward, episode_done, outcome, distance_traveled = util.step(self, action_current, action_past)
                action_past = copy.deepcopy(action_current)
                reward_sum += reward

                if ENABLE_STACKING:
                    frame_buffer = frame_buffer[self.model.state_size:] + list(next_state)      # Update big buffer with single step
                    next_state = []                                                         # Prepare next set of frames (state)
                    for depth in range(self.model.stack_depth):
                        start = self.model.state_size * (self.model.frame_skip - 1) + (self.model.state_size * self.model.frame_skip * depth)
                        next_state += frame_buffer[start : start + self.model.state_size]

                # Train
                if self.is_training == True:
                    self.replay_buffer.add_sample(state, action, [reward], next_state, [episode_done])
                    if self.replay_buffer.get_length() >= self.model.batch_size:
                        loss_c, loss_a, = self.model._train(self.replay_buffer)
                        loss_critic += loss_c
                        loss_actor += loss_a

                if ENABLE_VISUAL:
                    self.visual.update_reward(reward_sum)
                state = copy.deepcopy(next_state)
                step += 1
                time.sleep(self.model.step_time)

            # Episode done
            self.total_steps += step
            duration = time.perf_counter() - episode_start

            if self.total_steps >= self.observe_steps:
                self.episode += 1
                self.finish_episode(step, duration, outcome, distance_traveled, reward_sum, loss_critic, loss_actor)
            else:
                print(f"Observe steps completed: {self.total_steps}/{self.observe_steps}")

    def finish_episode(self, step, eps_duration, outcome, dist_traveled, reward_sum, loss_critic, lost_actor):
            print(f"Epi: {self.episode} R: {reward_sum:.2f} outcome: {util.translate_outcome(outcome)} \
                    steps: {step} steps_total: {self.total_steps}, time: {eps_duration:.2f}")
            if (self.is_training):
                self.graph.update_data(step, self.total_steps, outcome, reward_sum, loss_critic, lost_actor)
                self.logger.file_log.write(f"{self.episode}, {reward_sum}, {outcome}, {eps_duration}, {step}, {self.total_steps}, \
                                                {self.replay_buffer.get_length()}, {loss_critic / step}, {lost_actor / step}\n")

                if (self.episode % MODEL_STORE_INTERVAL == 0) or (self.episode == 1):
                    self.graph.draw_plots(self.episode)
                    self.sm.save_session(self.episode, self.model.networks, self.graph.graphdata, self.replay_buffer.buffer)
                    self.logger.update_comparison_file(self.episode, self.graph.get_success_count(), self.graph.get_reward_average())
            else:
                self.logger.update_test_results(step, outcome, dist_traveled, eps_duration, 0)
                util.wait_new_goal(self)

def main(args=sys.argv[1:]):
    rclpy.init(args=args)
    drl_agent = DrlAgent(*args)
    rclpy.spin(drl_agent)
    drl_agent.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
