import time
import rclpy
from ros_nodes import (
    OdomSubscriber,
    ResetWorldClient,
    CmdVelPublisher,
    MarkerPublisher,
    PhysicsClient,
    SensorSubscriber,
    SlamHandler
)
import numpy as np
from geometry_msgs.msg import Pose, Twist
from squaternion import Quaternion
from std_srvs.srv import Empty
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory
from colorama import Fore, Style
from collections import deque
import random

class ROS_env:
    def __init__(
        self,
        init_target_distance=2.0,
        target_dist_increase=0.001,
        max_target_dist=8.0,
        target_reached_delta=0.5,
        collision_delta=0.4,
        args=None,
    ):
        rclpy.init(args=args)
        self.cmd_vel_publisher = CmdVelPublisher()
        # self.scan_subscriber = ScanSubscriber()
        # self.odom_subscriber = OdomSubscriber()
        # self.robot_state_publisher = SetModelStateClient()
        self.world_reset = ResetWorldClient()
        self.physics_client = PhysicsClient()
        self.publish_target = MarkerPublisher()
        self.element_positions = [
            [-2.93, 3.17],
            [2.86, -3.0],
            [-2.77, -0.96],
            [2.83, 2.93],
        ]
        self.sensor_subscriber = SensorSubscriber()
        self.target_dist = init_target_distance
        self.target_dist_increase = target_dist_increase
        self.max_target_dist = max_target_dist
        self.target_reached_delta = target_reached_delta
        self.collision_delta = collision_delta
        self.slam_handler = SlamHandler()
        self.target = [0.0, 0.0]

    def terminate(self):
        self.slam_handler.stop()

    def step(self, lin_velocity=0.0, ang_velocity=0.1):
        self.cmd_vel_publisher.publish_cmd_vel(lin_velocity, ang_velocity)
        self.physics_client.unpause_physics()
        time.sleep(0.1)
        rclpy.spin_once(self.sensor_subscriber)
        self.physics_client.pause_physics()

        (
            latest_scan,
            latest_position,
            latest_orientation,
        ) = self.sensor_subscriber.get_latest_sensor()

        distance, cos, sin, _ = self.get_dist_sincos(
            latest_position, latest_orientation
        )
        collision = self.check_collision(latest_scan)
        goal = self.check_target(distance, collision)
        action = [lin_velocity, ang_velocity]
        reward = self.get_reward(goal, collision, action, latest_scan)

        return latest_scan, distance, cos, sin, collision, goal, action, reward

    def reset(self):
        self.world_reset.reset_world()
        action = [0.0, 0.0]
        self.cmd_vel_publisher.publish_cmd_vel(
            linear_velocity=action[0], angular_velocity=action[1]
        )

        self.element_positions = [
            [-2.93, 3.17],
            [2.86, -3.0],
            [-2.77, -0.96],
            [2.83, 2.93],
        ]
        

        self.slam_handler.stop()
        self.slam_handler.start()           
        time.sleep(3)

        for i in range(10):
            self.physics_client.unpause_physics()
            rclpy.spin_once(self.sensor_subscriber)
            time.sleep(0.2)
            self.physics_client.pause_physics()

        for i in range(1):
            latest_scan, distance, cos, sin, _, _, action, reward = self.step(
                lin_velocity=action[0], ang_velocity=action[1]
            )

        if self.ready():
            self.target = random.choice(self.sensor_subscriber.frontiers)
        
        self.publish_target.publish(self.target[0], self.target[1])

        print(self.target)

        return latest_scan, distance, cos, sin, False, False, action, reward

    def eval(self, scenario):
        self.cmd_vel_publisher.publish_cmd_vel(0.0, 0.0)

        self.target = [scenario[-1].x, scenario[-1].y]
        self.publish_target.publish(self.target[0], self.target[1])

        self.physics_client.unpause_physics()
        time.sleep(1)
        latest_scan, distance, cos, sin, _, _, a, reward = self.step(
            lin_velocity=0.0, ang_velocity=0.0
        )
        return latest_scan, distance, cos, sin, False, False, a, reward

    def set_random_position(self, name):
        angle = np.random.uniform(-np.pi, np.pi)
        pos = False
        while not pos:
            x = np.random.uniform(-4.0, 4.0)
            y = np.random.uniform(-4.0, 4.0)
            pos = self.check_position(x, y, 1.8)
        self.element_positions.append([x, y])
        self.set_position(name, x, y, angle)

    def set_robot_position(self):
        angle = np.random.uniform(-np.pi, np.pi)
        pos = False
        while not pos:
            x = np.random.uniform(-4.0, 4.0)
            y = np.random.uniform(-4.0, 4.0)
            pos = self.check_position(x, y, 1.8)
        self.set_position("turtlebot3_waffle", x, y, angle)
        return x, y

    def set_position(self, name, x, y, angle):
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation.x = quaternion.x
        pose.orientation.y = quaternion.y
        pose.orientation.z = quaternion.z
        pose.orientation.w = quaternion.w

        self.robot_state_publisher.set_state(name, pose)
        rclpy.spin_once(self.robot_state_publisher)

    def check_position(self, x, y, min_dist):
        pos = True
        for element in self.element_positions:
            distance_vector = [element[0] - x, element[1] - y]
            distance = np.linalg.norm(distance_vector)
            if distance < min_dist:
                pos = False
        return pos

    def check_collision(self, laser_scan):
        if min(laser_scan) < self.collision_delta:
            return True
        return False

    def check_target(self, distance, collision):
        if distance < self.target_reached_delta and not collision:
            self.target_dist += self.target_dist_increase
            if self.target_dist > self.max_target_dist:
                self.target_dist = self.max_target_dist
            return True
        return False

    def get_dist_sincos(self, odom_position, odom_orientation):
        # Calculate robot heading from odometry data
        odom_x = odom_position.x
        odom_y = odom_position.y
        quaternion = Quaternion(
            odom_orientation.w,
            odom_orientation.x,
            odom_orientation.y,
            odom_orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)
        pose_vector = [np.cos(angle), np.sin(angle)]
        goal_vector = [self.target[0] - odom_x, self.target[1] - odom_y]

        distance = np.linalg.norm(goal_vector)
        cos, sin = self.cossin(pose_vector, goal_vector)

        return distance, cos, sin, angle

    def get_reward(self, goal, collision, action, laser_scan):

        # def map_scale(map_value_gain):
        #     if map_value_gain>self.max_reward_in_history:
        #         self.max_reward_in_history=map_value_gain

        #     scaled_map_value_gain = self.max_map_exploration_reward*map_value_gain/ self.max_reward_in_history
        #     scaled_map_value_gain = max(0, scaled_map_value_gain)
        #     return scaled_map_value_gain
        
        # def avoid_idle():
        #     # print(self.robot_position_buffer)
        #     std_position_x = np.std(np.array(self.robot_position_buffer)[:,0])
        #     std_position_y = np.std(np.array(self.robot_position_buffer)[:,1])
        #     motion_speed = std_position_x + std_position_y
        #     if motion_speed < self.robot_position_idle_penalty_threshold:
        #         idling_penalty = (self.robot_position_idle_penalty_threshold - motion_speed)*self.max_idling_penalty
        #         # print(f"Idling penalty : {idling_penalty}")
        #         return idling_penalty
        #     else: return 0

        # def sparcity_reward():
        #     # print(self.robot_position_buffer)
        #     n = len(self.robot_trajectory)
        #     # print(n)
        #     current_std_x = np.std(np.array(self.robot_trajectory)[:,0])
        #     current_std_y = np.std(np.array(self.robot_trajectory)[:,1])
            
        #     rate_std_x =  current_std_x*n - self.prev_std_x*(n-1)
        #     rate_std_y =  current_std_y*n- self.prev_std_y*(n-1)

        #     self.prev_std_x = current_std_x
        #     self.prev_std_y = current_std_y
            
        #     scale = 1
        #     # sparcity = (rate_std_x + rate_std_y) * scale
        #     sparcity = (current_std_x + current_std_y) * scale
        #     return sparcity
        if goal:
            print("Goal reached")
            return 100.0
        
        elif collision:
            print("Collision")
            return -100.0
        else:
            r3 = lambda x: 2.5 - x if x < 2.5 else 0.0
            # print(map_scale(map_value_gain))
            # idling_penlaty = avoid_idle()
            # map_reward = map_scale(map_value_gain)
            rotation_penalty = abs(action[1])/2
            # sparcity_reward_ = sparcity_reward()
            collision_penalty = r3(min(laser_scan)) / 2
            totol_reward = action[0] - rotation_penalty - collision_penalty
            # print(math.isnan(totol_reward))
            print(f"Rewards----- Total: {totol_reward:.2f} | Rot Penalty: {rotation_penalty:.2f} | Col Pen : {collision_penalty:.2f}")

            return totol_reward

    def ready(self):
        if self.sensor_subscriber.latest_position==None or len(self.sensor_subscriber.frontiers)==0:
            return False
        else: return True

    @staticmethod
    def cossin(vec1, vec2):
        vec1 = vec1 / np.linalg.norm(vec1)
        vec2 = vec2 / np.linalg.norm(vec2)
        cos = np.dot(vec1, vec2)
        sin = np.cross(vec1, vec2).item()

        return cos, sin
