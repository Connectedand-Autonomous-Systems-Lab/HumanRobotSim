import time
import rclpy
from ros_nodes import (
    ScanSubscriber,
    OdomSubscriber,
    ResetWorldClient,
    CmdVelPublisher,
    PhysicsClient,
    SensorSubscriber,
    MapSubscriber,
    SlamHandler,
)
import numpy as np
from geometry_msgs.msg import Pose, Twist
from squaternion import Quaternion
from std_srvs.srv import Empty
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory
from colorama import Fore, Style
from collections import deque
import subprocess
import signal
import random
import os

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
        # self.publish_target = MarkerPublisher()
        self.map_subscriber = MapSubscriber()
        self.element_positions = [
            [-2.93, 3.17],
            [2.86, -3.0],
            [-2.77, -0.96],
            [2.83, 2.93],
        ]
        self.sensor_subscriber = SensorSubscriber()
        self.slam_handler = SlamHandler()
        self.target_dist = init_target_distance
        self.target_dist_increase = target_dist_increase
        self.max_target_dist = max_target_dist
        self.target_reached_delta = target_reached_delta
        self.collision_delta = collision_delta
        
        self.target = self.set_target_position([0.0, 0.0])
        self.process = None
        # self.slam_handler.start()
        
        # Rewards and Penalties
        self.max_reward_in_history = 5000
        self.max_map_exploration_reward = 20
        self.max_idling_penalty = 10
        self.robot_position_buffer_size = 20
        self.robot_position_idle_penalty_threshold = 0.2
        self.robot_position_buffer = deque()
        self.robot_trajectory = []

    def step(self, is_tf_available, lin_velocity=0.0, ang_velocity=0.1):
        self.cmd_vel_publisher.publish_cmd_vel(lin_velocity, ang_velocity)

        self.physics_client.unpause_physics()
        # self.unpause_ros2_bag()
        time.sleep(0.1)
        rclpy.spin_once(self.sensor_subscriber)
        # self.pause_ros2_bag()
        self.physics_client.pause_physics()

        (
            latest_scan,
            latest_position,
            latest_orientation,
            free_pixels,
        ) = self.sensor_subscriber.get_latest_sensor(is_tf_available)
        try :
            while latest_scan == None:
                print(Fore.RED + "Scan not available" + Style.RESET_ALL)
                exit()
        except ValueError as e: # this means either of above has some values. So it cannot be compared to None
            # print(e)
            pass

        self.robot_trajectory.append([latest_position.x,latest_position.y])
        collision = self.check_collision(latest_scan)
        action = [lin_velocity, ang_velocity]
        difference_map_value = self.sensor_subscriber.map_value - self.sensor_subscriber.previous_map_value 
        reward = self.get_reward(collision, action, latest_scan, difference_map_value)

        return latest_scan, latest_position, collision, action, reward, free_pixels

    def terminate(self):
        self.slam_handler.stop()

    def reset(self, is_transform_available):
        # self.stop_ros2_bag()
        maze_number = random.randint(1,5)
        self.world_reset.reset_world(maze_number)
        action = [0.0, 0.0]
        self.cmd_vel_publisher.publish_cmd_vel(
            linear_velocity=action[0], angular_velocity=action[1]
        )

        
        # self.pause_ros2_bag()

        self.element_positions = [
            [-2.93, 3.17],
            [2.86, -3.0],
            [-2.77, -0.96],
            [2.83, 2.93],
        ]
        # self.set_positions()

        # self.publish_target.publish(self.target[0], self.target[1])

        self.slam_handler.maze_number = maze_number
        self.slam_handler.stop()
        self.slam_handler.start()    
        self.sensor_subscriber.transform = None        
        # print("after slam reset")
        self.physics_client.unpause_physics()
        rclpy.spin_once(self.sensor_subscriber)
        print("Waiting after reset")
        time.sleep(3)
        self.physics_client.pause_physics()

        # self.start_ros2_bag(maze_number)
        for i in range(20):
            latest_scan, latest_position, collision, action, reward, free_pixels= self.step(
                is_transform_available, lin_velocity=action[0], ang_velocity=action[1]
            )
        self.robot_trajectory = []

        return latest_scan, latest_position, False, action, reward, free_pixels

    def eval(self, scenario):
        self.cmd_vel_publisher.publish_cmd_vel(0.0, 0.0)

        self.target = [scenario[-1].x, scenario[-1].y]
        # self.publish_target.publish(self.target[0], self.target[1])

        # for element in scenario[:-1]:
        #     self.set_position(element.name, element.x, element.y, element.angle)

        self.physics_client.unpause_physics()
        time.sleep(1)
        latest_scan, latest_position, collision, action, reward, free_pixels = self.step(
            True, lin_velocity=0.0, ang_velocity=0.0
        )
        return latest_scan, latest_position, False, False, action, reward

    def append_to_robot_position_buffer(self, current_position):
        if len(self.robot_position_buffer)<=self.robot_position_buffer_size:
            self.robot_position_buffer.append([current_position.x,current_position.y])
        else:
            self.robot_position_buffer.popleft()
            self.robot_position_buffer.append([current_position.x,current_position.y])
            
    def set_target_position(self, robot_position):
        pos = False
        while not pos:
            x = np.clip(
                robot_position[0]
                + np.random.uniform(-self.target_dist, self.target_dist),
                -4.0,
                4.0,
            )
            y = np.clip(
                robot_position[1]
                + np.random.uniform(-self.target_dist, self.target_dist),
                -4.0,
                4.0,
            )
            pos = self.check_position(x, y, 1.2)
        self.element_positions.append([x, y])
        return [x, y]

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

    def get_reward(self, collision, action, laser_scan, map_value_gain):

        def map_scale(map_value_gain):
            if map_value_gain>self.max_reward_in_history:
                self.max_reward_in_history=map_value_gain

            scaled_map_value_gain = self.max_map_exploration_reward*map_value_gain/ self.max_reward_in_history
            scaled_map_value_gain = max(0, scaled_map_value_gain)
            return scaled_map_value_gain
        
        def avoid_idle():
            # print(self.robot_position_buffer)
            std_position_x = np.std(np.array(self.robot_position_buffer)[:,0])
            std_position_y = np.std(np.array(self.robot_position_buffer)[:,1])
            motion_speed = std_position_x + std_position_y
            if motion_speed < self.robot_position_idle_penalty_threshold:
                idling_penalty = (self.robot_position_idle_penalty_threshold - motion_speed)*self.max_idling_penalty
                # print(f"Idling penalty : {idling_penalty}")
                return idling_penalty
            else: return 0

        def sparcity_reward():
            # print(self.robot_position_buffer)
            n = len(self.robot_trajectory)
            # print(self.robot_trajectory)
            # print(n)
            current_std_x = np.std(np.array(self.robot_trajectory)[:,0])
            current_std_y = np.std(np.array(self.robot_trajectory)[:,1])
            
            # rate_std_x =  current_std_x*n - self.prev_std_x*(n-1)
            # rate_std_y =  current_std_y*n- self.prev_std_y*(n-1)

            # self.prev_std_x = current_std_x
            # self.prev_std_y = current_std_y
            
            scale = 1
            # sparcity = (rate_std_x + rate_std_y) * scale
            sparcity = min(current_std_x,current_std_y)*scale
            return sparcity

        if collision:
            return -100.0
        else:
            r3 = lambda x: 1.35 - x if x < 1.35 else 0.0
            # print(map_scale(map_value_gain))
            # idling_penlaty = avoid_idle()
            map_reward = map_scale(map_value_gain)
            rotation_penalty = abs(action[1])*30
            sparcity_reward_ = sparcity_reward()
            totol_reward = action[0]*10 - rotation_penalty - r3(min(laser_scan)) / 2  
            # print(math.isnan(totol_reward))
            print(f"Rewards----- Total: {totol_reward:.2f} | Map gain: {map_reward:.2f} | Rot Penalty: {rotation_penalty:.2f} | Sparcity : {sparcity_reward_:.2f}")

            return totol_reward

    def start_ros2_bag(self, bag_name="human_data_bag"):
        # Start playing the bag file
        self.process = subprocess.Popen(["ros2", "bag", "play", "src/DRL-exploration/unity_end/human_robot_pkg/rosbag/" + str(bag_name)+"/"],
                                        preexec_fn=os.setsid 
                                        )
        print(f"Playing ROS 2 bag: {bag_name}")

    def pause_ros2_bag(self):
        # print("Pausing playback...")
        self.process.send_signal(signal.SIGSTOP)  # Pause the playback
    
    def unpause_ros2_bag(self):
        # print("Resuming playback...")
        self.process.send_signal(signal.SIGCONT)  # Resume the playback

    def stop_ros2_bag(self):
        """Stop the ROS 2 bag playback process."""
        if self.process:
            os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            self.process.wait()
            self.process = None
            print("Playback stopped.")
        else:
            print("No active playback process.")

    @staticmethod
    def cossin(vec1, vec2):
        vec1 = vec1 / np.linalg.norm(vec1)
        vec2 = vec2 / np.linalg.norm(vec2)
        cos = np.dot(vec1, vec2)
        sin = np.cross(vec1, vec2).item()

        return cos, sin
