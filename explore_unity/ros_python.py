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
import math

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
        self.slam_handler.start()
        
        # Rewards and Penalties
        self.max_reward_in_history = 50
        self.max_map_exploration_reward = 50
        self.map_difference_threshold = 200
        self.map_buffer_counter = 0
        self.map_buffer_size = 8
        self.max_idling_penalty = 3
        self.robot_position_buffer_size = 20
        self.robot_position_idle_penalty_threshold = 1.2
        self.robot_position_buffer = deque()
        self.robot_trajectory = []
        self.prev_std_x = 0
        self.prev_std_y = 0

    def terminate(self):
        self.slam_handler.stop()

    def step(self, is_tf_available, lin_velocity=0.0, ang_velocity=0.1):
        self.cmd_vel_publisher.publish_cmd_vel(lin_velocity, ang_velocity)
        self.physics_client.unpause_physics()
        time.sleep(0.1)
        rclpy.spin_once(self.sensor_subscriber)
        
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
                continue
        except ValueError as e: # this means either of above has some values. So it cannot be compared to None
            print(e)
            pass

        self.append_to_robot_position_buffer(latest_position)
        self.robot_trajectory.append([latest_position.x,latest_position.y])
        collision = self.check_collision(latest_scan)
        if collision:
            print(Fore.RED + "Collision!" + Style.RESET_ALL)
        action = [lin_velocity, ang_velocity]
        # difference_map_value = self.sensor_subscriber.map_value - self.sensor_subscriber.previous_map_value 
        # reward = self.get_reward(collision, action, latest_scan, difference_map_value)
        reward = self.get_reward(collision, action, latest_scan)

        return latest_scan, latest_position, collision, action, reward, free_pixels

    def ready(self):
        if self.sensor_subscriber.latest_position==None or self.sensor_subscriber.latest_scan==None:
            return False
        else: return True

    def reset(self, is_transform_available):
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
        # self.set_positions()

        # self.publish_target.publish(self.target[0], self.target[1])

        self.slam_handler.stop()
        self.slam_handler.start()    
        self.sensor_subscriber.transform = None  
        self.prev_std_x=0
        self.prev_std_y=0   
        

        self.physics_client.unpause_physics()
        rclpy.spin_once(self.sensor_subscriber)
        print("Waiting after reset")
        time.sleep(0.5)
        self.physics_client.pause_physics()
        
        while True:
            self.cmd_vel_publisher.publish_cmd_vel(0.0, 0.0)
            self.physics_client.unpause_physics()
            time.sleep(0.1)
            rclpy.spin_once(self.sensor_subscriber)
            self.physics_client.pause_physics()
            if self.ready(): break

        self.robot_trajectory = []
        latest_scan, latest_position, collision, action, reward, free_pixels= self.step(
                is_transform_available, lin_velocity=action[0], ang_velocity=action[1]
            )
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

    def get_furthest_angle(scan_ranges, angle_min, angle_max):
        """
        Get the angle that gives the furthest distance in laser scan.

        Args:
            scan_ranges (list or np.array): Laser scan distance readings.
            angle_min (float): Minimum angle of laser scan in radians.
            angle_max (float): Maximum angle of laser scan in radians.

        Returns:
            float: Angle in radians where the furthest distance is detected.
        """
        # Convert scan to numpy array
        scan_ranges = np.array(scan_ranges)
        
        # Get the index of the maximum distance (furthest point)
        max_index = np.argmax(scan_ranges)
        
        # Calculate angle step size between each laser beam
        angle_step = (angle_max - angle_min) / len(scan_ranges)
        
        # Get the angle corresponding to the max distance
        furthest_angle = angle_min + max_index * angle_step
        
        return furthest_angle

    def get_reward(self, collision, action, laser_scan):

        def map_scale(map_value_gain):
            if map_value_gain>self.max_reward_in_history:
                self.max_reward_in_history=map_value_gain

            scaled_map_value_gain = self.max_map_exploration_reward*map_value_gain/ self.max_reward_in_history
            scaled_map_value_gain = max(0, scaled_map_value_gain)
            return scaled_map_value_gain
        
        def map_reward_binary():
            if self.sensor_subscriber.map_value > self.sensor_subscriber.previous_map_value + self.map_difference_threshold:
                self.map_buffer_counter = 0
                return 1
            else:
                if self.map_buffer_counter< self.map_buffer_size:
                    self.map_buffer_counter +=1
                    return 1
                else:
                    return -1
        
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
            # print(n)
            current_std_x = np.std(np.array(self.robot_trajectory)[:,0])
            current_std_y = np.std(np.array(self.robot_trajectory)[:,1])
            
            rate_std_x =  current_std_x*n - self.prev_std_x*(n-1)
            rate_std_y =  current_std_y*n- self.prev_std_y*(n-1)

            self.prev_std_x = current_std_x
            self.prev_std_y = current_std_y
            
            scale = 1
            # sparcity = (rate_std_x + rate_std_y) * scale
            sparcity = (current_std_x + current_std_y) * scale
            return sparcity

        def trajectory_deviation():
            current_position = self.robot_trajectory[-1]
            total_distance = 0
            for point in self.robot_trajectory[:-1]:
                total_distance += math.sqrt((point[0]-current_position[0])**2 + (point[1]-current_position[1])**2)
            try:
                avg_distance = total_distance/(len(self.robot_trajectory)-1)
                return avg_distance
            except ZeroDivisionError:
                return 0

        if collision:
            return -100.0
        else:
            r3 = lambda x: 2.5 - x if x < 2.5 else 0.0
            # print(map_scale(map_value_gain))
            # idling_penlaty = avoid_idle()
            map_reward = map_reward_binary()
            ang_penalty = abs(action[1])/2
            sparcity_reward_ = sparcity_reward()
            idling_penalty = avoid_idle()
            collision_penalty = r3(min(laser_scan))
            trajectory_deviation_ = trajectory_deviation()
            totol_reward = action[0]  + map_reward- collision_penalty - ang_penalty - idling_penalty
            # print(math.isnan(totol_reward))
            # print(f"Rewards----- Map gain: {map_reward:.2f} | Lin vel: {action[0]:.2f} | Col Pen : {collision_penalty:.2f} | traj dev: {trajectory_deviation:.2f} | Total: {totol_reward:.2f}")
            reward_str = "Rewards----- "
            reward_str += f"Map gain: {map_reward:.2f} | " 
            reward_str += f"Lin vel: {action[0]:.2f} | "
            reward_str += f"Col Pen: {collision_penalty:.2f} | " 
            # reward_str += f"traj dev: {trajectory_deviation_:.2f} | " 
            reward_str += f"ang pen: {ang_penalty:.2f} | " 
            reward_str += f"idle pen: {idling_penalty:.2f} | " 
            reward_str += f"Total: {totol_reward:.2f}" 
            print(reward_str)
            return totol_reward

    @staticmethod
    def cossin(vec1, vec2):
        vec1 = vec1 / np.linalg.norm(vec1)
        vec2 = vec2 / np.linalg.norm(vec2)
        cos = np.dot(vec1, vec2)
        sin = np.cross(vec1, vec2).item()

        return cos, sin
