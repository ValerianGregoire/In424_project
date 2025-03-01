__author__ = "Johvany Gustave, Jonatan Alvarez"
__copyright__ = "Copyright 2025, IN424, IPSA 2025"
__credits__ = ["Johvany Gustave", "Jonatan Alvarez"]
__license__ = "Apache License 2.0"
__version__ = "1.0.0"


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import euler_from_quaternion

import numpy as np

from .my_common import *    #common variables are stored here


class Agent(Node):
    """
    This class is used to define the behavior of ONE agent
    """
    def __init__(self):
        Node.__init__(self, "Agent")
        
        self.load_params()

        #initialize attributes
        self.agents_pose = [None]*self.nb_agents    #[(x_1, y_1), (x_2, y_2), (x_3, y_3)] if there are 3 agents
        self.x = self.y = self.yaw = None   #the pose of this specific agent running the node
        self.row = self.col = self.dest_row = self.dest_col = None # Position of this agent in the gridmap

        # Lidar variables
        self.ranges = self.intensities = self.min_angle = self.max_angle = self.step = None # Lidar data

        self.map_agent_pub = self.create_publisher(OccupancyGrid, f"/{self.ns}/map", 1) #publisher for agent's own map
        self.init_map()

        # Movement variables
        self.init_movement()

        #Subscribe to agents' pose topic
        odom_methods_cb = [self.odom1_cb, self.odom2_cb, self.odom3_cb]
        for i in range(1, self.nb_agents + 1):  
            self.create_subscription(Odometry, f"/bot_{i}/odom", odom_methods_cb[i-1], 1)
        
        if self.nb_agents != 1: #if other agents are involved subscribe to the merged map topic
            self.create_subscription(OccupancyGrid, "/merged_map", self.merged_map_cb, 1)
        
        self.create_subscription(LaserScan, f"{self.ns}/laser/scan", self.lidar_cb, qos_profile=qos_profile_sensor_data) #subscribe to the agent's own LIDAR topic
        self.cmd_vel_pub = self.create_publisher(Twist, f"{self.ns}/cmd_vel", 1)    #publisher to send velocity commands to the robot

        #Create timers to autonomously call the following methods periodically
        self.create_timer(0.2, self.map_update) #0.2s of period <=> 5 Hz
        self.create_timer(0.5, self.strategy)      #0.5s of period <=> 2 Hz
        self.create_timer(1, self.publish_maps) #1Hz

    def load_params(self):
        """ Load parameters from launch file """
        self.declare_parameters(    #A node has to declare ROS parameters before getting their values from launch files
            namespace="",
            parameters=[
                ("ns", rclpy.Parameter.Type.STRING),    #robot's namespace: either 1, 2 or 3
                ("robot_size", rclpy.Parameter.Type.DOUBLE),    #robot's diameter in meter
                ("env_size", rclpy.Parameter.Type.INTEGER_ARRAY),   #environment dimensions (width height)
                ("nb_agents", rclpy.Parameter.Type.INTEGER),    #total number of agents (this agent included) to map the environment
            ]
        )

        #Get launch file parameters related to this node
        self.ns = self.get_parameter("ns").value
        self.robot_size = self.get_parameter("robot_size").value
        self.env_size = self.get_parameter("env_size").value
        self.nb_agents = self.get_parameter("nb_agents").value

    def init_map(self):
        """ Initialize the map to share with others if it is bot_1 """
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "map"    #set in which reference frame the map will be expressed (DO NOT TOUCH)
        self.map_msg.header.stamp = self.get_clock().now().to_msg() #get the current ROS time to send the msg
        self.map_msg.info.resolution = self.robot_size  #Map cell size corresponds to robot size
        self.map_msg.info.height = int(self.env_size[0]/self.map_msg.info.resolution)   #nb of rows
        self.map_msg.info.width = int(self.env_size[1]/self.map_msg.info.resolution)    #nb of columns
        self.map_msg.info.origin.position.x = -self.env_size[1]/2   #x and y coordinates of the origin in map reference frame
        self.map_msg.info.origin.position.y = -self.env_size[0]/2
        self.map_msg.info.origin.orientation.w = 1.0    #to have a consistent orientation in quaternion: x=0, y=0, z=0, w=1 for no rotation
        self.map = np.ones(shape=(self.map_msg.info.height, self.map_msg.info.width), dtype=np.int8)*UNEXPLORED_SPACE_VALUE #all the cells are unexplored initially
        self.w, self.h = self.map_msg.info.width, self.map_msg.info.height  
    
    def coordinates_to_grid(self, x, y):
        """
            @brief Computes the indexes (i,j) of the grid cell corresponding to the coordinates
        """
        # resolution = 0.5; env = [20, 20]; h = 40; w = 40

        # x goes from -10 to 10 -> +10 to go from 0 to 20
        x += self.env_size[0]/2
        # y goes from 10 to -10 -> (-10) * -1 to go from 0 to 20 
        y = (y - self.env_size[1]/2) * -1

        # Check for incorrect numbers
        if float('inf') in (x, y) or x != x or y != y:
            return None, None
        
        grid_x = int(x  // self.map_msg.info.resolution)
        grid_y = int(y // self.map_msg.info.resolution)

        # Ignore out of grid values
        if grid_x >= self.h or grid_y >= self.w:
            return None, None
        
        return (grid_y, grid_x) # The grid inverts x and y

    def grid_to_coordinates(self, i, j):
        """
            Returns the coordinates in map coordinates of a grid cell
        """
        # Goes from 0 to 20
        try:
            x = (i + 1) * self.map_msg.info.resolution
            y = (j + 1) * self.map_msg.info.resolution
        except TypeError:
            return None, None

        # Adjust to go from -10 to 10
        x -= self.env_size[0]/2

        # Adjust to go from 10 to -10
        y = (y - self.env_size[1]/2) * -1

        return x, y

    def merged_map_cb(self, msg):
        """ 
            Get the current common map and update ours accordingly.
            This method is automatically called whenever a new message is published on the topic /merged_map.
            'msg' is a nav_msgs/msg/OccupancyGrid message.
        """
        received_map = np.flipud(np.array(msg.data).reshape(self.h, self.w))    #convert the received list into a 2D array and reverse rows
        for i in range(self.h):
            for j in range(self.w):
                if (self.map[i, j] == UNEXPLORED_SPACE_VALUE) and (received_map[i, j] != UNEXPLORED_SPACE_VALUE):
                # if received_map[i, j] != UNEXPLORED_SPACE_VALUE:
                    self.map[i, j] = received_map[i, j]


    def odom1_cb(self, msg):
        """ 
            @brief Get agent 1 position.
            This method is automatically called whenever a new message is published on topic /bot_1/odom.
            
            @param msg This is a nav_msgs/msg/Odometry message.
        """
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if int(self.ns[-1]) == 1:
            self.x, self.y = x, y
            self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.agents_pose[0] = (x, y)
        # self.get_logger().info(f"Agent 1: ({x:.2f}, {y:.2f})")
    

    def odom2_cb(self, msg):
        """ 
            @brief Get agent 2 position.
            This method is automatically called whenever a new message is published on topic /bot_2/odom.
            
            @param msg This is a nav_msgs/msg/Odometry message.
        """
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if int(self.ns[-1]) == 2:
            self.x, self.y = x, y
            self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.agents_pose[1] = (x, y)
        # self.get_logger().info(f"Agent 2: ({x:.2f}, {y:.2f})")


    def odom3_cb(self, msg):
        """ 
            @brief Get agent 3 position.
            This method is automatically called whenever a new message is published on topic /bot_3/odom.
            
            @param msg This is a nav_msgs/msg/Odometry message.
        """
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if int(self.ns[-1]) == 3:
            self.x, self.y = x, y
            self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        self.agents_pose[2] = (x, y)
        # self.get_logger().info(f"Agent 3: ({x:.2f}, {y:.2f})")


    def map_update(self):
        """ Consider sensor readings to update the agent's map """
        # Get angles
        angles = np.arange(self.min_angle, self.max_angle + self.step, self.step)
        # Loop through each laser
        for i, range_ in enumerate(self.ranges):
            # self.get_logger().info(f"self.x={self.x}, self.y={self.y}")
            if not range_ == float('inf'):
                azimuth = angles[i]
                if not self.yaw: # Prevent computations without position data
                    break
                true_azimuth = self.yaw + azimuth

                wall_x, wall_y = self.x + np.cos(true_azimuth) * range_, self.y + np.sin(true_azimuth) * range_
                j, k = self.coordinates_to_grid(wall_x, wall_y)
                if j == None or k == None:
                    break
                self.map[self.coordinates_to_grid(self.x, self.y)] = FREE_SPACE_VALUE
                self.map[j, k] = OBSTACLE_VALUE
        pass
    

    def lidar_cb(self, msg):
        """ 
            @brief Get messages from LIDAR topic.
            This method is automatically called whenever a new message is published on topic /bot_x/laser/scan, where 'x' is either 1, 2 or 3.
            
            @param msg This is a sensor_msgs/msg/LaserScan message.
        """
        self.ranges, self.intensities = msg.ranges, msg.intensities
        self.min_angle, self.max_angle, self.step = msg.angle_min, msg.angle_max, msg.angle_increment
        self.map_update()
        # self.get_logger().info(f"Agent {id(self)}: {self.min_angle}")

    def publish_maps(self):
        """ 
            Publish updated map to topic /bot_x/map, where x is either 1, 2 or 3.
            This method is called periodically (1Hz) by a ROS2 timer, as defined in the constructor of the class.
        """
        self.map_msg.data = np.flipud(self.map).flatten().tolist()  #transform the 2D array into a list to publish it
        self.map_agent_pub.publish(self.map_msg)    #publish map to other agents

    def init_movement(self):
        self.cmd_vel = Twist()
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
    
    def move_to_dest(self):
        """
        State machine to rotate towards/drive to a given row/col cell combination.
        """
        # Convert the target row and column to world coordinates
        dest_x, dest_y = self.grid_to_coordinates(self.dest_row, self.dest_col)
        # self.get_logger().info(f"Target x: {dest_x}\nTarget y: {dest_y}")

        # Calculate deltas
        try:
            delta_x = dest_x - self.x  # x-axis difference
            delta_y = dest_y - self.y  # y-axis difference
        except TypeError:
            return
        
        # Compute target angle using arctan2 (result is in [-pi, pi])
        # Rows and cols are inverted in the world, so the angle is inverted too
        target_angle = np.arctan2(delta_y, delta_x)
        
        # Convert target_angle to [0, 2π)
        if target_angle < 0:
            target_angle += 2 * np.pi

        # Compute angle difference, ensuring it's within [-π, π]
        angle_diff = (target_angle - self.yaw + np.pi) % (2 * np.pi) - np.pi

        # Tuning parameters
        angle_tolerance = 0.1  # Radians: acceptable angle error
        speed = 1.0            # Linear speed
        rotation_speed = 0.5   # Angular speed

        # Stop if at the destination
        if (self.row, self.col) == (self.dest_row, self.dest_col):
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
        
        # Move forward if facing the right direction
        elif abs(angle_diff) < angle_tolerance:
            self.cmd_vel.linear.x = speed
            self.cmd_vel.angular.z = 0.0
        
        # Rotate towards target angle
        else:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = rotation_speed * np.sign(angle_diff)

    def strategy(self):
        """ Decision and action layers """

        # Do not touch (update current gridmap position data)
        self.row, self.col = self.coordinates_to_grid(self.x, self.y)

        # ======================== UPDATE THIS SECTION ========================
        # Make sure no empty values are passed to the algorithm
        if None in (self.dest_row, self.dest_col):
            self.dest_row = self.dest_col = 0

        # Update self.dest_row and self.dest_col


        # =====================================================================

        # Do not touch (state machine for movement)
        self.move_to_dest()

        # Publish command
        self.cmd_vel_pub.publish(self.cmd_vel)
        pass

def main():
    rclpy.init()

    node = Agent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    while True:
        rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()