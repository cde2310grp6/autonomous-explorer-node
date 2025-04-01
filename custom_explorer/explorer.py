import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from collections import deque
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
import math



# Mission control
from custom_msg_srv.srv import StartExploration


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # Subscriber to the map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Visited frontiers set
        self.visited_frontiers = set()

        # Map and position data
        self.map_data = None
        self.robot_position = (0, 0)  # Placeholder, update from localization

        self.previous_frontier = None

        # Mission Control
        self.start_explore_srv = self.create_service(StartExploration, '/mission_execute/start_exploration', self.start_exploration)

    def start_exploration(self, request, response):
        if request.explore_now:
            self.get_logger().info("starting exploration by mission_execute")
            self.timer = self.create_timer(5.0, self.explore)
        else:
            self.get_logger().info("stopping exploration by mission_execute")
            self.timer.cancel()
        return response
        

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")

    def navigate_to(self, x, y):
        """
        Send navigation goal to Nav2.
        """
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        
        #### WIP: not as robot_position not being updated
        ### set goal orientation to face in direction from initial_pos to final_pos
        ### to reduce unnecessary rotations
        ### if you don't set this parameter, robot will automatically face front everytime it reaches goal
        # Calculate the yaw angle to face the goal
        robot_x, robot_y = self.robot_position
        yaw = math.atan2(y - robot_y, x - robot_x)
        # Convert the yaw angle to a quaternion
        quaternion = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigating to goal: x={x}, y={y}")

        # Wait for the action server
        self.nav_to_pose_client.wait_for_server()

        # Send the goal and register a callback for the result
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the goal response and attach a callback to the result.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        """
        Callback to handle the result of the navigation action.
        """
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed with result: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")

    def find_frontiers(self, map_array):
        """
        Detect frontiers in the occupancy grid map using BFS.
        """
        frontiers = []
        rows, cols = map_array.shape
        visited = np.zeros_like(map_array, dtype=bool)
        queue = deque()

        # Start BFS from the robot's position
        robot_row, robot_col = self.robot_position
        queue.append((robot_row, robot_col))
        visited[robot_row, robot_col] = True

        while queue:
            r, c = queue.popleft()

            # Check if the current cell is a frontier
            if map_array[r, c] == 0:  # Free cell
                neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                if -1 in neighbors:
                    frontiers.append((r, c))

            # Add neighbors to the queue
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols and not visited[nr, nc]:
                        visited[nr, nc] = True
                        queue.append((nr, nc))

        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def choose_frontier(self, frontiers, map_array):
        """
        Choose the best frontier to explore based on distance, number of unknown cells around it, and direction.
        """
        robot_row, robot_col = self.robot_position
        min_score = float('inf')
        chosen_frontier = None

        for frontier in frontiers:
            if frontier in self.visited_frontiers:
                continue

            # Calculate distance to the robot
            distance = np.sqrt((robot_row - frontier[0])**2 + (robot_col - frontier[1])**2)

            # Calculate the number of unknown cells around the frontier
            r, c = frontier
            neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
            unknown_count = np.sum(neighbors == -1)

            # Check for obstacles around the frontier
            obstacle_free = True
            check_range = 15
            for i in range(r-check_range, r + check_range):
                for j in range(c-check_range, c + check_range):
                    try: # try-except to reject frontiers too close to boundary of map
                        if map_array[i, j] == 100:  # Assuming 100 represents an obstacle
                            obstacle_free = False
                            break
                    except:
                        break
                if not obstacle_free:
                    break

            if not obstacle_free:
                continue

            # Calculate a score based on distance, unknown count, and direction
            direction_penalty = 0
            if self.previous_frontier:
                prev_r, prev_c = self.previous_frontier
                direction_penalty = np.sqrt((prev_r - r)**2 + (prev_c - c)**2)

            score = distance - unknown_count + direction_penalty  # Adjust the weights as needed

            if score < min_score:
                min_score = score
                chosen_frontier = frontier

        if chosen_frontier:
            self.visited_frontiers.add(chosen_frontier)
            self.previous_frontier = chosen_frontier
            self.get_logger().info(f"Chosen frontier: {chosen_frontier}")
        else:
            self.get_logger().warning("No valid frontier found")

        return chosen_frontier

    def explore(self):
        self.get_logger().info(f"exploring now!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        # Convert map to numpy array
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        # Detect frontiers
        frontiers = self.find_frontiers(map_array)

        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration complete!")



            # self.shutdown_robot()
            return

        # Choose the closest frontier
        chosen_frontier = self.choose_frontier(frontiers, map_array)

        if not chosen_frontier:
            self.get_logger().warning("No frontiers to explore")
            return

        # Convert the chosen frontier to world coordinates
        goal_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

        # Navigate to the chosen frontier
        self.navigate_to(goal_x, goal_y)

    # def shudown_robot(self):
    #     
    #
    #
    #     self.get_logger().info("Shutting down robot exploration")


def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    try:
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        explorer_node.destroy_node()
        rclpy.shutdown()
