import rclpy
from rclpy.node import Node

from time import sleep

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# for calling exploration node
from custom_msg_srv.srv import StartExploration

# for mission control
from custom_msg_srv.msg import CasualtyDetect, MapExplored

# for activating launcher
from std_srvs.srv import Trigger

class MissionExecuteNode(Node):
    def __init__(self):
        super().__init__('mission_execute')
        self.get_logger().info("Mission Execute Node Started")

        # Toggle exploration mode of explore node
        self.explore_client = self.create_client(StartExploration, '/mission_execute/start_exploration')
        # Subscribe to exploration node completion status
        self.explore_complete_subscriber = self.create_subscription(MapExplored, 'map_explored', self.map_explored_callback, 10)
        self.explore_complete = False

        self.task_complete = False
        self.casualties_found = 0

        # Subscriber to the map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Map and position data
        self.map_data = None
        self.robot_position = (0, 0)  # Placeholder, update from localization

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose_mission_exec')

        # Subscriber to casualty_detect
        # thermal cam data
        self.casualty_subscriber = self.create_subscription(CasualtyDetect, 'casualty_detect', self.casualty_callback, 10)
        self.casualty_yaws = []
        self.got_casualty = False

        # subscribe to launcher service
        self.launcher_control = self.create_client(Trigger, '/launch_ball')

        # start the mission!
        self.mission_execute()

    def explorenode(self, exploreNow):
        request = StartExploration.Request()
        request.explore_now = exploreNow
        while not self.explore_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('explore node service not available, waiting again...')
        future = self.explore_client.call_async(request)

    def map_explored_callback(self, msg):
        self.explore_complete = msg.explore_complete
        self.get_logger().info("Map fully explored")

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")

    def casualty_callback(self, msg):
        self.casualty_yaws = msg.yaws
        self.got_casualty = msg.got_casualty

    def move_to_casualty(self):
        # move to the casualty location
        # get the yaw of the casualty location
        # rotate the robot to face the casualty location
        # move to the casualty location
        self.launch_ball()
        self.task_complete = True
        self.casualties_found += 1
        pass

    def launch_ball(self):
        request = Trigger.Request()
        while not self.launcher_control.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('launch ball service not available, waiting again...')
        future = self.launcher_control.call_async(request)

    def random_walk(self):
        # move to random location
        # rotate the robot to face the random location
        # move to the random location  
        pass

    def mission_execute(self):
        """
        Mission execution logic
        1. start exploration until receive map fully explored message from exploration node
        in between if got casualty, 
        if not the same casualty as before,
            stop exploreNode(exploreNow=False) 
            mark on the map location of casualty.
        else 
            continue exploration
        """

        # call exploration to begin exploration
        self.explorenode(exploreNow=True)
        self.launch_ball()
"""
        # keep exploring until map is fully explored
        while not self.explore_complete:

            # if got casualty keep calling move_to_casualty until self.task_complete
            if self.gotCasualty:
                self.explorenode(exploreNow=False)
                while not self.task_complete:
                    self.move_to_casualty()

                self.explorenode(exploreNow=True)
                self.task_complete = False

        # map fully explored
        # assuming we only need to find 3 casualties for mission completion
        while (self.casualties_found < 3):
            self.random_walk()

            if self.gotCasualty:
                while not self.task_complete:
                    self.move_to_casualty()

        # mission complete
        # proceed to ramp...
"""










def main(args=None):
    rclpy.init(args=args)
    mission_execute = MissionExecuteNode()
    rclpy.spin(mission_execute)
    rclpy.shutdown()

if __name__ == "__main__":
    main()