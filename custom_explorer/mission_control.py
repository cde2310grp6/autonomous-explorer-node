import rclpy
from rclpy.node import Node

# service for control of exploration node
from custom_msg_srv.srv import StartExploration

# for launcher service
from std_srvs.srv import Trigger 


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.get_logger().info("Mission Control Node Started")

        # sub to explorer service for start exploration
        self.exploration_service = self.create_client(StartExploration, 'start_exploration')

        # sub to launcher service
        self.launcher_service = self.create_client(Trigger, 'launch_ball')

        self.mission_start()

    def toggle_exploration(self, explore_now):
        self.req = StartExploration.Request()
        self.req.explore_now = explore_now
        while not self.exploration_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('explorer not avail, waiting...')
        self.exploration_service.call_async(self.req)


    def launch_now(self):
        self.get_logger().info("calling launcher")
        self.req = Trigger.Request()
        while not self.launcher_service.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('launcher not avail, waiting...')
        self.launcher_service.call_async(self.req)


    def mission_start(self):
        self.get_logger().info("Starting mission...")
        # Here you would typically initialize your explorer node
        # self.explorer_node = ExplorerNode()
        # self.explorer_node.start_exploration()
        self.toggle_exploration(explore_now=True)
            

def main():
    rclpy.init()
    mission_control = MissionControl()

    try:
        rclpy.spin(mission_control)
    except KeyboardInterrupt:
        pass
    finally:
        mission_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()