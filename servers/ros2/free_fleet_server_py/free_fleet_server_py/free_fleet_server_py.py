import rclpy
from rclpy.node import Node

from rmf_fleet_msgs.msg import RobotState


class FreeFleetServer(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.create_subscription(RobotState, 'free_fleet_robot_states', self.cb)

    def cb(self):
        print('msg rx\n')

    def main(self):
        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass


def main():
    rclpy.init()
    FreeFleetServer('free_fleet_server').main()


if __name__ == '__main__':
    main()
