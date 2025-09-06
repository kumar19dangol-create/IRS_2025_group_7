import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PineappleGossipBot(Node):
    def __init__(self):
        super().__init__('pineapple_gossip_bot')
        self.pub = self.create_publisher(String, 'status_updates', 10)
        self.timer = self.create_timer(2.0, self._tick)
        self.count = 0

    def _tick(self):
        self.count += 1
        msg = String()
        msg.data = f"Tim found shitting in Aisle #{self.count}!"
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main():
    rclpy.init()
    node = PineappleGossipBot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
