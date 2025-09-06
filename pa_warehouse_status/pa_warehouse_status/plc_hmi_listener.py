import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = '/hmi/unified_status'

class PlcHmiListener(Node):
    def __init__(self):
        super().__init__('plc_hmi_listener')
        # Subscribe to String messages on TOPIC (queue depth = 10)
        self.sub = self.create_subscription(String, TOPIC, self.listener_callback, 10)

    def listener_callback(self, msg: String):
        # msg.data is a text string; expected to be JSON
        data   = json.loads(msg.data) # convert JSON text -> Python dict
        stamp  = data["stamp"] # dict with 'sec' and 'nanosec'
        box    = data["box"] # dict with 'weight_raw', 'location', ...
        counts = data["counts"] # dict with 'big','medium','small','total'
        # Log the key fields in a nice format
        self.get_logger().info(
            f"PLC status | t={stamp['sec']}.{stamp['nanosec']} "
            f"weight_raw={box['weight_raw']} loc={box['location']} "
            f"counts: big={counts['big']} med={counts['medium']} "
            f"small={counts['small']} total={counts['total']}"
        )

def main():
    rclpy.init()
    rclpy.spin(PlcHmiListener()) # keep the node alive; callback fires on each message
    rclpy.shutdown()

