import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class TFPublisher(Node):
    """
    Subscribes to the raw TransformStamped message from the iPhone
    and broadcasts it to the global ROS TF tree.
    """
    def __init__(self):
        super().__init__('arkit_tf_broadcaster')
        
        # 1. Initialize the TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 2. Subscribe to the iPhone's pose topic
        self.subscription = self.create_subscription(
            TransformStamped,
            '/arkit/pose_tf', # The topic you publish from the iPhone
            self.transform_callback,
            10 # QoS setting
        )
        self.get_logger().info('ARKit TF Broadcaster started, listening to /arkit/pose_tf')

    def transform_callback(self, msg):
        # The message received is already a TransformStamped, so we just
        # forward it directly to the TF Broadcaster.
        
        # The header (timestamp and frame_id) and child_frame_id are set by the iPhone app.
        self.tf_broadcaster.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TFPublisher()
    
    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        pass
    
    tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
