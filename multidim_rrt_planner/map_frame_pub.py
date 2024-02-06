import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped


class MapFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('map_frame_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        timer_period = 0.1  # Broadcast at 10 Hz
        self.timer = self.create_timer(timer_period, self.broadcast_map_frame)

    def broadcast_map_frame(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    map_frame_broadcaster = MapFrameBroadcaster()
    rclpy.spin(map_frame_broadcaster)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
