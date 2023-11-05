import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class ObstaclePublisher(Node):

    def __init__(self):
        super().__init__('obstacle_publisher')
        self.publisher = self.create_publisher(
            MarkerArray, 'obstacle_topic', 10)
        self.timer = self.create_timer(
            1.0, self.publish_obstacle)
