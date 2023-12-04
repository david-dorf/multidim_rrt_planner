"""Publishes obstacles in 2D."""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from .submodules.Marker import Circle, Rectangle, create_marker


class ObstaclePublisher2D(Node):
    """
    Publishes obstacles in 2D.

    Methods
    -------
    publish_obstacle()
        Publishes obstacles in 2D.

    Attributes
    ----------
    publisher : rclpy.publisher.Publisher
        Publisher for obstacle markers
    timer : rclpy.timer.Timer
        Timer for publishing obstacles
    obstacle_1 : Circle
        First example obstacle
    obstacle_2 : Rectangle
        Second example obstacle
    obstacle_list : list
        List of obstacles

    """

    def __init__(self):
        """Initialize the node."""
        super().__init__('obstacle_publisher_2D')
        self.publisher = self.create_publisher(
            MarkerArray, 'obstacle_markers_2D', 10)
        self.timer = self.create_timer(
            1.0, self.publish_obstacle)
        self.obstacle_1 = Circle(1.0, 1.0, 1.0)
        self.obstacle_2 = Rectangle(-1.0, -1.0, 1.0, 1.0, 0.0)
        self.obstacle_list = [self.obstacle_1, self.obstacle_2]

    def publish_obstacle(self):
        """Publish obstacles in 2D."""
        marker_array = MarkerArray()
        for obstacle in self.obstacle_list:
            if isinstance(obstacle, Circle):
                marker = create_marker(Marker.CYLINDER, self.obstacle_list.index(
                    obstacle), [1.0, 0.0, 0.0, 1.0],
                    [obstacle.radius * 2, obstacle.radius * 2, 0.1], [obstacle.x, obstacle.y, 0.0])
                marker_array.markers.append(marker)
            elif isinstance(obstacle, Rectangle):
                marker = create_marker(Marker.CUBE, self.obstacle_list.index(
                    obstacle), [1.0, 0.0, 0.0, 1.0],
                    [obstacle.width, obstacle.height, 0.1], [obstacle.x, obstacle.y, 0.0])
                marker_array.markers.append(marker)
        self.publisher.publish(marker_array)


def main(args=None):
    """Run the main function."""""
    rclpy.init(args=args)
    obstacle_publisher = ObstaclePublisher2D()
    rclpy.spin(obstacle_publisher)
    rclpy.shutdown()
