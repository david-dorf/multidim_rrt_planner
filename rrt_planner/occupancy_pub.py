import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header


class OccupancyGridPublisher(Node):

    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        self.publisher = self.create_publisher(
            OccupancyGrid, 'occupancy_grid', 10)
        self.timer = self.create_timer(
            1.0, self.publish_occupancy_grid)  # Publish at 1 Hz

    def publish_occupancy_grid(self):
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = 'map'
        occupancy_grid_msg.info.resolution = 0.05  # Cell size in meters
        # Number of cells in the grid along the x-axis
        occupancy_grid_msg.info.width = 100
        # Number of cells in the grid along the y-axis
        occupancy_grid_msg.info.height = 100
        occupancy_grid_msg.info.origin.position.x = -2.5
        occupancy_grid_msg.info.origin.position.y = -2.5
        occupancy_grid_msg.data = [
            0] * (occupancy_grid_msg.info.width * occupancy_grid_msg.info.height)

        # Populate the occupancy grid data
        for i in range(20, 80):
            occupancy_grid_msg.data[i *
                                    occupancy_grid_msg.info.width + 20] = 100
            occupancy_grid_msg.data[i *
                                    occupancy_grid_msg.info.width + 80] = 100
        for i in range(20, 80):
            occupancy_grid_msg.data[20 *
                                    occupancy_grid_msg.info.width + i] = 100
            occupancy_grid_msg.data[80 *
                                    occupancy_grid_msg.info.width + i] = 100
        for i in range(40, 60):
            occupancy_grid_msg.data[(
                i + 40) * occupancy_grid_msg.info.width + i] = 100
            occupancy_grid_msg.data[(
                i + 40) * occupancy_grid_msg.info.width + (60 - i)] = 100

        self.publisher.publish(occupancy_grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
