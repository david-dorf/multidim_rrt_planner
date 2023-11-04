import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
from .submodules.TreeNode import TreeNode
from .submodules.Obstacles import Circle, Rectangle


class RRT2DNode(Node):
    """
    Generates a 2D RRT path and publishes it as a Path message and as a set of markers

    Attributes
    ----------
    start_position : np.array
        The starting position of the RRT
    goal_position : np.array
        The goal position of the RRT
    map_size : np.array
        The size of the map
    node_limit : int
        The maximum number of nodes to generate
    goal_tolerance : float
        The maximum distance between the goal and the final node
    step_size : float
        The step size for each node
    animate : bool
        Whether or not to animate the RRT

    Methods
    -------
    run_rrt_2D()
        Generates the RRT
    create_marker()
        Creates a marker for visualization
    publish_markers()
        Publishes the markers
    publish_path()
        Publishes the path
    plot_rrt_2D()
        Plots the RRT
    """

    def __init__(self):
        super().__init__('rrt_2d_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('start_position', [0., 0.]),
                ('goal_position', [1., -1.]),
                ('map_sub_mode', False)
            ]
        )
        self.start_position = np.array(
            self.get_parameter('start_position').value)
        self.goal_position = np.array(
            self.get_parameter('goal_position').value)
        self.map_sub_mode = self.get_parameter('map_sub_mode').value
        if self.map_sub_mode:
            self.map_data = None
            self.occupancy_grid_subscription = self.create_subscription(
                OccupancyGrid, 'occupancy_grid_topic', self.occupancy_grid_callback, 10)
        else:
            self.map_size = np.array([10, 10])
            self.map_resolution = 1.0
        self.node_list = []
        self.node_limit = 5000
        self.goal_tolerance = 0.2
        self.step_size = 0.05
        self.wall_confidence = 50
        self.animate = True
        self.obstacle_1 = Circle(1.0, 1.0, 1.0)
        self.obstacle_2 = Rectangle(-1.0, -1.0, 1.0, 1.0, 0.0)
        self.obstacle_list = [self.obstacle_1, self.obstacle_2]
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.run_rrt_2D()

    def run_rrt_2D(self):
        """
        Generates the RRT
        """
        if self.map_sub_mode:
            while self.map_data is None:
                self.get_logger().info('Waiting for map data...')
                rclpy.spin_once(self)
        start_node = TreeNode(self.start_position, None)
        self.node_list = [start_node]
        completed = False
        while len(self.node_list) < self.node_limit:
            random_position_x = np.random.uniform(
                -(self.map_size[0] * self.step_size)/2, (self.map_size[0] * self.step_size)/2)
            random_position_y = np.random.uniform(
                -(self.map_size[1] * self.step_size)/2, (self.map_size[1] * self.step_size)/2)
            random_position = np.array([random_position_x, random_position_y])
            min_distance = np.inf
            min_node = None
            for node in self.node_list:
                goal_vec = self.goal_position - node.val
                goal_distance = np.linalg.norm(goal_vec)
                goal_node = TreeNode(self.goal_position, node)
                if goal_distance < self.goal_tolerance:
                    node.add_child(goal_node)
                    self.node_list.append(goal_node)
                    completed = True
                    break
                new_node_vec = random_position - node.val  # Find node closest to random point
                distance = np.linalg.norm(new_node_vec)
                if distance < min_distance:
                    min_node_vec = new_node_vec
                    min_node = node
                min_distance = np.min([distance, min_distance])
            if completed:
                break
            if min_distance != 0:
                new_node_unit_vec = min_node_vec / min_distance
                new_node_val = min_node.val + new_node_unit_vec * self.step_size
                new_node = TreeNode(new_node_val, min_node)
                obstacle_collision = False  # Check if new_node collides with any obstacles
                if self.obstacle_list:
                    for obstacle in self.obstacle_list:
                        if isinstance(obstacle, Circle):
                            obstacle_vec = new_node.val - \
                                np.array([obstacle.x, obstacle.y])
                            obstacle_distance = np.linalg.norm(obstacle_vec)
                            if obstacle_distance < obstacle.radius:
                                obstacle_collision = True
                                break
                        elif isinstance(obstacle, Rectangle):
                            if (obstacle.x - obstacle.width/2 < new_node.val[0] < obstacle.x + obstacle.width/2) and (obstacle.y - obstacle.height/2 < new_node.val[1] < obstacle.y + obstacle.height/2):
                                obstacle_collision = True
                                break
                    if obstacle_collision:
                        continue
                wall_collision = False  # Check if new_node is closer than a step size to the wall
                if self.map_sub_mode:
                    for pixel_center in self.wall_centers:
                        if np.linalg.norm(new_node.val - pixel_center) < self.step_size:
                            wall_collision = True
                            break
                if wall_collision:
                    continue

                min_node.add_child(new_node)
                self.node_list.append(new_node)
                self.publish_markers()
        if not completed:
            self.get_logger().info('Path not found')
            return

    def timer_callback(self):
        """
        Timer callback for publishing the markers and path
        """
        self.publish_markers()
        self.publish_path()

    def occupancy_grid_callback(self, msg):
        """
        Callback for the OccupancyGrid subscriber.

        Parameters:
        - msg (OccupancyGrid): The received OccupancyGrid message.
        """
        self.map_data = msg.data
        self.map_size = np.array([msg.info.width, msg.info.height])
        self.map_resolution = msg.info.resolution
        self.map_origin = np.array([msg.info.origin.position.x,
                                    msg.info.origin.position.y])
        self.map_matrix = np.array(self.map_data).reshape(
            (msg.info.height, msg.info.width))
        self.pixel_centers = np.array([np.array([i, j]) for i in range(
            msg.info.width) for j in range(msg.info.height)])
        self.pixel_centers = self.pixel_centers * self.map_resolution + self.map_origin
        self.pixel_centers = self.pixel_centers.reshape(
            self.map_size[0], self.map_size[1], 2)
        self.wall_indices = np.where(self.map_matrix >= self.wall_confidence)
        self.wall_centers = self.pixel_centers[self.wall_indices]

    def create_marker(self, marker_type: int, marker_id: int, color: list, scale: list, position: list) -> Marker:
        """
        Creates a marker for visualization.

        Parameters
        ----------
        marker_type : int
            The type of marker
        marker_id : int
            The ID of the marker
        color : list
            The color of the marker
        scale : list
            The scale of the marker
        position : list
            The position of the marker
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "rrt_markers"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        return marker

    def publish_markers(self):
        """
        Publishes the markers
        """
        marker_publisher = self.create_publisher(
            MarkerArray, 'rrt_markers', 10)
        marker_array = MarkerArray()
        for node in self.node_list:
            marker = self.create_marker(Marker.SPHERE, self.node_list.index(node) + 2, [
                0.0, 1.0, 0.0, 1.0], [0.1, 0.1, 0.1], [node.val[0], node.val[1], 0.0])
            marker_array.markers.append(marker)
        for obstacle in self.obstacle_list:
            if isinstance(obstacle, Circle):
                marker = self.create_marker(Marker.CYLINDER, self.obstacle_list.index(
                    obstacle) + len(self.node_list) + 2, [1.0, 0.0, 0.0, 1.0], [obstacle.radius * 2, obstacle.radius * 2, 0.1], [obstacle.x, obstacle.y, 0.0])
                marker_array.markers.append(marker)
            elif isinstance(obstacle, Rectangle):
                marker = self.create_marker(Marker.CUBE, self.obstacle_list.index(
                    obstacle) + len(self.node_list) + 2, [1.0, 0.0, 0.0, 1.0], [obstacle.width, obstacle.height, 0.1], [obstacle.x, obstacle.y, 0.0])
                marker_array.markers.append(marker)
        marker = self.create_marker(Marker.SPHERE, 0, [
            1.0, 0.0, 0.0, 1.0], [0.2, 0.2, 0.2], [self.start_position[0], self.start_position[1], 0.0])
        marker_array.markers.append(marker)
        marker = self.create_marker(Marker.SPHERE, 1, [
            0.0, 0.0, 1.0, 1.0], [0.2, 0.2, 0.2], [self.goal_position[0], self.goal_position[1], 0.0])
        marker_array.markers.append(marker)
        marker_publisher.publish(marker_array)

    def set_start_goal(self, start, goal):
        start_x, start_y = start
        goal_x, goal_y = goal
        if not (-self.map_size[0] < start_x < self.map_size[0]):
            raise ValueError("Start is out of map bounds.")
        if not (-self.map_size[1] < start_y < self.map_size[1]):
            raise ValueError("Start is out of map bounds.")
        if not (-self.map_size[0] < goal_x < self.map_size[0]):
            raise ValueError("Goal is out of map bounds.")
        if not (-self.map_size[1] < goal_y < self.map_size[1]):
            raise ValueError("Goal is out of map bounds.")
        for obstacle in self.obstacle_list:
            if isinstance(obstacle, Circle):
                obstacle_vec = np.array(
                    [start_x - obstacle.x, start_y - obstacle.y])
                obstacle_distance = np.linalg.norm(obstacle_vec)
                if obstacle_distance < obstacle.radius:
                    raise ValueError("Start is inside an obstacle.")
            elif isinstance(obstacle, Rectangle):
                if (obstacle.x - obstacle.width/2 < start_x < obstacle.x + obstacle.width/2) and (obstacle.y - obstacle.height/2 < start_y < obstacle.y + obstacle.height/2):
                    raise ValueError("Start is inside an obstacle.")
        for pixel_center in self.wall_centers:
            if np.linalg.norm(np.array([start_x, start_y]) - pixel_center) < self.step_size:
                raise ValueError("Start is too close to a wall.")
            if np.linalg.norm(np.array([goal_x, goal_y]) - pixel_center) < self.step_size:
                raise ValueError("Goal is too close to a wall.")

    def publish_path(self):
        """
        Publishes the path
        """
        path_publisher = self.create_publisher(Path, 'rrt_path', 10)
        path = Path()
        path.header.frame_id = "map"
        current_node = self.node_list[-1]
        while current_node.parent:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = current_node.val[0]
            pose.pose.position.y = current_node.val[1]
            pose.pose.position.z = 0.0
            path.poses.append(pose)
            current_node = current_node.parent
        path_publisher.publish(path)


def main(args=None):
    rclpy.init(args=args)
    rrt_2d_node = RRT2DNode()
    rclpy.spin(rrt_2d_node)
    rrt_2d_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
