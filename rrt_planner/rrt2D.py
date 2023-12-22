import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
import numpy as np
from .submodules.TreeNode import TreeNode
from .submodules.Marker import Circle, Rectangle, create_marker


class RRT2DNode(Node):
    """
    A ROS2 node for generating a 2D RRT.

    Parameters
    ----------
    start_x : float
        The x coordinate of the start position.
    start_y : float
        The y coordinate of the start position.
    goal_x : float
        The x coordinate of the goal position.
    goal_y : float
        The y coordinate of the goal position.
    map_sub_mode : bool
        Whether or not to subscribe to the occupancy grid.
    obstacle_sub_mode : bool
        Whether or not to subscribe to the obstacle markers.
    step_size : float
        The step size for each node.
    node_limit : int
        The maximum number of nodes to generate.
    goal_tolerance : float
        The tolerance for reaching the goal.
    wall_confidence : int
        The confidence threshold for the occupancy grid.

    Attributes
    ----------
    start_position : numpy.ndarray
        The start position.
    goal_position : numpy.ndarray
        The goal position.
    map_data : list
        The map data.
    map_size : numpy.ndarray
        The size of the map.
    map_resolution : float
        The resolution of the map.
    map_origin : numpy.ndarray
        The origin of the map.
    map_matrix : numpy.ndarray
        A matrix representation of the map.
    pixel_centers : numpy.ndarray
        The centers of each pixel in the map.
    wall_indices : tuple
        The indices of the wall pixels in the map matrix.
    wall_centers : numpy.ndarray
        The center coordinates of the wall pixels.
    start_node : TreeNode
        The start node.
    node_list : list
        The list of nodes.
    timer : Timer
        The ROS timer for publishing markers and the path.
    marker_publisher : Publisher
        The ROS publisher for the markers.

    Methods
    -------
    run_rrt_2D()
        Generates the RRT.
    timer_callback()
        Timer callback for publishing the final markers and path until the node is destroyed.
    occupancy_grid_callback(msg)
        Callback for the OccupancyGrid subscriber.
    obstacle_callback(msg)
        Callback for the obstacle subscriber.
    publish_markers()
        Publishes a marker for each node, start, and goal in the RRT.
    set_start_goal(start, goal)
        Sets the start and goal positions.
    publish_path()
        Publishes the path.

    """

    def __init__(self):
        super().__init__('rrt_2d_node')
        parameters = [
            ('start_x', 0.0),
            ('start_y', 0.0),
            ('goal_x', 1.0),
            ('goal_y', -1.0),
            ('map_sub_mode', True),
            ('obstacle_sub_mode', True),
            ('step_size', 0.05),
            ('node_limit', 5000),
            ('goal_tolerance', 0.2),
            ('wall_confidence', 50)
        ]
        self.declare_parameters(namespace='', parameters=parameters)
        for param, _ in parameters:
            setattr(self, param, self.get_parameter(param).value)
        self.start_position = np.array([self.start_x, self.start_y])
        self.goal_position = np.array([self.goal_x, self.goal_y])
        self.marker_publisher = self.create_publisher(
            MarkerArray, 'rrt_markers', 10)
        self.path_publisher = self.create_publisher(Path, 'rrt_path', 10)
        if self.map_sub_mode:
            self.map_data = None
            self.occupancy_grid_subscription = self.create_subscription(
                OccupancyGrid, 'occupancy_grid', self.occupancy_grid_callback, 10)
        else:
            self.map_size = np.array([100, 100])
            self.map_resolution = 1.0
        if self.obstacle_sub_mode:
            self.obstacle_list = []
            self.obstacle_subscription = self.create_subscription(
                MarkerArray, 'obstacle_markers_2D', self.obstacle_callback, 10)
        self.start_node = TreeNode(self.start_position, None)
        self.node_list = [self.start_node]
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.run_rrt_2D()

    def run_rrt_2D(self):
        """Generate the RRT in 2D."""
        self.get_logger().info('Generating RRT...')
        if self.map_sub_mode:
            while self.map_data is None:
                self.get_logger().info('Waiting for map data...')
                rclpy.spin_once(self)
            self.get_logger().info('Map data received')
        if self.obstacle_sub_mode:
            while not self.obstacle_list:
                self.get_logger().info('Waiting for obstacle data...')
                rclpy.spin_once(self)
            self.get_logger().info('Obstacle data received')
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
                            if (obstacle.x - obstacle.width/2 < new_node.val[0]
                                < obstacle.x + obstacle.width/2) and \
                                (obstacle.y - obstacle.height/2 < new_node.val[1]
                                 < obstacle.y + obstacle.height/2):
                                obstacle_collision = True
                                break
                    if obstacle_collision:
                        continue
                wall_collision = False  # Check if new_node is closer than a step size to the wall
                if self.map_sub_mode:
                    for pixel_center in self.wall_centers:
                        if np.linalg.norm(new_node.val - pixel_center) < self.map_resolution or \
                                np.linalg.norm(new_node.val - pixel_center) < self.step_size:
                            wall_collision = True
                            break
                if wall_collision:
                    continue

                min_node.add_child(new_node)
                self.node_list.append(new_node)
                self.publish_markers()  # Publish markers while RRT is running
        if completed:
            self.get_logger().info('Path found')
        else:
            self.get_logger().info('Path not found')
            return

    def timer_callback(self):
        """Timer callback for publishing the final markers and path until the node is destroyed."""
        self.publish_markers()
        self.publish_path()

    def occupancy_grid_callback(self, msg):
        """
        Run callback for the OccupancyGrid subscriber.

        Arguments:
        ---------
        msg : OccupancyGrid
            The received OccupancyGrid message.

        """
        self.map_data = msg.data
        self.map_size = np.array([msg.info.width, msg.info.height])
        self.map_resolution = msg.info.resolution
        self.map_origin = np.array([msg.info.origin.position.x,
                                    msg.info.origin.position.y])
        self.map_matrix = np.array(self.map_data).reshape(
            (msg.info.height, msg.info.width)).T
        self.pixel_centers = np.array([np.array([i, j]) for i in range(
            msg.info.width) for j in range(msg.info.height)])
        self.pixel_centers = self.pixel_centers * self.map_resolution + self.map_origin
        self.pixel_centers = self.pixel_centers.reshape(
            self.map_size[0], self.map_size[1], 2)
        self.wall_indices = np.where(self.map_matrix >= self.wall_confidence)
        self.wall_centers = self.pixel_centers[self.wall_indices]

    def obstacle_callback(self, msg):
        """
        Run callback for the obstacle subscriber.

        Arguments:
        ---------
        msg : MarkerArray
            The received MarkerArray message.

        """
        self.obstacle_list = []
        for marker in msg.markers:
            if marker.type == Marker.CYLINDER:
                self.obstacle_list.append(Circle(
                    marker.pose.position.x, marker.pose.position.y, marker.scale.x/2))
            elif marker.type == Marker.CUBE:
                self.obstacle_list.append(Rectangle(
                    marker.pose.position.x, marker.pose.position.y, marker.scale.x, marker.scale.y,
                    marker.pose.orientation.z))

    def publish_markers(self):
        """Publish a marker for each node in the RRT and the start and goal."""
        marker_array = MarkerArray()
        for node in self.node_list:
            marker = create_marker(Marker.SPHERE, self.node_list.index(node) + 2, [
                0.0, 1.0, 0.0, 1.0], [0.1, 0.1, 0.1], [node.val[0], node.val[1], 0.0])
            marker_array.markers.append(marker)
        marker = create_marker(Marker.SPHERE, 0, [
            1.0, 0.0, 0.0, 1.0], [0.2, 0.2, 0.2],
            [self.start_position[0], self.start_position[1], 0.0])
        marker_array.markers.append(marker)
        marker = create_marker(Marker.SPHERE, 1, [
            0.0, 0.0, 1.0, 1.0], [0.2, 0.2, 0.2],
            [self.goal_position[0], self.goal_position[1], 0.0])
        marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)

    def set_start_goal(self, start, goal):
        """
        Set the start and goal positions.

        Arguments:
        ---------
        start : tuple
            The start position
        goal : tuple
            The goal position

        """
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
                if (obstacle.x - obstacle.width/2 < start_x < obstacle.x + obstacle.width/2) and \
                        (obstacle.y - obstacle.height/2 < start_y <
                         obstacle.y + obstacle.height/2):
                    raise ValueError("Start is inside an obstacle.")
        for pixel_center in self.wall_centers:
            if np.linalg.norm(np.array([start_x, start_y]) - pixel_center) < self.step_size:
                raise ValueError("Start is too close to a wall.")
            if np.linalg.norm(np.array([goal_x, goal_y]) - pixel_center) < self.step_size:
                raise ValueError("Goal is too close to a wall.")

    def publish_path(self):
        """Publish the path in 2D."""
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
        self.path_publisher.publish(path)


def main(args=None):
    rclpy.init(args=args)
    rrt_2d_node = RRT2DNode()
    rclpy.spin(rrt_2d_node)
    rrt_2d_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
