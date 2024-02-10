import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Empty
import numpy as np
from .TreeNode import TreeNode
from .Marker import Sphere, Box, create_marker


class RRT3DNode(Node):
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
    goal_z : float
        The z coordinate of the goal position.
    map_bounds_x : float
        The x boundary of the map that contains the RRT.
    map_bounds_y : float
        The y boundary of the map.
    map_bounds_z : float
        The z boundary of the map that contains the RRT.
    obstacle_sub_mode : bool
        Whether to subscribe to obstacle data or not.
    step_size : float
        The step size for the RRT.
    node_limit : int
        The maximum number of nodes to generate.
    goal_tolerance : float
        The tolerance for the goal position.

    Attributes
    ----------
    start_position : numpy.ndarray
        The start position.
    goal_position : numpy.ndarray
        The goal position.
    map_size : numpy.ndarray
        The boundaries containing the RRT.
    obstacle_list : list
        The list of obstacles.
    start_node : TreeNode
        The start node.
    node_list : list
        The list of nodes.
    marker_publisher : Publisher
        The ROS publisher for the markers.
    path_publisher : Publisher

    Methods
    -------
    run_rrt_3D()
        Generates the RRT.
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
        super().__init__('rrt_3d_node')
        parameters = [
            ('start_x', 0.0),
            ('start_y', 0.0),
            ('start_z', 0.0),
            ('goal_x', 1.0),
            ('goal_y', -1.0),
            ('goal_z', 1.0),
            ('map_bounds_x', 100.0),
            ('map_bounds_y', 100.0),
            ('map_bounds_z', 100.0),
            ('obstacle_sub_mode', False),
            ('step_size', 0.05),
            ('node_limit', 5000),
            ('goal_tolerance', 0.5),
        ]
        self.declare_parameters(namespace='', parameters=parameters)
        for param, _ in parameters:
            setattr(self, param, self.get_parameter(param).value)
        self.start_position = np.array(
            [self.start_x, self.start_y, self.start_z])
        self.goal_position = np.array([self.goal_x, self.goal_y, self.goal_z])
        self.marker_publisher = self.create_publisher(
            MarkerArray, 'rrt_markers', 10)
        self.path_publisher = self.create_publisher(Path, 'rrt_path', 10)
        self.map_size = np.array(
            [self.map_bounds_x, self.map_bounds_y, self.map_bounds_z])
        if self.obstacle_sub_mode:
            self.obstacle_list = []
            self.obstacle_subscription = self.create_subscription(
                MarkerArray, 'obstacle_markers_3D', self.obstacle_callback, 10)
        self.rrt_srv = self.create_service(Empty, 'run_rrt', self.rrt_callback)
        self.start_node = TreeNode(self.start_position, None)
        self.node_list = [self.start_node]
        self.marker_array = MarkerArray()
        self.path = Path()

    def rrt_callback(self, request, response):
        """Service callback for running the RRT."""
        # Clear the markers and path
        self.marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        self.marker_array.markers.append(clear_marker)
        self.marker_publisher.publish(self.marker_array)
        self.path = Path()
        self.path.header.frame_id = "map"
        self.path_publisher.publish(self.path)

        # Publish the start and goal markers
        self.marker_array.markers.append(create_marker(Marker.SPHERE, 0, [
            1.0, 0.0, 0.0, 1.0], [0.2, 0.2, 0.2], [self.start_position[0], self.start_position[1],
                                                   self.start_position[2]]))
        self.marker_array.markers.append(create_marker(Marker.SPHERE, 1, [
            0.0, 0.0, 1.0, 1.0], [0.2, 0.2, 0.2], [self.goal_position[0], self.goal_position[1],
                                                   self.goal_position[2]]))

        # Run the RRT
        self.node_list = [self.start_node]
        self.run_rrt_3D()
        return response

    def run_rrt_3D(self):
        """Generate the RRT in 3D."""
        self.get_logger().info('Generating RRT...')
        if self.obstacle_sub_mode:
            while not self.obstacle_list:
                self.get_logger().info('Waiting for obstacle data...')
                rclpy.spin_once(self)
            self.get_logger().info('Obstacle data received')
        while len(self.node_list) < self.node_limit:
            random_position_x = np.random.uniform(
                -(self.map_size[0] * self.step_size)/2, (self.map_size[0] * self.step_size)/2)
            random_position_y = np.random.uniform(
                -(self.map_size[1] * self.step_size)/2, (self.map_size[1] * self.step_size)/2)
            random_position_z = np.random.uniform(
                -(self.map_size[2] * self.step_size)/2, (self.map_size[2] * self.step_size)/2)
            random_position = np.array(
                [random_position_x, random_position_y, random_position_z])
            min_distance = np.inf
            min_node = None
            for node in self.node_list:
                goal_vec = self.goal_position - node.val
                goal_distance = np.linalg.norm(goal_vec)
                goal_node = TreeNode(self.goal_position, node)
                if goal_distance < self.goal_tolerance:
                    node.add_child(goal_node)
                    self.node_list.append(goal_node)
                    self.get_logger().info('Path found')
                    self.publish_path()
                    return
                new_node_vec = random_position - node.val  # Find node closest to random point
                distance = np.linalg.norm(new_node_vec)
                if distance < min_distance:
                    min_node_vec = new_node_vec
                    min_node = node
                min_distance = np.min([distance, min_distance])
            if min_distance != 0:
                new_node_unit_vec = min_node_vec / min_distance
                new_node_val = min_node.val + new_node_unit_vec * self.step_size
                new_node = TreeNode(new_node_val, min_node)
                obstacle_collision = False  # Check if new_node collides with any obstacles
                if self.obstacle_sub_mode:
                    if self.obstacle_list:
                        for obstacle in self.obstacle_list:
                            if isinstance(obstacle, Sphere):
                                obstacle_vec = new_node.val - \
                                    np.array(
                                        [obstacle.x, obstacle.y, obstacle.z])
                                obstacle_distance = np.linalg.norm(
                                    obstacle_vec)
                                if obstacle_distance < obstacle.radius:
                                    obstacle_collision = True
                                    break
                            elif isinstance(obstacle, Box):
                                if (obstacle.x - obstacle.width/2 < new_node.val[0]
                                    < obstacle.x + obstacle.width/2) and \
                                    (obstacle.y - obstacle.height/2 < new_node.val[1]
                                     < obstacle.y + obstacle.height/2) and \
                                    (obstacle.z - obstacle.depth/2 < new_node.val[2]
                                     < obstacle.z + obstacle.depth/2):
                                    obstacle_collision = True
                                    break
                        if obstacle_collision:
                            continue
                min_node.add_child(new_node)
                self.node_list.append(new_node)
                marker = create_marker(Marker.SPHERE, self.node_list.index(new_node) + 2, [
                    0.0, 1.0, 0.0, 1.0], [0.1, 0.1, 0.1], [new_node.val[0], new_node.val[1],
                                                           new_node.val[2]])
                self.marker_array.markers.append(marker)
                self.marker_publisher.publish(self.marker_array)
        self.get_logger().info('Path not found')
        return

    def obstacle_callback(self, msg):
        """
        Run callback for the obstacle subscriber.

        Arguments:
        ---------
        msg : MarkerArray
            The obstacle marker data.

        """
        self.obstacle_list = []
        for marker in msg.markers:
            if marker.type == Marker.SPHERE:
                self.obstacle_list.append(Sphere(
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
                    marker.scale.x/2))
            elif marker.type == Marker.CUBE:
                self.obstacle_list.append(Box(
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z,
                    marker.scale.x, marker.scale.y, marker.scale.z, marker.pose.orientation.x))

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
        start_x, start_y, start_z = start
        goal_x, goal_y, goal_z = goal
        if not (-self.map_size[0] < start_x < self.map_size[0]):
            raise ValueError("Start is out of map bounds.")
        if not (-self.map_size[1] < start_y < self.map_size[1]):
            raise ValueError("Start is out of map bounds.")
        if not (-self.map_size[2] < start_z < self.map_size[2]):
            raise ValueError("Start is out of map bounds.")
        if not (-self.map_size[0] < goal_x < self.map_size[0]):
            raise ValueError("Goal is out of map bounds.")
        if not (-self.map_size[1] < goal_y < self.map_size[1]):
            raise ValueError("Goal is out of map bounds.")
        if not (-self.map_size[2] < goal_z < self.map_size[2]):
            raise ValueError("Goal is out of map bounds.")
        for obstacle in self.obstacle_list:
            if isinstance(obstacle, Sphere):
                obstacle_vec = np.array([start_x, start_y, start_z]) - \
                    np.array([obstacle.x, obstacle.y, obstacle.z])
                obstacle_distance = np.linalg.norm(obstacle_vec)
                if obstacle_distance < obstacle.radius:
                    raise ValueError("Start is inside an obstacle.")
            elif isinstance(obstacle, Box):
                if (obstacle.x-obstacle.width/2 < start_x < obstacle.x+obstacle.width/2 and
                        obstacle.y-obstacle.height/2 < start_y < obstacle.y+obstacle.height/2 and
                        obstacle.z-obstacle.depth/2 < start_z < obstacle.z+obstacle.depth/2):
                    raise ValueError("Start is inside an obstacle.")

    def publish_path(self):
        """Publish the path as a Path message in 3D."""
        self.path = Path()
        self.path.header.frame_id = "map"
        current_node = self.node_list[-1]
        while current_node.parent:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = current_node.val[0]
            pose.pose.position.y = current_node.val[1]
            pose.pose.position.z = current_node.val[2]
            self.path.poses.append(pose)
            current_node = current_node.parent
        self.path_publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    rrt_3d_node = RRT3DNode()
    rclpy.spin(rrt_3d_node)
    rrt_3d_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
