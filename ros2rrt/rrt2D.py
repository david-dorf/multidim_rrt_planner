import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import matplotlib.pyplot as plt
from .submodules.TreeNode import TreeNode
from .submodules.Obstacles import Circle, Rectangle


class RRT2DNode(Node):
    def __init__(self):
        super().__init__('rrt_2d_node')
        self.start_position = np.array([0., 0.])
        self.goal_position = np.array([3.2, -4.1])
        self.map_size = np.array([10, 10])
        self.node_limit = 1000
        self.goal_tolerance = 0.5
        self.step_size = 0.2
        self.animate = True

        self.run_rrt_2D()

    def run_rrt_2D(self):
        start_node = TreeNode(self.start_position, None)
        node_list = [start_node]
        completed = False

        while len(node_list) < self.node_limit:
            random_position_x = np.random.randint(
                -self.map_size[0], self.map_size[0])
            random_position_y = np.random.randint(
                -self.map_size[1], self.map_size[1])
            random_position = np.array([random_position_x, random_position_y])
            min_distance = np.inf
            min_node = None
            for node in node_list:
                goal_vec = self.goal_position - node.val
                goal_distance = np.linalg.norm(goal_vec)
                goal_node = TreeNode(self.goal_position, node)
                if goal_distance < self.goal_tolerance:
                    node.add_child(goal_node)
                    node_list.append(goal_node)
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
                min_node.add_child(new_node)
                node_list.append(new_node)

        if not completed:
            print("Goal pose not found before node limit exceeded.")
            return

        self.publish_markers(node_list)
        self.plot_rrt(node_list)

    def publish_markers(self, node_list):
        marker_publisher = self.create_publisher(
            MarkerArray, 'rrt_markers', 10)
        marker_array = MarkerArray()

        # Create markers for the RRT nodes and connections
        for node in node_list:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "rrt_markers"  # Set a unique namespace for each marker
            marker.id = node_list.index(node)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = node.val[0]
            marker.pose.position.y = node.val[1]
            marker.pose.position.z = 0.0  # 2D
            marker_array.markers.append(marker)
        marker_publisher.publish(marker_array)

    def plot_rrt(self, node_list):
        plt.xlim(-self.map_size[0], self.map_size[0])
        plt.ylim(-self.map_size[1], self.map_size[1])
        plt.scatter(self.start_position[0], self.start_position[1], c='r')
        plt.scatter(self.goal_position[0], self.goal_position[1], c='b')
        if self.animate:
            plt.ion()
        for node in node_list:
            if node.parent:
                plt.plot([node.val[0], node.parent.val[0]],
                         [node.val[1], node.parent.val[1]],
                         'g')
                if self.animate:
                    plt.pause(0.05)
        current_node = node_list[-1]
        while current_node.parent:
            if current_node.parent:
                plt.plot([current_node.val[0], current_node.parent.val[0]],
                         [current_node.val[1], current_node.parent.val[1]],
                         c='r')
                if self.animate:
                    plt.pause(0.0001)
            current_node = current_node.parent
        if self.animate:
            plt.ioff()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    rrt_2d_node = RRT2DNode()
    rclpy.spin(rrt_2d_node)
    rrt_2d_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
