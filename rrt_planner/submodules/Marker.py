from visualization_msgs.msg import Marker


class Circle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius


class Rectangle:
    def __init__(self, x, y, width, height, angle):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.angle = angle


class Sphere:
    def __init__(self, x, y, z, radius):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius


class Box:
    def __init__(self, x, y, z, width, height, depth, angle):
        self.x = x
        self.y = y
        self.z = z
        self.width = width
        self.height = height
        self.depth = depth
        self.angle = angle


def create_marker(marker_type: int, marker_id: int, color: list,
                  scale: list, position: list) -> Marker:
    """
    Create a marker for visualization.

    Arguments:
    ---------
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
