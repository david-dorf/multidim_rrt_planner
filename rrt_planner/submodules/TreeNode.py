class TreeNode:
    """
    Tree structure node. Not to be confused with a ROS node.

    Attributes:
    val (ndarray): Location of the node.
    parent (Node): Parent node of this node.
    children (list of Node): Children nodes of this node.
    """

    def __init__(self, val, parent) -> None:
        self.val = val
        self.parent = parent
        self.children = []

    def add_child(self, child):
        self.children.append(child)
