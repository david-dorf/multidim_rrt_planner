class TreeNode:
    """
    Tree structure node. Not to be confused with a ROS node.

    Attributes
    ----------
    val : list
        The value of the node
    parent : TreeNode
        The parent of the node
    children : list
        The children of the node

    Methods
    -------
    add_child(child)
        Add a child to the node

    """

    def __init__(self, val, parent) -> None:
        """Initialize the node."""
        self.val = val
        self.parent = parent
        self.children = []

    def add_child(self, child):
        """Add a child to the node."""
        self.children.append(child)
