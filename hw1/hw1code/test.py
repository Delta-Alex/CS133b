class Node:
    # Initialization
    def __init__(self, row, col):
        # Save the matching state.
        self.row = row
        self.col = col

        # Clear the list of neighbors (used for the full graph).
        self.neighbors = []

        # Clear the parent (used for the search tree), as well as the
        # actual cost to reach (via the parent).
        # FIXME: You may want to add further information as needed here.
        self.parent = None      # No parent
        self.cost   = 500       # Unable to reach = infinite cost

        # State of the node during the search algorithm.
        self.seen = False
        self.done = False


l = []
a = Node(1,1)
b = Node(2,1)
l.append(a)
l.append(b)
print(l)
l.remove(a)
print(l)