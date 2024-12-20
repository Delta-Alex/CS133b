'''sokoban_forward.py

   Solve Sokoban.  This is the skeleton code, running only the
   (regular) forward search order.

   PLEASE FINISH BY WRITING THE transition() function for the Node
   class (marked by FIXME).

   The overall cost is based solely on the number of steps.  And the
   crazy (back-and-forth) nature of the problems means we don't have
   an effective cost-to-go estimate.  So we use a simple Dijkstra
   algorithm with a simple FIFO on-deck queue.

   Note the forward vs. backward speed depends a lot on the particular
   maze.  But for most challenging makes backwards works much better,
   as much as 10x fewer nodes tested:

   Grid  Solution         Nodes Created searching           Possible
          Steps         Forward            Backward           Nodes
    01       4          23 ( 4.646%)       31 ( 6.263%)          495
    02  no solution     25 ( 1.225%)       24 ( 1.176%)         2040
    03      11          74 (16.017%)       77 (16.667%)          462
    04      78        5450 (78.986%)     1470 (21.304%)         6900
    05         TO BE DETERMINED          5316 ( 7.573%)        70200
    06     105      200936 (61.093%)    15438 ( 4.694%)       328900
    07     155      340868 (69.052%)    69864 (14.153%)       493640
    08     126      119144 (26.611%)    35357 ( 7.897%)       447720
    09      34      638005 ( 1.363%)    75031 ( 0.160%)     46823400
    10     138      865250 (85.934%)    53785 ( 5.342%)      1006880
    11     177    13965580 (44.371%)   531655 ( 1.677%)     31474716

'''

import math

from visualgrid import VisualGrid


######################################################################
#
#  Define the grids:
#
#    # = wall
#    R = robot (starting position)
#    x = box (starting position)
#    o = target (box goal position)
#    * = "both x and o", a box starting on a goal position
#
grid01 = ['######',
          '#    #',
          '##Rx #',
          '# * o#',
          '######']

grid02 = ['#########',
          '# R# o* #',
          '#  # #  #',
          '#x   #  #',
          '#########']

grid03 = ['      #####',
          '#######o# #',
          '# R   x   #',
          '# # # # ###',
          '#       #  ',
          '#########  ']

grid04 = ['#########',
          '###    ##',
          '### ##R##',
          '### # x #',
          '# oo# x #',
          '#       #',
          '#  ######',
          '#########']

grid05 = ['###########',
          '#######o  #',
          '#######o# #',
          '#######o# #',
          '# R x x x #',
          '# # # # ###',
          '#       ###',
          '###########']

grid06 = ['    #### ',
          '  ###  ##',
          ' ## x   #',
          '## x  # #',
          '# R#xx  #',
          '# oo  ###',
          '# oo###  ',
          '#####    ']

grid07 = ['###########',
          '########  #',
          '## x      #',
          '##   x x  #',
          '## ########',
          '## # o  ###',
          '#  # #  ###',
          '#  R o ####',
          '## # # ####',
          '##   o ####',
          '###########']

grid08 = ['  ####         ',
          '  #  #    #####',
          '  #  #    #   #',
          '  #  ######o# #',
          '####  x    o  #',
          '#   xx# ###o# #',
          '#   #   # #   #',
          '######### #R ##',
          '          #  # ',
          '          #### ']

grid09 = ['########',
          '###   ##',
          '#oRx  ##',
          '### xo##',
          '#o##x ##',
          '# # o ##',
          '#x *xxo#',
          '#   o  #',
          '########']

grid10 = ['  ###### ',
          '  #    # ',
          '  #  x # ',
          ' ####x # ',
          '## x x # ',
          '#oooo# ##',
          '#     R #',
          '##  #   #',
          ' ########']

grid11 = ['  #######',
          '# #ddddd#',
          '# # # # #',
          '  # R x #',
          '### ### #',
          '#d  ### #',
          '#dx d##o#',
          '##dx d#o#',
          ' ##dx  o#',
          '# ##dx#o#',
          '## ## #o#',
          '### #ddd#',
          '### #####']

# Better reverse
grid12 = ['##################',
          '#    # # # o #   #',
          '# R  # #o#   #   #',
          '#    # # #   #   #',
          '#    # # #   #   #',
          '#  x             #',
          '#  x             #',
          '#      #   #######',
          '#      #         #',
          '##################']

# Better forward
grid13 = ['#################',
          '#    # # #  #   #',
          '# R  # # # x#   #',
          '#    # # #  #   #',
          '#    # # #  #   #',
          '#               #',
          '#  o            #',
          '#      #  #######',
          '#      #        #',
          '#################']

#
#   CONFIGURATION: Select the GRID, TESTING, and FORWARD/BACKWARD.
#

# Choose the grid
grid = grid05

# Choose whether to try the testing.
TESTING = True

# Choose the search tree growth direction.
FORWARD = True

#
#   Colors
#
WHITE  = [1.00, 1.00, 1.00]
BLACK  = [0.00, 0.00, 0.00]
RED    = [1.00, 0.00, 0.00]
BROWN  = [0.59, 0.29, 0.00]


######################################################################
#
#   General SPACE and MAZE Class
#
#   Define
#    a) Individual spaces, which have a row and col.  These are
#       sortable by row, then column.  The also show the adjacents
#       spaces (RIGHT, UP, LEFT, DOWN) which are either None or
#       another valid space.  And a list of all valid neighbors
#       (adjacent spaces).
#
#    b) The entire maze, which is a collection of the spaces, computed
#       from the above grids.  This also extracts the special spaces
#       (robot, boxes, targets) and provides visualization.
#
# Define the directions (just to be more verbose/clear).
class Direction:
    RIGHT = 0
    UP    = 1
    LEFT  = 2
    DOWN  = 3
    DIRECTIONS = (  RIGHT,      UP,    LEFT,    DOWN)
    OPPOSITES  = (   LEFT,    DOWN,   RIGHT,      UP)
    DELTAS     = ((0,  1), (-1, 0), (0, -1), ( 1, 0))

# Define the individual spaces.  Note (i) the class defines a
# less-than operator, so spaces can be sorted (row, then column).  And
# (ii) the class provides the neighbors both by direction (important
# when pushing) or as a general list.
class Space:
    def __init__(self, r, c):
        # Define the variables.
        self.r = r              # Row
        self.c = c              # Column

        # The adjacent/neighbors by direction and as a list.
        self.adjacent  = [None] * len(Direction.DIRECTIONS)
        self.neighbors = []

    # Define the "less-than" to enable sorting by (row,col).
    def __lt__(self, other):
        return ((self.r < other.r) or
                ((self.r == other.r) and (self.c < other.c)))

    # Print (for debugging).
    def __repr__(self):
        return("(%2d,%2d)" % (self.r, self.c))

# Define the maze, being the collection of spaces, including the
# neighbor connections.  This also identifies the special spaces and
# provides a visualization.
class Maze:
    def __init__(self, grid):
        # Check the size of the grid (maze).
        rows = len(grid)
        cols = len(grid[0])
        for line in grid:
            assert len(line) == cols, "Inconsistent lines in grid (maze)"

        # Create the visual.
        self.visual = VisualGrid(rows, cols)

        # Create a sorted list of legal (non-wall) spaces.  Else mark black.
        self.spaces = []
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] == '#':
                    self.visual.color(r, c, BLACK)
                else:
                    self.spaces.append(Space(r,c))
        self.spaces.sort()      # To be safe: they should already be sorted.

        # Connect the spaces.
        for space in self.spaces:
            for (i, (dr, dc)) in enumerate(Direction.DELTAS):
                matches = [s for s in self.spaces
                           if (s.r,s.c) == (space.r+dr,space.c+dc)] + [None]
                space.adjacent[i] = matches[0]
            space.neighbors = [a for a in space.adjacent if a is not None]

        # Pull out the robot, boxes, and targets (in sorted order!).
        robots  = [s for s in self.spaces if grid[s.r][s.c] in 'Rr']
        boxes   = [s for s in self.spaces if grid[s.r][s.c] in 'Xx*']
        targets = [s for s in self.spaces if grid[s.r][s.c] in 'Oo*']
        assert len(robots) == 1,            "Must have exactly one robot"
        assert len(boxes)   > 0,            "Must have some boxes"
        assert len(boxes)  == len(targets), "Unequal num boxes and targets"
        self.robot   = robots[0]
        self.boxes   = boxes
        self.targets = targets

        # Add the targets to the visual.
        for target in self.targets:
            self.visual.write(target.r, target.c, 'o')
        self.show(wait=0.1)

    # Show the visual.
    def show(self, robot=None, boxes=None, wait=False):
        # Grab default values if not specified.
        if not robot: robot = self.robot
        if not boxes: boxes = self.boxes
        # Mark the robot as red, boxes as brown.
        for s in [robot]: self.visual.color(s.r, s.c, RED)
        for s in  boxes : self.visual.color(s.r, s.c, BROWN)
        # Show
        self.visual.show(wait)
        # Clear the colors.
        for s in [robot]: self.visual.color(s.r, s.c, WHITE)
        for s in  boxes : self.visual.color(s.r, s.c, WHITE)

    # Print (for debugging).
    def __str__(self):
        return(f"Robot:   {self.robot}\r\n" +
               f"Boxes:   {self.boxes}\r\n" +
               f"Targets: {self.targets}")
    def __repr__(self):
        s = str(self)
        for space in self.spaces:
            s += (f"r\n<Space {space} -> [%7s,%7s,%7s,%7s]>"
                  % tuple('' if a is None else str(a) for a in space.adjacent))
        return(s)


######################################################################
#
#   Planning Algorithm
#
#   Define Node class (to contain the state as well as the search
#   tree), as well as the actual algorithm.
#

# State Transition Function (moving forward in time): Given the space
# occupied by the robot together with the list of spaces occupied by
# the boxes, as well as a direction of movement, return either None if
# the movement is not possible or the new robot space and new list of
# boxes.
def transition(robot, boxes, direction):
    
    newrobot = robot.adjacent[direction]
    # if there is a wall in the way
    if newrobot is None:
        return None

    # no boxes in the way
    if newrobot not in boxes:
        return (newrobot, boxes)

    else:
        oldbox = newrobot
        # check if box can be pushed
        newbox  = oldbox.adjacent[direction]

        # there is a wall
        if newbox is None:
            return None

        # there is a box in the way
        if newbox in boxes:
            return None

        else:
            boxes.remove(oldbox)
            boxes.append(newbox)
            return (newrobot, boxes)


    #return None
    #or
    #return (newrobot, newboxes)


# Node class. This retains the state and the search tree data.  Note,
# as we are instantiating the node as we build the tree, we already
# know the parent and can set the cost at instantiation.  Otherwise,
# this contains the children() function to determine, instantiate, and
# return the possible child nodes (states).
class Node:
    def __init__(self, robot, boxes, parent):
        # Make sure the list of boxes is sorted!  Note, which box goes
        # to which target is irrelevant, so we always keep the list of
        # boxes and targets ordered.  This avoids duplicate states and
        # makes is easy to compare boxes and targets!
        boxes.sort()

        # Save the state
        self.state = tuple([robot] + boxes)

        # Save the individual elements to compute the children.
        self.robot = robot      # Single space occupied by robot
        self.boxes = boxes      # Sorted list of spaces with boxes

        # Set the parent, together with the number of steps.
        self.parent = parent    # Link to parent node
        if parent is None:      # Compute the cost coming from the parent.
            self.cost = 0
        else:
            self.cost = parent.cost + 1

    # Compute the children, i.e. the nodes/states that can be reached
    # by one of the four actions (movement directions).  Return a list
    # of these nodes which must consider what happens to the boxes.
    def children(self):
        # Build up the list of children, trying in each direction.
        children = []
        for direction in Direction.DIRECTIONS:
            # Try the movement (copy the boxes list to isolate changes).
            result = transition(self.robot, self.boxes.copy(), direction)
            if result is not None:
                children.append(Node(result[0], result[1], self))
        return children

    # Print (for debugging).
    def __repr__(self):
        s = "<Node R %s, B" % str(self.robot)
        for b in self.boxes:
            s += " %s" % str(b)
        s += " with Cost %d>" % self.cost
        return s


#
#   Search/Planner Algorithm
#
def planner(initrobot, initboxes, targets):
    ####################  CHECK DIFFICULTY  ####################
    # Compute the number of possible states/nodes.  Count the internal
    # spaces, reachable by the robot.  Compute the combinations
    internalspaces = [initrobot]
    for space in internalspaces:
        for n in space.neighbors:
            if n not in internalspaces:
                internalspaces.append(n)
    L = len(internalspaces)
    B = len(initboxes)
    Nmax = L * math.comb(L-1, B)
    print("# of internal spaces %d" % L)
    print("# of boxes  %d"          % B)
    print("# of possible states %d" % Nmax)


    ####################  INITIALIZE  ####################
    # Track the nodes created (seen) as a dictionary indexed by state.
    nodes = {}

    # Set up the starting on-deck queue, with zero cost (parent = None)
    onDeck = []
    onDeck.append(Node(initrobot, initboxes, None))

    # Record all nodes on this starting on-deck queue as seen/existing.
    for node in onDeck:
        nodes[node.state] = node


    ####################  LOOP  ####################
    # Define a report function.
    def report(text, cost):
        Nseen = len(nodes)
        Ndeck = len(onDeck)
        print(f"{text} nodes {Nseen:7d} ({100*Nseen/Nmax:6.3f}%), " +
              f"done {Ndone:7d} ({100*Ndone/Nmax:6.3f}%), " +
              f"on-deck {Ndeck:6d}, cost {cost:3d}")

    # Continually expand/build the search tree.
    Ndone = 0
    while True:
        # Show the current status:
        if Ndone % 10000 == 0:
            report("So far", onDeck[0].cost)

        # Grab the next node (first on deck).
        node = onDeck.pop(0)

        # Check whether we have reached the end.
        if (node.boxes == targets):
            break

        # Process the children (save and add on-deck queue as needed).
        children = node.children()
        for child in children:
            # Add the child only if not already seen.
            if nodes.get(child.state, None) is None:
                onDeck.append(child)
                nodes[child.state] = child

        # Count the fully processed nodes.
        Ndone += 1

        # Also make sure we still have something to look at!
        if not (len(onDeck) > 0):
            report("FAILED", node.cost)
            return (None, nodes.values())


    ####################  BUILD PATH  ##############################
    # Report the final metrics.
    report("At END", node.cost)

    # Build and return the path.
    path = [node]
    while node.parent is not None:
        node = node.parent
        path.insert(0, node)
    return (path, nodes.values())


######################################################################
#
#  Main Code
#
if __name__== "__main__":

    # Test the Node class.
    if TESTING:
        print("Testing results:")
        testmaze = Maze(grid01)
        testnode = Node(testmaze.robot, testmaze.boxes, None)
        for node in [testnode] + testnode.children():
            print(node, end=' ')
            testmaze.show(robot=node.robot, boxes=node.boxes, wait=' ')
        print("Regular results:")

    # Create/show the maze.
    maze = Maze(grid)
    print(maze)

    # Run the planner.
    (path, nodes) = planner(maze.robot, maze.boxes, maze.targets)

    # Show the steps.
    if not path:
        print("UNABLE TO FIND A PATH")
        input("Hit return to show all nodes")
        for node in nodes:
            print(node, end=' ')
            maze.show(robot=node.robot, boxes=node.boxes, wait=' ')
    else:
        print("Found path with %d steps" % (len(path)-1))
        input("Hit return to show path")
        for node in path:
            print(node)
            maze.show(robot=node.robot, boxes=node.boxes, wait=0.1)
