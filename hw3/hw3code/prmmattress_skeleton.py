#!/usr/bin/env python3
#
#   prmmattress.py
#
#   Use PRM to find a path for the mattress move.
#
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math               import pi, sin, cos, sqrt, ceil
from scipy.spatial      import KDTree
from shapely.geometry   import Point, LineString, MultiLineString, Polygon
from shapely.prepared   import prep

from astar import AStarNode, astar
import vandercorput


######################################################################
#
#   Parameters
#
#   FIXME: Define the N/K...
#
# for part a: N=1100, K =15 -> 82.6 percent
# for part b: N=325, K =15 -> 83.7 percent
# for part b: N=1000, K =25 -> 79.7 percent
N = 325
K = 15

# FIXME: Include the bonus wall for part (c)
# part c: N=325, K = 15 ->85.2 percent
BONUSWALL = False


######################################################################
#
#   World Definitions
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (0, 30)
(ymin, ymax) = (0, 20)

(xA, xB, xC, xD, xE) = ( 5, 12, 15, 18, 21)
(yA, yB, yC, yD)     = ( 5, 10, 12, 15)

xlabels = (xmin, xA, xB, xC, xD, xE, xmax)
ylabels = (ymin, yA, yB, yC, yD,     ymax)

outside = LineString([[xmin, ymin], [xmax, ymin], [xmax, ymax],
                      [xmin, ymax], [xmin, ymin]])
wall1   = LineString([[xmin, yB], [xC, yB]])
wall2   = LineString([[xD, yB], [xmax, yB]])
wall3   = LineString([[xB, yC], [xC, yC], [xC, ymax]])
bonus   = LineString([[xD, yC], [xE, yC]])

# Collect all the wall and prepare (for faster checking).
if BONUSWALL:
    walls = prep(MultiLineString([outside, wall1, wall2, wall3, bonus]))
else:
    walls = prep(MultiLineString([outside, wall1, wall2, wall3]))

# Define the start/goal states (x, y, theta)
(xstart, ystart, tstart) = (xA, yD, pi/2)
(xgoal,  ygoal,  tgoal)  = (xA, yA, 0)

# Define the mattress dimensions.
L = 7           # Length of the mattress
W = 1           # Width of the mattress

# Step size for testing/plotting.  Make sure there is a slight overlap
# in the physical bounding box if you move the coordinates this far.
Dcspace = 0.75


######################################################################
#
#   Utilities: Angle Wrapping and Visualization
#

# Angle Wrap Utility.  Return the angle wrapped into +/- 1/2 of full range.
def wrap(angle, fullrange):
    return angle - fullrange * round(angle/fullrange)

# Visualization Class.
class Visualization:
    def __init__(self):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.grid(True)
        plt.gca().axis('on')
        plt.gca().set_xlim(xmin, xmax)
        plt.gca().set_ylim(ymin, ymax)
        plt.gca().set_xticks(xlabels)
        plt.gca().set_yticks(ylabels)
        plt.gca().set_aspect('equal')

        # Show the walls.
        for l in walls.context.geoms:
            plt.plot(*l.xy, 'k', linewidth=2)
        if bonus in walls.context.geoms:
            plt.plot(*bonus.xy, 'b:', linewidth=3)

        # Show.
        self.show()

    def show(self, text = ''):
        # Show the plot.
        plt.pause(0.001)
        # If text is specified, print and wait for confirmation.
        if len(text)>0:
            input(text + ' (hit return to continue)')

    def drawNode(self, node, *args, **kwargs):
        plt.plot(*node.mattress.exterior.xy, *args, **kwargs)

    def drawEdge(self, head, tail, *args, **kwargs):
        plt.plot([head.x, tail.x], [head.y, tail.y], *args, **kwargs)

    def drawPath(self, path, *args, **kwargs):
        for i in range(len(path)-1):
            n = ceil(path[i].distance(path[i+1]) / Dcspace)
            for j in range(n):
                node = path[i].intermediate(path[i+1], j/n)
                self.drawNode(node, *args, **kwargs)
        self.drawNode(path[-1], *args, **kwargs)


######################################################################
#
#   Node Definition
#
class Node(AStarNode):
    def __init__(self, x, y, theta):
        # Setup the basic A* node.
        super().__init__()

        # Precompute the sin/cos functions.
        c = cos(theta)
        s = sin(theta)

        # Define/remember the state (x,y,theta) and the coordinates
        # (x,y,s,c) used for the distance function.  We are assuming a
        # 180deg symmetry, so using 1/2 sin(2*theta), 1/2 cos(2*theta).
        self.x = x
        self.y = y
        self.s = L/2 * (s*c)            # = L/2 * 1/2 * sin(2*theta)
        self.c = L/2 * (c*c-s*s)/2      # = L/2 * 1/2 * cos(2*theta)
        self.t = theta

        # Pre-compute the mattress box (for collision detection and drawing).
        self.mattress = Polygon([[x + L/2*c + W/2*s, y + L/2*s - W/2*c],
                                 [x + L/2*c - W/2*s, y + L/2*s + W/2*c],
                                 [x - L/2*c - W/2*s, y - L/2*s + W/2*c],
                                 [x - L/2*c + W/2*s, y - L/2*s - W/2*c],
                                 [x + L/2*c + W/2*s, y + L/2*s - W/2*c]])

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Node %5.2f,%5.2f @ %5.2f>" % (self.x, self.y, self.t))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    # Note this should rotate in the "closer direction", we we wrap
    # the angle difference to fall within +/-90deg.
    def intermediate(self, other, alpha):
        return Node(self.x + alpha *     (other.x - self.x),
                    self.y + alpha *     (other.y - self.y),
                    self.t + alpha * wrap(other.t - self.t, pi))

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        return (self.x, self.y, self.s, self.c)

    # Compute the relative distance to another node.
    def distance(self, other):
        return sqrt((self.x - other.x)**2 + (self.y - other.y)**2 +
                    (self.s - other.s)**2 + (self.c - other.c)**2)

    ###############
    # A* functions:
    # Actual and Estimated costs.
    def costToConnect(self, other):
        return self.distance(other)

    def costToGoEst(self, other):
        return self.distance(other)

    ################
    # PRM functions:
    # Check whether in free space.
    def inFreespace(self):
        return walls.disjoint(self.mattress)

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        for delta in vandercorput.sequence(Dcspace / self.distance(other)):
            if not self.intermediate(other, delta).inFreespace():
                return False
        return True


######################################################################
#
#   PRM Functions
#
# Create the list of nodes.
def createNodes(N):
    #FIXME: create the list via (a) uniform sampling and (b) near edges.
   
    nodes = []

    #part a
    '''while len(nodes) < N:
        x_coord = random.uniform(xmin, xmax)
        y_coord = random.uniform(ymin, ymax)
        theta_coord = random.uniform(0, 2.0*pi)
        node = Node(x_coord, y_coord, theta_coord)
        if node.inFreespace():
            nodes.append(node)'''
    

    #part b
    q1 = None
    r = 2
    while len(nodes) < N:
        # get q1 (node that is in collision with object)
        while q1 is None:
            x_coord = random.uniform(xmin, xmax)
            y_coord = random.uniform(ymin, ymax)
            theta_coord = random.uniform(0, 2.0*pi)
            node = Node(x_coord, y_coord, theta_coord)
            if not node.inFreespace():
                q1 = node

        #sample about q1
        d = r * sqrt(random.uniform(0,1))
        phi = random.uniform(0, 2.0*pi)
        x_coord = q1.x + d * cos(phi)
        y_coord = q1.y + d * sin(phi)
        theta_coord = random.uniform(0, 2.0*pi)
        node = Node(x_coord, y_coord, theta_coord)
        if node.inFreespace():
            q2 = node
            nodes.append(q2)

        q1 = None


    return nodes

# Connect to up to K nearest neighbors (classic/standard approach)
def connectNearestNeighbors(nodes, K):
    # Clear any existing neighbors.  Use a set to add below.
    for node in nodes:
        node.neighbors = set()

    # Determine the indices for the K nearest neighbors.  Distance is
    # computed as the Euclidean distance of the coordinates.  This
    # also reports the node itself as the closest neighbor, so add one
    # extra here and ignore the first element below.
    X = np.array([node.coordinates() for node in nodes])
    [dist, idx] = KDTree(X).query(X, k=(K+1))
    
    # Add the edges.  Ignore the first neighbor (being itself).
    for i, nbrs in enumerate(idx):
        for n in nbrs[1:]:
            if nodes[n] not in nodes[i].neighbors:
                if nodes[i].connectsTo(nodes[n]):
                    nodes[i].neighbors.add(nodes[n])
                    nodes[n].neighbors.add(nodes[i])

# Connect to K neighbors (from all, testing nearest first)
def connectKNeighbors(nodes, K):
    # Clear any existing neighbors.  Use a set to add below.
    for node in nodes:
        node.neighbors = set()

    # Report all other nodes, sorted by distance, computed as the
    # Euclidean distance of the coordinates.  This includes the node
    # itself, so ignore the first element below.
    X = np.array([node.coordinates() for node in nodes])
    [dist, idx] = KDTree(X).query(X, k=len(nodes))

    # Check all until we have K neighbors:
    for i, nbrs in enumerate(idx):
        for n in nbrs[1:]:
            if len(nodes[i].neighbors) >= K:
                break
            if nodes[n] not in nodes[i].neighbors:
                if nodes[i].connectsTo(nodes[n]):
                    nodes[i].neighbors.add(nodes[n])
                    nodes[n].neighbors.add(nodes[i])

# Post Process the Path
def PostProcess(path):
    #FIXME: Remove nodes in the path than can be skipped without collisions
    ref_node = path[0]
    skipped_nodes = []
    if len(path) > 2:
        for i in range(1, len(path)-1):
            if ref_node.connectsTo(path[i+1]):
                skipped_nodes.append(path[i])
            else:
                ref_node = path[i]
        for node in skipped_nodes:
            path.remove(node)


######################################################################
#
#  Main Code
#

def success_rate():
    success_rate = 0
    num_samples = 1000

    for i in range(num_samples):
        # Report the parameters.
        print("Run number {}".format(i))
        print('Running with', N, 'nodes and', K, 'neighbors.')

        # Create the start/goal nodes.
        startnode = Node(xstart, ystart, tstart)
        goalnode  = Node(xgoal,  ygoal,  tgoal)

        # Create the list of sample points.
        print("Sampling the nodes...")
        tic = time.time()
        nodes = createNodes(N)
        toc = time.time()
        print("Sampled the nodes in %fsec." % (toc-tic))
        

        # Add the start/goal nodes.
        nodes.append(startnode)
        nodes.append(goalnode)


        # Connect to the nearest neighbors.  FIXME: Switch methods for (d).
        print("Connecting the nodes...")
        tic = time.time()
        #connectNearestNeighbors(nodes, K)
        connectKNeighbors(nodes, K)
        toc = time.time()
        print("Connected the nodes in %fsec." % (toc-tic))

        # Run the A* planner.
        print("Running A*...")
        tic = time.time()
        path = astar(nodes, startnode, goalnode)
        toc = time.time()
        print("Ran A* in %fsec." % (toc-tic))

        # If unable to connect, show the part explored.
        if not path:
            pass
        else:
            success_rate += 1.0

    success_rate = success_rate/num_samples
    print("Success rate: {}".format(success_rate))


def main():
    # Report the parameters.
    print('Running with', N, 'nodes and', K, 'neighbors.')

    # Create the figure.
    visual = Visualization()

    # Create the start/goal nodes.
    startnode = Node(xstart, ystart, tstart)
    goalnode  = Node(xgoal,  ygoal,  tgoal)

    # Show the start/goal nodes.
    visual.drawNode(startnode, color='orange', linewidth=3)
    visual.drawNode(goalnode,  color='purple', linewidth=3)
    visual.show("Showing basic world")


    # Create the list of sample points.
    print("Sampling the nodes...")
    tic = time.time()
    nodes = createNodes(N)
    toc = time.time()
    print("Sampled the nodes in %fsec." % (toc-tic))
    
    # Show the sample nodes.
    if True:
        for node in nodes:
            visual.drawNode(node, color='k', linewidth=1)
        visual.show("Showing the nodes")

    # Add the start/goal nodes.
    nodes.append(startnode)
    nodes.append(goalnode)


    # Connect to the nearest neighbors.  FIXME: Switch methods for (d).
    print("Connecting the nodes...")
    tic = time.time()
    #connectNearestNeighbors(nodes, K)
    connectKNeighbors(nodes, K)
    toc = time.time()
    print("Connected the nodes in %fsec." % (toc-tic))

    # Show the neighbor connections.
    if False:
        for (i,node) in enumerate(nodes):
            for neighbor in node.neighbors:
                if neighbor not in nodes[:i]:
                    visual.drawEdge(node, neighbor, color='g', linewidth=1)
        visual.show("Showing the full graph")


    # Run the A* planner.
    print("Running A*...")
    tic = time.time()
    path = astar(nodes, startnode, goalnode)
    toc = time.time()
    print("Ran A* in %fsec." % (toc-tic))

    # If unable to connect, show the part explored.
    if not path:
        print("UNABLE TO FIND A PATH")
        for node in nodes:
            if node.done:
                visual.drawNode(node, color='r', linewidth=1)
        visual.show("Showing DONE nodes")
        return

    # Show the path.
    visual.drawPath(path, color='r', linewidth=1)
    visual.show("Showing the raw path")


    # Post Process the path.
    PostProcess(path)

    # Show the post-processed path.
    visual.drawPath(path, color='b', linewidth=1)
    visual.show("Showing the post-processed path")


if __name__== "__main__":
    main()
    #success_rate()
