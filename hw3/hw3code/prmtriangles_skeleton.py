#!/usr/bin/env python3
#
#   prmtriangles.py
#
#   Use PRM to find a path around triangular obstacles.
#
from asyncio import FastChildWatcher
from sre_constants import SUCCESS
from tracemalloc import start
from unicodedata import numeric
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math          import pi, sin, cos, sqrt, ceil
from scipy.spatial import KDTree
from shapely.geometry   import Point, LineString, Polygon, MultiPolygon
from shapely.prepared   import prep

from astar import AStarNode, astar


######################################################################
#
#   Parameters
#
#   FIXME: Define the N/K...
#
N = 12 #FIXME: Select the number of nodes
K = 10 #FIXME: Select the number of nearest neighbors


######################################################################
#
#   World Definitions
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (0, 14)
(ymin, ymax) = (0, 10)

# Collect all the triangle and prepare (for faster checking).
triangles = prep(MultiPolygon([
    Polygon([[ 2, 6], [ 3, 2], [ 4, 6], [ 2, 6]]),
    Polygon([[ 6, 5], [ 7, 7], [ 8, 5], [ 6, 5]]),
    Polygon([[ 6, 9], [ 8, 9], [ 6, 7], [ 6, 9]]),
    Polygon([[10, 3], [11, 6], [12, 3], [10, 3]])]))

# Define the start/goal states (x, y, theta)
(xstart, ystart) = ( 1, 5)
(xgoal,  ygoal)  = (13, 5)


######################################################################
#
#   Utilities: Visualization
#
# Visualization Class
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
        plt.gca().set_aspect('equal')

        # Show the triangles.
        for poly in triangles.context.geoms:
            plt.plot(*poly.exterior.xy, 'k-', linewidth=2)

        # Show.
        self.show()

    def show(self, text = ''):
        # Show the plot.
        plt.pause(0.001)
        # If text is specified, print and wait for confirmation.
        if len(text)>0:
            input(text + ' (hit return to continue)')

    def drawNode(self, node, *args, **kwargs):
        plt.plot(node.x, node.y, *args, **kwargs)

    def drawEdge(self, head, tail, *args, **kwargs):
        plt.plot((head.x, tail.x),
                 (head.y, tail.y), *args, **kwargs)

    def drawPath(self, path, *args, **kwargs):
        for i in range(len(path)-1):
            self.drawEdge(path[i], path[i+1], *args, **kwargs)


######################################################################
#
#   Node Definition
#
class Node(AStarNode):
    def __init__(self, x, y):
        # Setup the basic A* node.
        super().__init__()

        # Define/remember the state/coordinates (x,y).
        self.x = x
        self.y = y

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Point %5.2f,%5.2f>" % (self.x, self.y))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate(self, other, alpha):
        return Node(self.x + alpha * (other.x - self.x),
                    self.y + alpha * (other.y - self.y))

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        return (self.x, self.y)

    # Compute the relative distance to another node.
    def distance(self, other):
        #FIXME: compute and return the distance.
        dist = sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
        return dist

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
        return triangles.disjoint(Point(self.coordinates()))

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        line = LineString([self.coordinates(), other.coordinates()])
        return triangles.disjoint(line)


######################################################################
#
#   PRM Functions
#
# Create the list of nodes.
def createNodes(N):
    # Add nodes sampled uniformly across the space.
    nodes = []
    #FIXME: create the list of valid nodes sampling uniformly in x and y.
    while len(nodes) < N:
        x_coord = random.uniform(xmin, xmax)
        y_coord = random.uniform(ymin, ymax)
        node = Node(x_coord, y_coord)
        if node.inFreespace():
            nodes.append(node)
    return nodes


# Connect the nearest neighbors
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
'''def main():
    success_rate = 0.0
    num_samples = 10000
    for i in range(num_samples):
        # Report the parameters.
        print('Running with', N, 'nodes and', K, 'neighbors.')

        # Create the start/goal nodes.
        startnode = Node(xstart, ystart)
        goalnode  = Node(xgoal,  ygoal)

        # Create the list of nodes.
        print("Sampling the nodes...")
        tic = time.time()
        nodes = createNodes(N)
        toc = time.time()
        print("Sampled the nodes in %fsec." % (toc-tic))

        # Add the start/goal nodes.
        nodes.append(startnode)
        nodes.append(goalnode)

        # Connect to the nearest neighbors.
        print("Connecting the nodes...")
        tic = time.time()
        connectNearestNeighbors(nodes, K)
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
            #print("UNABLE TO FIND A PATH")
            pass
        else:
            path_narrow = True
            for node in path:
                if node.y < 5 or node.y >= 9:
                    path_narrow = False
                    break     
            if path_narrow:
                success_rate += 1.0

    success_rate = success_rate/num_samples
    print("Success rate: {}".format(success_rate))'''

def main():
    # Report the parameters.
    print('Running with', N, 'nodes and', K, 'neighbors.')

    # Create the figure.
    visual = Visualization()

    # Create the start/goal nodes.
    startnode = Node(xstart, ystart)
    goalnode  = Node(xgoal,  ygoal)

    # Show the start/goal nodes.
    visual.drawNode(startnode, color='orange', marker='o')
    visual.drawNode(goalnode,  color='purple', marker='o')
    visual.show("Showing basic world")


    # Create the list of nodes.
    print("Sampling the nodes...")
    tic = time.time()
    nodes = createNodes(N)
    toc = time.time()
    print("Sampled the nodes in %fsec." % (toc-tic))

    # Show the sample nodes.
    for node in nodes:
        visual.drawNode(node, color='k', marker='x')
    visual.show("Showing the nodes")

    # Add the start/goal nodes.
    nodes.append(startnode)
    nodes.append(goalnode)


    # Connect to the nearest neighbors.
    print("Connecting the nodes...")
    tic = time.time()
    connectNearestNeighbors(nodes, K)
    toc = time.time()
    print("Connected the nodes in %fsec." % (toc-tic))

    # Show the neighbor connections.
    for (i,node) in enumerate(nodes):
        for neighbor in node.neighbors:
            if neighbor not in nodes[:i]:
                visual.drawEdge(node, neighbor, color='g', linewidth=0.5)
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
                visual.drawNode(node, color='r', marker='o')
        visual.show("Showing DONE nodes")
        return

    # Show the path.
    visual.drawPath(path, color='r', linewidth=2)
    visual.show("Showing the raw path")


    # Post Process the path.
    PostProcess(path)

    # Show the post-processed path.
    visual.drawPath(path, color='b', linewidth=2)
    visual.show("Showing the post-processed path")

if __name__== "__main__":
    main()
