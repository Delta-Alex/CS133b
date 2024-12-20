#!/usr/bin/env python3
#
#   prmrobot.py
#
#   Use PRM to find a path for the planar three link robot.
#
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math          import pi, sin, cos, sqrt, ceil
from scipy.spatial import KDTree
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
#part a) N=95, K = 6 -> 84.2 percent
N = 95
K = 6


######################################################################
#
#   World Definitions
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (-3.5, 3.5)
(ymin, ymax) = (-1.5, 3.5)

(xL, xR)     = (-0.5, 0.5)
(yB, yL, yR) = (-0.5, 1.5, 1.0)

xlabels = (-3.5, -3, -2.5, -2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5)
ylabels = (-1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5)

# Collect all the walls
walls = MultiLineString([[[xmin, yB], [xmax, yB]],
                         [[xmin, yL], [xL, yL], [xL, ymax]],
                         [[xR, yR], [xR, ymax]]])

# Define the start/goal states (joint values)
(startq1, startq2, startq3) = (0, 0, 0)
(goalq1,  goalq2,  goalq3)  = (pi/2, 0, 0)

# FIXME: Define any distances you need to use, in physical or joint spaces?
Dx = 0.1 #dmin = 0.1m in the lecture video
Dq = Dx/3.0 #(dmin/length of arm = dmin/3m)

Dqdraw = 0.5    # Joint distance to space robots while drawing path


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
        for l in walls.geoms:
            plt.plot(*l.xy, color='k', linewidth=2)

        # Place joint 0 only once!
        plt.gca().add_artist(plt.Circle((0,0),color='k',radius=0.05))

        # Show.
        self.show()

    def show(self, text = ''):
        # Show the plot.
        plt.pause(0.001)
        # If text is specified, print and wait for confirmation.
        if len(text)>0:
            input(text + ' (hit return to continue)')

    def drawTip(self, node, *args, **kwargs):
        plt.arrow(node.xB, node.yB, 0.9*(node.xC-node.xB), 0.9*(node.yC-node.yB),
                  head_width=0.1, head_length=0.1, *args, **kwargs)

    def drawRobot(self, node, *args, **kwargs):
        plt.plot((0, node.xA, node.xB), (0, node.yA, node.yB), *args, **kwargs)
        self.drawTip(node, *args, **kwargs)
        kwargs['radius']=0.05
        plt.gca().add_artist(plt.Circle((node.xA,node.yA),*args,**kwargs))
        plt.gca().add_artist(plt.Circle((node.xB,node.yB),*args,**kwargs))

    def drawNode(self, node, *args, **kwargs):
        self.drawTip(node, *args, **kwargs)

    def drawEdge(self, n1, n2, *args, **kwargs):
        plt.plot(((n1.xB + n1.xC)/2, (n2.xB + n2.xC)/2),
                 ((n1.yB + n1.yC)/2, (n2.yB + n2.yC)/2), *args, **kwargs)

    def drawPath(self, path, *args, **kwargs):
        for i in range(len(path)-1):
            n = ceil(path[i].distance(path[i+1]) / Dqdraw)
            for j in range(n):
                node = path[i].intermediate(path[i+1], j/n)
                self.drawRobot(node, *args, **kwargs)
                plt.pause(0.1)
        self.drawRobot(path[-1], *args, **kwargs)


######################################################################
#
#   Node Definition
#
class Node(AStarNode):
    def __init__(self, q1, q2, q3):
        # Setup the basic A* node.
        super().__init__()

        # FIXME: Finish the initialization.
        #FIXME: Save any states/coordinates you need.
        # joint angles 
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

        self.q1_s, self.q1_c = sin(q1), cos(q1)
        self.q2_s, self.q2_c = sin(q2), cos(q2)
        self.q3_s, self.q3_c = sin(q3), cos(q3)

        # Pre-compute the link positions.
        (self.xA, self.yA) = (          cos(q1)      ,           sin(q1)      )
        (self.xB, self.yB) = (self.xA + cos(q1+q2)   , self.yA + sin(q1+q2)   )
        (self.xC, self.yC) = (self.xB + cos(q1+q2+q3), self.yB + sin(q1+q2+q3))
        self.links = LineString([[0,0], [self.xA,self.yA],
                                 [self.xB,self.yB], [self.xC, self.yC]])

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Joints %6.1fdeg,%6.1fdeg,%6.1fdeg>" %
                (self.q1 * 180/pi, self.q2 * 180/pi, self.q3 * 180/pi))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    # Note, how to you move from 350deg to 370deg?  Is this a +20deg
    # movement?  Or is 370deg=10deg and this is a -240deg movement?
    def intermediate(self, other, alpha):
        #FIXME: Please implement
        return Node(self.q1 + alpha * wrap(other.q1 - self.q1, 2*pi),
                    self.q2 + alpha * wrap(other.q2 - self.q2, 2*pi),
                    self.q3 + alpha * wrap(other.q3 - self.q3, 2*pi))

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        #FIXME: Please implement
        return (self.q1_c, self.q1_s, self.q2_c, self.q2_s, self.q3_c, self.q3_s)

    # Compute the relative distance to another node.  See above.
    def distance(self, other):
        #FIXME: Please implement
        return sqrt( (self.q1_c - other.q1_c)**2 + (self.q1_s - other.q1_s)**2 + 
                     (self.q2_c - other.q2_c)**2 + (self.q2_s - other.q2_s)**2 +
                     (self.q3_c - other.q3_c)**2 + (self.q3_s - other.q3_s)**2 )


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
        #FIXME: return True if you are know the arm is not hitting any wall.
        return walls.disjoint(self.links)

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        #FIXME: return True if you can move without collision.
        if walls.distance(self.links) < Dx or walls.distance(other.links) < Dx:
            return False

        for delta in vandercorput.sequence(Dq / self.distance(other)):
            intermediate_node = self.intermediate(other, delta)
            if not intermediate_node.inFreespace():
                return False
            if walls.distance(intermediate_node.links) < Dx:
                return False

        return True


######################################################################
#
#   PRM Functions
#
# Create the list of nodes.
def createNodes(N):
    #FIXME: return a list of valid nodes.
    #pass
    nodes = []
    while len(nodes) < N:
        q1_coord = random.uniform(-pi, pi)
        q2_coord = random.uniform(-pi, pi)
        q3_coord = random.uniform(-pi, pi)
        node = Node(q1_coord, q2_coord, q3_coord)
        if node.inFreespace():
            nodes.append(node)

    return nodes

# Connect to the nearest neighbors.
def connectNeighbors(nodes, K):
    #FIXME: determine/set the node.neighbors for all nodes.  Make sure
    #       you create an undirected graph: the neighbors should be
    #       symmetric.  So, if node B becomes a neighbor to node A,
    #       then also add node A to the neighbors of node B.

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
    #FIXME: remove unnecessary nodes from the path, if the predecessor and
    #       successor can connect directly.  I.e. minimize the steps.
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
        startnode = Node(startq1, startq2, startq3)
        goalnode  = Node(goalq1,  goalq2,  goalq3)

        # Create the list of sample points.
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
        connectNeighbors(nodes, K)
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
            success_rate += 1

    success_rate = success_rate/num_samples
    print("Success rate: {}".format(success_rate))

    


def main():
    # Report the parameters.
    print('Running with', N, 'nodes and', K, 'neighbors.')

    # Create the figure.
    visual = Visualization()

    # Create the start/goal nodes.
    startnode = Node(startq1, startq2, startq3)
    goalnode  = Node(goalq1,  goalq2,  goalq3)

    # Show the start/goal nodes.
    visual.drawRobot(startnode, color='orange', linewidth=2)
    visual.drawRobot(goalnode,  color='purple', linewidth=2)
    visual.show("Showing basic world")


    # Create the list of sample points.
    print("Sampling the nodes...")
    tic = time.time()
    nodes = createNodes(N)
    toc = time.time()
    print("Sampled the nodes in %fsec." % (toc-tic))

    # Show the sample nodes.
    for node in nodes:
        visual.drawNode(node, color='k', linewidth=1)
    visual.show("Showing the nodes (last link only)")

    # Add the start/goal nodes.
    nodes.append(startnode)
    nodes.append(goalnode)


    # Connect to the nearest neighbors.
    print("Connecting the nodes...")
    tic = time.time()
    connectNeighbors(nodes, K)
    toc = time.time()
    print("Connected the nodes in %fsec." % (toc-tic))

    # Show the neighbor connections.
    if False:
        for (i,node) in enumerate(nodes):
            for neighbor in node.neighbors:
                if neighbor not in nodes[:i]:
                    visual.drawEdge(node, neighbor, color='g', linewidth=2)
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
                visual.drawNode(node, color='r')
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

    # Report the path.
    print(path[0])
    for i in range(1,len(path)):
        path[i] = path[i-1].intermediate(path[i], 1.0)
        print(path[i])
    

if __name__== "__main__":
    main()
    #success_rate()
