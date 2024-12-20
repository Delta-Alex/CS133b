#!/usr/bin/env python3
#
#   hw5_utilities.py
#
#   Two classes to help the visualization and simulation.
#
#
#   VISUALIZATION:
#
#     visual = Visualization(walls, robot)
#     visual.Show(prob)
#     visual.Show(prob, markRobot=True)
#
#     walls         NumPy 2D array defining both the grid size
#                   (rows/cols) and walls (being non-zero elements).
#     robot         Robot object
#     prob          NumPy 2D array of probabilities, values 0 to 1.
#
#   visual.Show() will visualize the probabilities.  The second form
#   will also mark the robot's position with an 'x'.
#
#
#   ROBOT SIMULATION:
#
#     robot = Robot(walls, row=0, col=0, probProximal=[1.0],
#                   probCmd=1.0, kidnap=False)
#     robot.Command(drow, dcol)
#     True/False = robot.Sensor(drow, dcol)
#
#     probCmd       Probability the command is executed (0 to 1).
#     probProximal  List of probabilities (0 to 1).  Each element is
#                   the probability that the proximity sensor will
#                   fire at a distance of (index+1 = 1,2,3,etc)
#     kidnap        Flag - kidnap the robot (at some point)
#     (drow, dcol)  Delta up/right/down/left: (-1,0) (0,1) (1,0) (0,-1)
#
#   Simulate a robot, to give us the sensor readings.  If the starting
#   row/col are not given, pick them randomly.  Note both the command
#   and the sensor may be configured to a random probability level.
#
import matplotlib.pyplot as plt
import numpy as np
import random


#
#   Probailiity Grid Visualization
#
class Visualization():
    def __init__(self, walls, robot):
        # Save the walls, robot, and determine the rows/cols:
        self.walls = walls
        self.robot = robot
        self.spots = np.sum(np.logical_not(walls))
        self.rows  = np.size(walls, axis=0)
        self.cols  = np.size(walls, axis=1)

        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.grid(False)
        plt.gca().axis('off')
        plt.gca().set_aspect('equal')
        plt.gca().set_xlim(0, self.cols)
        plt.gca().set_ylim(self.rows, 0)

        # Add the row/col numbers.
        for row in range(0, self.rows, 2):
            plt.gca().text(         -0.3, 0.5+row, '%d'%row,
                           verticalalignment='center',
                           horizontalalignment='right')
        for row in range(1, self.rows, 2):
            plt.gca().text(self.cols+0.3, 0.5+row, '%d'%row,
                           verticalalignment='center',
                           horizontalalignment='left')
        for col in range(0, self.cols, 2):
            plt.gca().text(0.5+col,          -0.3, '%d'%col,
                           verticalalignment='bottom',
                           horizontalalignment='center')
        for col in range(1, self.cols, 2):
            plt.gca().text(0.5+col, self.rows+0.3, '%d'%col,
                           verticalalignment='top',
                           horizontalalignment='center')

        # Draw the grid, zorder 1 means draw after zorder 0 elements.
        for row in range(self.rows+1):
            plt.gca().axhline(row, lw=1, color='k', zorder=1)
        for col in range(self.cols+1):
            plt.gca().axvline(col, lw=1, color='k', zorder=1)

        # Clear the content and mark.  Then show with zeros.
        self.content = None
        self.mark    = None
        self.Show(np.zeros((self.rows, self.cols)))

    def Flush(self):
        # Show the plot.
        plt.pause(0.001)

    def Mark(self, markRobot=True):
        # Clear/potentially remove the previous mark.
        if self.mark is not None:
            self.mark.remove()
            self.mark = None

        # If requested, add a new mark.
        if markRobot:
            # Grab the robot position and check.
            row = self.robot.row
            col = self.robot.col
            assert (row >= 0) and (row < self.rows), "Illegal robot row"
            assert (col >= 0) and (col < self.cols), "Illegal robot col"

            # Draw the mark.
            self.mark  = plt.gca().text(0.5+col, 0.5+row, 'x', color = 'green',
                                        verticalalignment='center',
                                        horizontalalignment='center',
                                        zorder=1)

    def Grid(self, prob):
        # Check the probability grid array size.
        assert (np.size(prob, axis=0) == self.rows), "Inconsistent num of rows"
        assert (np.size(prob, axis=1) == self.cols), "Inconsistent num of cols"

        # Potentially remove the previous grid/content.
        if self.content is not None:
            self.content.remove()
            self.content = None

        # Create the color range.  There are clearly more elegant ways...
        color = np.ones((self.rows, self.cols, 3))
        for row in range(self.rows):
            for col in range(self.cols):
                if self.walls[row,col]:
                    color[row,col,0:3] = np.array([0.0, 0.0, 0.0])   # Black
                else:
                    # Shades of pink/purple/blue. Yellow means impossible.
                    p    = prob[row,col]
                    pmin = 0.9 / self.spots
                    if p == 0:
                        color[row,col,0:3] = np.array([1.0, 1.0, 0.0])
                    elif p < pmin:
                        rlevel = (1.0 - p)
                        glevel = (1.0 - p)
                        color[row,col,0:3] = np.array([rlevel, glevel, 1.0])
                    else:
                        rlevel = (1.0 - p)
                        glevel = (1.0 - p) * pmin/p
                        color[row,col,0:3] = np.array([rlevel, glevel, 1.0])
    
        # Draw the boxes.
        self.content = plt.gca().imshow(color,
                                        aspect='equal',
                                        interpolation='none',
                                        extent=[0, self.cols, self.rows, 0],
                                        zorder=0)

    def Show(self, prob, markRobot = False):
        # Update the content.
        self.Grid(prob)

        # Potentially add the mark.
        self.Mark(markRobot)

        # Flush the figure.
        self.Flush()


#
#  Robot (Emulate the actual robot)
#
#    probCmd is the probability that the command is actually executed
#
#    probProximal is a list of probabilities.  Each element
#    corresponds to the probability that the proximity sensor will
#    fire at a distance of (index+1).
#
class Robot():
    def __init__(self, walls, row = 0, col = 0,
                 probProximal = [1.0], probCmd = 1.0, kidnap = False):
        # Check the row/col arguments.
        assert (row >= 0) and (row < np.size(walls, axis=0)), "Illegal row"
        assert (col >= 0) and (col < np.size(walls, axis=1)), "Illegal col"

        # Report.
        if walls[row, col]:
            location = " (random location)"
        else:
            location = " (at %d, %d)" % (row, col)
        print("Starting robot with real" +
              " probProximal = " + str(probProximal) +
              ", probCmd = " + str(probCmd) +
              ", kidnap = " + str(kidnap) + str(location))

        # Save the walls, the initial location, and the probabilities.
        self.walls        = walls
        self.probCmd      = probCmd
        self.probProximal = probProximal
        if kidnap:
            self.countdown = random.randrange(10, 15)
        else:
            self.countdown = 0

        # Set the initial location (the default is invalid, so randomize).
        self.JumpTo(row, col)

    def JumpTo(self, row, col):
        # If invalid, randomize until we have a valid location.
        while self.walls[row, col]:
            row = random.randrange(0, np.size(self.walls, axis=0))
            col = random.randrange(0, np.size(self.walls, axis=1))
        # Set the location.
        self.row = row
        self.col = col

    def Command(self, drow, dcol):
        # Check the delta.
        assert ((abs(drow+dcol) == 1) and (abs(drow-dcol) == 1)), "Bad delta"

        # Check whether to kidnap.
        if self.countdown > 0:
            self.countdown -= 1
            if self.countdown == 0:
                self.JumpTo(0,0)
                return

        # Try to move the robot the given delta.
        row = self.row + drow
        col = self.col + dcol
        if (not self.walls[row, col]) and (random.random() < self.probCmd):
            self.row = row
            self.col = col

    def Sensor(self, drow, dcol): 
        # Check the delta.
        assert ((abs(drow+dcol) == 1) and (abs(drow-dcol) == 1)), "Bad delta"

        # Check the proximity in the given direction.
        for k in range(len(self.probProximal)):
            if self.walls[self.row + drow*(k+1), self.col + dcol*(k+1)]:
                return (random.random() < self.probProximal[k])
        return False
