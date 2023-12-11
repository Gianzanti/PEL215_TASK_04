import math
from matplotlib import pyplot as plt
import numpy as np
from icecream import ic


class GridMap(object):
    """
    The Map class stores an occupancy grid as a two dimensional
    numpy array.

    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters.
        origin_x   --  Position of the grid cell (0,0) in
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.


    Note that x increases with increasing column number and y increases
    with increasing row number.
    """

    def __init__(self, origin_x=0, origin_y=0, resolution=0.25, width=40, height=40):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.zeros((self.width, self.height))
        self.thresholdFree = -4000
        self.thresholdOccupied = 4000

        self.cost = {
            "free": math.log(0.35 / 0.65),
            "occupied": math.log(0.65 / 0.35),
            "unknown": math.log(1),
        }
        # ic(self.cost)

    def bresenham(self, start, end):
        """
        Implementation of Bresenham's line drawing algorithm
        See en.wikipedia.org/wiki/Bresenham's_line_algorithm
        Bresenham's Line Algorithm
        Produces a np.array from start and end (original from roguebasin.com)
        >>> points1 = bresenham((4, 4), (6, 10))
        >>> print(points1)
        np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
        """
        # setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        is_steep = abs(dy) > abs(dx)  # determine how steep the line is
        if is_steep:  # rotate line
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        # swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        dx = x2 - x1  # recalculate differentials
        dy = y2 - y1  # recalculate differentials
        error = int(dx / 2.0)  # calculate error
        # error = dy - dx  # calculate error
        y_step = 1 if y1 < y2 else -1
        # iterate over bounding box generating points between start and end
        y = y1

        points = []
        for x in range(x1, x2 + 1):
            coord = [y, x] if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx

        if swapped:  # reverse the list if the coordinates were swapped
            points.reverse()

        points = np.array(points)
        return points

    def calc_xy_index_from_pos(self, pos, lower_pos, max_index):
        ind = int(np.floor((pos - lower_pos) / self.resolution))
        if 0 <= ind <= max_index:
            return ind
        else:
            return None

    def get_xy_index_from_pos(self, pos):
        indx = int(np.floor((pos[0] - self.origin_x) / self.resolution))
        indy = int(np.floor((pos[1] - self.origin_y) / self.resolution))
        if (0 <= indx < self.width) and (0 <= indy < self.height):
            return (indx, indy)
        else:
            return None

    def setCell(self, idx, val, max_value=None, min_value=None):
        try:
            current = self.grid[idx[0]][idx[1]] + val
            if min_value is not None and current < min_value:
                current = min_value

            if max_value is not None and current > max_value:
                current = max_value

            self.grid[idx[0]][idx[1]] = current

        except ValueError:
            ic("***************** Invalid Cell")
            pass

    def set_occupancy_grid(self, lidar, position):
        iPos = self.get_xy_index_from_pos(position)
        if iPos is None:
            ic("***************** Invalid position")
            return

        cells = []
        for beam in lidar:
            idx = self.get_xy_index_from_pos((beam[0], beam[1]))
            if idx is None:
                continue

            if cells.count(idx) > 0:
                continue

            cells.append(idx)

            line_path = self.bresenham(iPos, idx)
            for z in line_path[:-1]:
                self.setCell(z, self.cost["free"], min_value=self.thresholdFree)

            self.setCell(idx, self.cost["occupied"], max_value=self.thresholdOccupied)

    def show(self):
        """Display the grid."""
        # ic(self.grid)
        plt.imshow(self.grid, cmap="PiYG_r")

        plt.clim(-100, 100)
        plt.gca().set_xticks(np.arange(-0.5, 0.1, 1), minor=True)
        plt.gca().set_yticks(np.arange(-0.5, 0.1, 1), minor=True)
        plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
        plt.colorbar()

        plt.show()
