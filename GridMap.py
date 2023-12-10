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

    def __init__(self, origin_x=0, origin_y=0, resolution=0.5, width=20, height=20):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.zeros((self.width, self.height))
        self.thresholdFree = -10000
        self.thresholdOccupied = 3000

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

    def set_occupancy_grid(self, laser_beams, position):
        ic(position)
        iPosX = self.calc_xy_index_from_pos(position[0], self.origin_x, self.width)
        iPosY = self.calc_xy_index_from_pos(position[1], self.origin_y, self.height)
        ic(iPosX, iPosY)
        if (iPosX is None) or (iPosY is None):
            ic("***************** Invalid position")
            return

        cells = []
        for beam in laser_beams:
            ix = self.calc_xy_index_from_pos(beam[0], self.origin_x, self.width)
            iy = self.calc_xy_index_from_pos(beam[1], self.origin_y, self.height)
            if (ix is None) or (iy is None):
                ic("***************** Invalid beam", beam)
                continue

            if cells.count((ix, iy)) != 0:
                continue

            ic(ix, iy)
            ic(beam)
            cells.append((ix, iy))

            laser_beams = self.bresenham((iPosX, iPosY), (ix, iy))
            ic(laser_beams)

            for z in laser_beams[:-1]:
                ic("free", z)
                if z[0] < self.width and z[1] < self.height and z[0] >= 0 and z[1] >= 0:
                    self.grid[z[0]][z[1]] += self.cost["free"]
                    if self.grid[z[0]][z[1]] < self.thresholdFree:
                        self.grid[z[0]][z[1]] = self.thresholdFree
                    ic(self.grid[z[0]][z[1]])

            if ix < self.width and iy < self.height and ix >= 0 and iy >= 0:
                ic("occupied", (ix, iy))
                self.grid[ix][iy] += self.cost["occupied"]
                if self.grid[ix][iy] > self.thresholdOccupied:
                    self.grid[ix][iy] = self.thresholdOccupied
                ic(self.grid[ix][iy])

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
