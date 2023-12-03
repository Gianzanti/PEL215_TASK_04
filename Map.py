import math
from matplotlib import pyplot as plt
import numpy as np
from icecream import ic

class Map(object):
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

    def __init__(self, origin_x=0.0, origin_y=0.0, resolution=.5, width=20, height=20):
        """ Construct an empty occupancy grid.
        
        Arguments: origin_x, 
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells 
                                in meters.
                   width, 
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.
                                
         The default arguments put (0,0) in the center of the grid. 
                                
        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width + 1
        self.height = height + 1
        self.grid = np.zeros((self.width, self.height))
        # default 0.5 -- [[0.5 for i in range(y_w)] for i in range(x_w)]
        self.cost = {'free': math.log(0.35/0.65), 'occupied': math.log(0.65/0.35), 'unknown': math.log(1)}
        # ic(self.cost)


    def set_cell(self, x, y, val):
        """ Set the value of a cell in the grid. 

        Arguments: 
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).

        This would probably be a helpful method!  Feel free to throw out
        point that land outside of the grid. 
        """
        self.grid[y][x] = val

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

    def set_occupancy_grid(self, ox, oy, position):
        ox_adjusted = ox + position[0]
        oy_adjusted = oy + position[1]
        # ic(ox_adjusted, oy_adjusted)

        iPosX = int(round((position[0] - self.origin_x) / self.resolution))
        iPosY = int(round((position[1] - self.origin_y) / self.resolution))
        # ic(iPosX, iPosY)

        cells = []
        for (x, y) in zip(ox_adjusted, oy_adjusted):
            ix = int(round((x - self.origin_x) / self.resolution))
            iy = int(round((y - self.origin_y) / self.resolution))
            # ic(ix, iy)

            if cells.count((ix, iy)) != 0:
                continue
            
            cells.append((ix, iy))
            
            laser_beams = self.bresenham((iPosX, iPosY), (ix, iy))
            
            for laser_beam in laser_beams[:-1]:
                if (laser_beam[0] < self.width and laser_beam[1] < self.height):
                    self.grid[laser_beam[0]][laser_beam[1]] += self.cost['free']

            if (ix < self.width and iy < self.height):
                self.grid[ix][iy] += self.cost['occupied']

            # if (ix + 1 < self.width and iy + 1 < self.height):
            #     self.grid[ix + 1][iy] += math.log(self.cost['occupied']) # extend the occupied area
            #     self.grid[ix][iy + 1] += math.log(self.cost['occupied']) # extend the occupied area
            #     self.grid[ix + 1][iy + 1] += math.log(self.cost['occupied']) # extend the occupied area

    def show(self):
        """ Display the grid. """
        # ic(self.grid)
        plt.imshow(self.grid, cmap="PiYG_r")

        plt.clim(-100, 100)
        plt.gca().set_xticks(np.arange(-.5, 0.1, 1), minor=True)
        plt.gca().set_yticks(np.arange(-.5, 0.1, 1), minor=True)
        plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
        plt.colorbar()

        plt.show()