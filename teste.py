import math
from icecream import ic

mapResolution = 0.2
ic(mapResolution)

mapSize = (10, 10)
ic(mapSize)

mapWidth = int(mapSize[0] / mapResolution)
ic(mapWidth)

mapHeight = int(mapSize[1] / mapResolution)
ic(mapHeight)

div = 0.6 / mapResolution
ic(div)

minimum = math.ceil(div)
ic(minimum)
