# load data from npz
# plot data

import numpy as np
import matplotlib.pyplot as plt
from icecream import ic

# load data from npz
data = np.load("data_grid.npz")
ic(data.files)
ic(data["grid"])

new_grid = data["grid"].copy()
width = new_grid.shape[0]
height = new_grid.shape[1]

max_value = np.max(data["grid"])
ic(max_value)
min_value = np.min(data["grid"])
ic(min_value)

mean_value = np.mean(data["grid"])
ic(mean_value)


# Zero out the elements of the copied array
new_grid.fill(0)

for x in range(0, width):
    for y in range(0, height):
        if data["grid"][x][y] >= 100:
            new_grid[x][y] = 1
        # elif data["grid"][x][y] < -2000:
        #     new_grid[x][y] = -1
        else:
            new_grid[x][y] = -1

ic(new_grid)


xy_res = np.array(new_grid).shape
plt.figure(1, figsize=(10, 4))
plt.subplot(122)
plt.imshow(new_grid, cmap="bone")
plt.clim(-1, 1)
plt.gca().set_xticks(np.arange(-0.5, xy_res[1], 1), minor=True)
plt.gca().set_yticks(np.arange(-0.5, xy_res[0], 1), minor=True)
plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
plt.colorbar()
# plt.subplot(121)
# plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-")
# plt.axis("equal")
# plt.plot(0.0, 0.0, "ob")
# plt.gca().set_aspect("equal", "box")
# bottom, top = plt.ylim()  # return the current y-lim
# plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
# plt.grid(True)
plt.show()
