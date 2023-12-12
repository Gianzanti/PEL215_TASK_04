import numpy as np
import matplotlib.pyplot as plt
from icecream import ic

data = np.load("data_grid.npz")

max_value = np.max(data["grid"])
ic(max_value)
min_value = np.min(data["grid"])
ic(min_value)
mean_value = np.mean(data["grid"])
ic(mean_value)

fig, ax = plt.subplots(1, 1, figsize=(20, 6))

ax.set_ylabel("Posição Y [0.25 m cada]")
ax.set_xlabel("Posição X [0.25 m cada]")
ax.set_title("Grid de Ocupação", fontsize=14)
ax.set_aspect("equal", "box")

new_grid = np.rot90(data["grid"], k=3)
new_grid = np.fliplr(new_grid)
ax.pcolor(
    new_grid,
    cmap="bone_r",
    edgecolor="tab:gray",
    linewidths=1,
)
plt.show()
