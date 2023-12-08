# from GridMap import GridMap
from icecream import ic

# map = GridMap()
# ic(map.bresenham((0, 0), (5, 3)))
# # ic(map.bresenham((4, 4), (6, 10)))


from itertools import cycle, islice


def cyclic_range(start, stop):
    # Create an iterator that cycles through the range indefinitely
    cyclic_iterator = cycle(range(stop))

    # Use islice to start from an arbitrary index and get a range of values
    result = list(islice(cyclic_iterator, start, start + stop))

    return result


# Example usage
arbitrary_start_index = 3
list_length = 7
result_list = cyclic_range(arbitrary_start_index, list_length)
ic(result_list)
