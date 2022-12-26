"""
Starting node: Green
End node: Blue
Current node being explored: Orange
Neighbor of current node: Yellow
Path: Red
"""


import heapq
import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

# grid size
n = 10

# initialize grid
grid = [[0 for _ in range(n)] for _ in range(n)]

# set up walls
grid[1][5] = 1
grid[2][2] = 1
grid[2][3] = 1
grid[2][4] = 1
grid[3][2] = 1
grid[3][4] = 1
grid[4][5] = 1

# start and end positions
start = (0, 0)
end = (8,8)

# Set up the heap and distances.
heap = []
distances = {}


def dikstra():
    # Initialize the distances to infinity and add the start point to the heap.
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            distances[(x, y)] = float("inf")
    heap.append(start)
    distances[start] = 0
    plt.title("Djikstra")

    while heap:
        current = heapq.heappop(heap)
        plt.pause(time)
        ax.scatter(current[0], current[1], c='orange', s=100)
        if current == end:
            break

        # Check the neighboring cells.
        #improve to 8
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            x, y = current[0] + dx, current[1] + dy
            if (0 <= x < len(grid[0]) and 0 <= y < len(grid)
                    and grid[x][y] == 0 and distances[current] + 1 < distances[(x, y)]):
                heap.append((x, y))
                if (x,y) == end:
                    break
                distances[(x, y)] = distances[current] + 1
                ax.scatter(x, y, c='yellow', s=100)
                plt.pause(time)
        if (x,y) == end:
            break

    x, y = end
    while (x, y) != start:
        ax.scatter(x, y, c='r', s=100)
        plt.pause(time)
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            x2, y2 = x + dx, y + dy
            if (x2, y2) in distances and distances[(x2, y2)] < distances[(x, y)]:
                x, y = x2, y2
                break
    ax.scatter(x, y, c='r', s=100)
    plt.show()


# initialize A* algorithm
def astar(grid, start, end):
    # set of nodes already evaluated
    closed_set = set()
    # set of currently discovered nodes that are not evaluated yet
    open_set = []
    # map of came_from nodes
    came_from = {}
    # map of cost_so_far
    cost_so_far = {}
    # add start node to open set
    heapq.heappush(open_set, (0, start))
    # initialize cost to 0 for the start node
    cost_so_far[start] = 0
    # initialize path
    path = []

    # loop until open set is empty
    while open_set:
        # find the node in open set with the lowest f_score
        current = heapq.heappop(open_set)[1]
        plt.pause(time)
        ax.scatter(current[0], current[1], c='orange', s=100)
        # if current node is the end node, construct the path and return it
        if current == end:
            path = reconstruct_path(came_from, end)
            return path

        # remove current node from open set and add it to closed set
        closed_set.add(current)

        # get the neighbors of the current node
        neighbors = get_neighbors(grid, current)

        # loop through the neighbors
        for neighbor in neighbors:
            if neighbor == end:
                came_from[neighbor] = current
                path = reconstruct_path(came_from, end)
                return path
            # if neighbor is already in closed set, skip it
            if neighbor in closed_set:
                continue

            # calculate new cost to neighbor
            new_cost = cost_so_far[current] + 1

            # if neighbor is not in open set, add it and set its cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                plt.pause(time)
                ax.scatter(neighbor[0], neighbor[1], c='yellow', s=100)
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic_function(neighbor, end)
                heapq.heappush(open_set, (priority, neighbor))

                # update came_from map
                came_from[neighbor] = current

    # if no path was found, return empty list
    return []


# function to get the neighbors of a node
# improve using 8 neighbors
def get_neighbors(grid, node):
    x, y = node
    neighbors = []

    # check for neighbors to the left, right, top, and bottom of the current node
    if x > 0:
        neighbors.append((x - 1, y))
    if x < len(grid) - 1:
        neighbors.append((x + 1, y))
    if y > 0:
        neighbors.append((x, y - 1))
    if y < len(grid[0]) - 1:
        neighbors.append((x, y + 1))

    # filter out any neighbors that are walls
    neighbors = [n for n in neighbors if grid[n[0]][n[1]] != 1]

    return neighbors


# function to calculate the Euclidean distance between two nodes
def heuristic_function(node1, node2):
    x1, y1 = node1
    x2, y2 = node2
    # euclidean dist
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    # manhattan dist
    # return abs(x1-x2) + abs(y1-y2)


# function to reconstruct the path from came_from map
def reconstruct_path(came_from, current):
    global path
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path


# function to update the plot for each frame of the animation
def update():
    global path

    if not path:
        path = astar(grid, start, end)

    # highlight the current node on the grid
    for i in range(len(path)):
        x, y = path[i]
        plt.pause(time)
        ax.scatter(x, y, c='r', s=100)


def A_star():
    plt.title("A star")
    update()
    plt.show()


path = []

# create the plot
fig, ax = plt.subplots()

# plot the grid
for i in range(n):
    for j in range(n):
        color = 'k' if grid[i][j] == 1 else 'w'
        ax.scatter(i, j, c=color, s=100)
# plot the start and end positions
ax.scatter(start[0], start[1], c='g', s=100)
ax.scatter(end[0], end[1], c='b', s=100)

# set plot limits
ax.set_xlim((-1, n))
ax.set_ylim((-1, n))

time = 0.001
# A_star()
dikstra()
