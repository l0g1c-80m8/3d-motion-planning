from bresenham import bresenham
import numpy as np
import constants as ct
from sklearn.neighbors import KDTree


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

    return grid, int(north_min), int(east_min)


def relative_grid_pose(local_pose, north_offset=0, east_offset=0):
    return int(local_pose[0]) - north_offset, int(local_pose[1]) - east_offset


def prune_path(path, grid):
    pruned_path = path.copy()
    idx = 0
    while idx < len(pruned_path) - 2:
        point1 = pruned_path[idx]
        point2 = pruned_path[idx + 1]
        point3 = pruned_path[idx + 2]

        bres_path = bresenham(int(point1[0]), int(point1[1]), int(point3[0]), int(point3[1]))
        is_path_valid = True

        for point in bres_path:
            if grid[point] != 0:
                is_path_valid = False
                break

        if is_path_valid:
            pruned_path.remove(point2)
        else:
            idx += 1

    return pruned_path


def get_grid_goal(data, grid_start, goal_distance, north_offset, east_offset):
    if goal_distance == ct.GOAL_DISTANCE['NEARBY']:
        return grid_start[0] + 15, grid_start[1] + 25
    elif goal_distance == ct.GOAL_DISTANCE['FAR']:
        return grid_start[0] + 420, grid_start[1] - 250
    elif goal_distance == ct.GOAL_DISTANCE['MEDIUM']:
        return grid_start[0] + 120, grid_start[1] - 100
    else:
        return relative_grid_pose((np.random.choice(data[:, 0], size=1)[0], np.random.choice(data[:, 1], size=1)[0]),
                                  north_offset, east_offset)


def get_random_point_in_free_space(free_space):
    new_conf_index = np.random.randint(0, len(free_space[0]))
    return free_space[0][new_conf_index], free_space[1][new_conf_index]


def get_nearest_in_tree(nodes, node_to_query, return_distance=False):
    kd_tree = KDTree(nodes, leaf_size=2)
    # return index of the nearest node in nodes
    nearest_node_distance, nearest_node_index = kd_tree.query([node_to_query], k=1, return_distance=True)

    if not return_distance:
        return nearest_node_index[0][0]
    return nearest_node_distance, nearest_node_index[0][0]


def get_new_node_on_grid(rand_conf, nearest_conf, delta):
    x1, y1 = rand_conf[0], rand_conf[1]
    x2, y2 = nearest_conf[0], nearest_conf[1]
    if x1 == x2 and y1 == y2:
        return False

    total_d = pow((x1 - x2) ** 2 + (y1 - y2) ** 2, 0.5)
    ratio_d = delta / total_d
    new_conf_x, new_conf_y = ((1 - ratio_d) * x2) + ratio_d * x1, ((1 - ratio_d) * y2) + ratio_d * y1

    return int(new_conf_x), int(new_conf_y)


def is_edge_valid(grid, node1, node2):
    bres_path = bresenham(int(node1[0]), int(node1[1]), int(node2[0]), int(node2[1]))

    for point in bres_path:
        if point[0] > 920 or point[1] > 920:
            return False
        if grid[point] != 0:
            return False
    return True


def rtt(grid, start, goal, delta, max_points=10000):
    free_space = np.where(grid == 0.0)
    nodes = [start]
    edges = []

    while len(nodes) < max_points:
        print(len(nodes))
        rand_conf = get_random_point_in_free_space(free_space)
        nearest_conf_index = get_nearest_in_tree(nodes, rand_conf)
        new_conf = get_new_node_on_grid(rand_conf, nodes[nearest_conf_index], delta)
        if not new_conf:
            continue
        if not is_edge_valid(grid, new_conf, nodes[nearest_conf_index]):
            continue
        nodes.append(new_conf)
        edges.append((nearest_conf_index, len(nodes) - 1))

        nearest_distance_from_goal, nearest_node_index_from_goal = get_nearest_in_tree(nodes, goal, True)
        if nearest_distance_from_goal < delta:
            nodes.append(goal)
            edges.append((nearest_node_index_from_goal, len(nodes) - 1))
            print(nearest_distance_from_goal, nodes[nearest_node_index_from_goal])
            break

    return nodes, edges
