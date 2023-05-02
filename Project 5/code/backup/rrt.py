import numpy as np
import matplotlib.pyplot as plt

# Define the problem
start = np.array([0, 0])
goal = np.array([10, 10])
obstacles = [np.array([5, 5]), np.array([7, 8]), np.array([3, 6])]

# Define the RRT Tree
class RRTNode:
    def __init__(self, state, parent=None):
        self.state = state
        self.parent = parent

class RRTTree:
    def __init__(self, start):
        self.nodes = [RRTNode(start)]

    def add_node(self, new_node, parent_node):
        new_node.parent = parent_node
        self.nodes.append(new_node)

    def get_nearest_node(self, random_node):
        distances = [np.linalg.norm(node.state - random_node) for node in self.nodes]
        nearest_node = self.nodes[np.argmin(distances)]
        return nearest_node

    def get_new_node(self, nearest_node, random_node, max_distance):
        direction = (random_node - nearest_node.state)
        distance = np.linalg.norm(direction)
        if distance > max_distance:
            direction = direction / distance * max_distance
        new_state = nearest_node.state + direction
        new_node = RRTNode(new_state)
        return new_node

# RRT path planner
def plan(start, goal, obstacles, max_distance, max_iterations):
    tree = RRTTree(start)
    for i in range(max_iterations):
        random_node = np.random.rand(2) * np.array([10, 10])
        nearest_node = tree.get_nearest_node(random_node)
        new_node = tree.get_new_node(nearest_node, random_node, max_distance)
        if not any([np.linalg.norm(obstacle - new_node.state) < 1 for obstacle in obstacles]):
            tree.add_node(new_node, nearest_node)
            if np.linalg.norm(new_node.state - goal) < 1:
                print("Goal reached after %d iterations!" % i)
                return tree
    print("Could not reach goal after %d iterations" % max_iterations)
    return tree

# Extract path from the RRT Tree
def extract_path(tree, goal):
    path = [goal]
    current_node = tree.nodes[-1]
    while current_node.parent is not None:
        path.append(current_node.parent.state)
        current_node = current_node.parent
    path.append(start)
    return path

# Plot the path
def plot_path(path, obstacles, tree=None):
    plt.scatter(start[0], start[1], color='green')
    plt.scatter(goal[0], goal[1], color='red')


    # Plot tree
    for node in tree.nodes:
        if node.parent is not None:
            plt.plot([node.state[0], node.parent.state[0]], [node.state[1], node.parent.state[1]], color='green')

    for obstacle in obstacles:
        plt.scatter(obstacle[0], obstacle[1], color='black')

    for i in range(len(path) - 1):
        plt.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], color='blue')
        
    plt.show()

# Run the planner
tree = plan(start, goal, obstacles, 1, 1000)
path = extract_path(tree, goal)
plot_path(path, obstacles, tree)
