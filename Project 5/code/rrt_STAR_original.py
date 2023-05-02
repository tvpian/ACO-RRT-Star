import numpy as np
import matplotlib.pyplot as plt

# Define the problem
start = np.array([0, 0])
goal = np.array([7, 7])
obstacles = [np.array([5, 5]), np.array([7, 8]), np.array([3, 6])]

# Define the RRT Tree
class RRTNode:
    def __init__(self, state, parent=None):
        self.state = state
        self.parent = parent
        self.cost = 0

class RRTTree:
    def __init__(self, start):
        self.nodes = [RRTNode(start)]

    def add_node(self, new_node, parent_node):
        new_node.parent = parent_node
        new_node.cost = parent_node.cost + np.linalg.norm(new_node.state - parent_node.state)
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
        new_node.cost = nearest_node.cost + np.linalg.norm(new_node.state - nearest_node.state)
        return new_node

    def rewire(self, new_node, max_distance):
        near_nodes = [node for node in self.nodes if np.linalg.norm(node.state - new_node.state) < max_distance]
        for near_node in near_nodes:
            tentative_cost = new_node.cost + np.linalg.norm(new_node.state - near_node.state)
            if tentative_cost < near_node.cost:
                near_node.parent = new_node
                near_node.cost = tentative_cost

    # RRT* path planner
    def plan(self, start, goal, obstacles, max_distance, max_iterations):
        tree = RRTTree(start)
        for i in range(max_iterations):
            random_node = np.random.rand(2) * np.array([10, 10])
            nearest_node = self.get_nearest_node(random_node)
            new_node = self.get_new_node(nearest_node, random_node, max_distance)
            if not any([np.linalg.norm(obstacle - new_node.state) < 1 for obstacle in obstacles]):
                self.add_node(new_node, nearest_node)
                self.rewire(new_node, max_distance)
                if np.linalg.norm(new_node.state - goal) < 1:
                    print("Goal reached after %d iterations!" % i)
                    return
        print("Could not reach goal after %d iterations" % max_iterations)

    # Extract path from the RRT* Tree
    def extract_path(self, goal):
        path = [goal]
        current_node = self.nodes[-1]
        while current_node.parent is not None:
            path.append(current_node.parent.state)
            current_node = current_node.parent
        path.append(start)
        return path

    # Plot the path
    def plot_path(self, path, obstacles):
        fig, ax = plt.subplots()
        ax.set_xlim([0, 10])
        ax.set_ylim([0, 10])
        ax.set_aspect('equal')

        #Plot the tree
        for node in self.nodes:
            if node.parent is not None:
                ax.plot([node.state[0], node.parent.state[0]], [node.state[1], node.parent.state[1]], '-g')

        for obstacle in obstacles:
            circle = plt.Circle((obstacle[0], obstacle[1]), 1.0, color='gray')
            ax.add_artist(circle)
        ax.plot([state[0] for state in path], [state[1] for state in path], '-r')
        plt.show()

# Run the planner
tree = RRTTree(start)
tree.plan(start, goal, obstacles, 1.0, 2000)
path = tree.extract_path(goal)
tree.plot_path(path, obstacles)