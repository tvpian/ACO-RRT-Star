import numpy as np
from scipy.spatial.distance import euclidean

class Ant:
    def __init__(self, position, utility):
        self.position = position
        self.utility = utility

class RRTStar:
    def __init__(self, start, goal, obstacles):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        
        # Initialize tree with start node
        self.tree = [start]
        
    def extend(self, sample):
        # Find nearest node in tree
        nearest_node = self.find_nearest_node(sample)
        
        # Find new node in direction of sample
        new_node = self.find_new_node(nearest_node, sample)
        
        # Add new node to tree
        self.tree.append(new_node)
        
        # Rewire tree
        self.rewire(new_node)
        
    def find_nearest_node(self, sample):
        # Find distances from sample to each node in tree
        distances = [euclidean(sample, node) for node in self.tree]
        
        # Return node in tree with minimum distance to sample
        return self.tree[np.argmin(distances)]
    
    def find_new_node(self, nearest_node, sample):
        # Calculate unit vector in direction of sample
        unit_vector = (sample - nearest_node) / euclidean(sample, nearest_node)
        
        # Calculate new node position
        new_node = nearest_node + unit_vector
        
        # Clip new node to state space bounds
        new_node = np.clip(new_node, self.start, self.goal)
        
        return new_node
    
    def rewire(self, new_node):
        # Find nodes in tree within a certain radius of new node
        nodes_in_radius = [node for node in self.tree if euclidean(new_node, node) < 0.1]
        
        # Find node in nodes_in_radius with minimum cost to go
        min_cost_node = nodes_in_radius[np.argmin([self.cost_to_go(node) for node in nodes_in_radius])]
        
        # Find nodes in nodes_in_radius that are closer to new node than min_cost_node
        nodes_to_rewire = [node for node in nodes_in_radius if euclidean(new_node, node) < euclidean(new_node, min_cost_node)]
        
        # Rewire nodes in nodes_to_rewire
        for node in nodes_to_rewire:
            node = new_node
            
    def cost_to_go(self, node):
        # Return cost to go from node to goal
        return euclidean(node, self.goal)
    
    def find_best_path(self):
        # Find node in tree with minimum cost to go
        best_node = self.tree[np.argmin([self.cost_to_go(node) for node in self.tree])]

        # Find path from start to best node
        path = [best_node]
        while path[-1] != self.start:
            path.append(self.find_nearest_node(path[-1]))

        return path[::-1]



class ACO_RRTStar:
    def __init__(self, start, goal, obstacles, k, alpha, beta):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.k = k
        self.alpha = alpha
        self.beta = beta
        
        # Initialize ants with random positions and utilities
        self.ants = [Ant(np.random.uniform(low=start, high=goal), 0) for i in range(k)]
        
        # Initialize RRT* tree with start node
        self.tree = RRTStar(start, goal, obstacles)
        
    def run(self, num_iterations):
        for i in range(num_iterations):
            # Sample from current ant distribution
            samples = [self.sample_ant() for j in range(self.k)]
            
            # Extend tree with samples
            for sample in samples:
                self.tree.extend(sample)
                
            # Calculate utilities for each sample
            utilities = [self.calculate_utility(sample) for sample in samples]
            
            # Update ants based on utilities
            self.update_ants(utilities)
        
        # Return optimal path found by RRT* path planner
        return self.tree.find_best_path()
    
    def sample_ant(self):
        # Select an ant based on utility values
        ant = self.select_ant()
        
        # Generate a sample using Gaussian distribution centered at ant position
        sample = np.random.normal(loc=ant.position, scale=self.get_std(ant))
        
        # Clip sample to state space bounds
        sample = np.clip(sample, self.start, self.goal)
        
        return sample
    
    def calculate_utility(self, sample):
        # Calculate exploitation factor
        exploitation_factor = self.tree.cost_to_go(sample) / self.tree.cost_to_go(self.goal)
        
        # Calculate exploration factor
        exploration_factor = self.get_exploration_factor(sample)
        
        # Calculate total utility
        utility = self.alpha * exploitation_factor + self.beta * exploration_factor
        
        return utility
    
    def update_ants(self, utilities):
        # Update each ant's utility value
        for i in range(self.k):
            self.ants[i].utility = utilities[i]
            
        # Update each ant's position distribution
        total_utility = sum([ant.utility for ant in self.ants])
        for ant in self.ants:
            ant.position = (ant.utility / total_utility) * ant.position + (1 - ant.utility / total_utility) * self.select_ant().position
            
    def select_ant(self):
        # Select an ant based on roulette wheel selection
        utilities = [ant.utility for ant in self.ants]
        total_utility = sum(utilities)
        # Avoid division by zero
        probabilities = [utility / total_utility if total_utility > 0 else 1 / self.k for utility in utilities]
        index = np.random.choice(self.k, p=probabilities)
        
        return self.ants[index]
    
    def get_std(self, ant):
        # Calculate standard deviation for ant based on distance to other ants
        distances = [euclidean(ant.position, other_ant.position) for other_ant in self.ants if other_ant != ant]
        std = np.mean(distances) if len(distances) > 0 else 1
        
        return std
    
    def get_exploration_factor(self, sample):
        # Calculate exploration factor based on how far the sample is from the tree
        nearest_node = self.tree.nearest_neighbor(sample)
        distance = euclidean(sample, nearest_node)
        return 1 / distance if distance > 0 else 1

# Define Rectangle class
class Rectangle:
    def __init__(self, center, width, height):
        self.center = center
        self.width = width
        self.height = height
        
    def contains(self, point):
        # Check if point is inside rectangle
        return (point[0] >= self.center[0] - self.width / 2 and point[0] <= self.center[0] + self.width / 2 and
                point[1] >= self.center[1] - self.height / 2 and point[1] <= self.center[1] + self.height / 2)

# Define plot_path function
def plot_path(path, obstacles, start, goal):
    # Plot obstacles
    for obstacle in obstacles:
        plt.gca().add_patch(plt.Rectangle((obstacle.center[0] - obstacle.width / 2, obstacle.center[1] - obstacle.height / 2), obstacle.width, obstacle.height, color='k'))
    
    # Plot start and goal
    plt.scatter(start[0], start[1], color='g', s=100)
    plt.scatter(goal[0], goal[1], color='r', s=100)
    
    # Plot path
    for i in range(len(path) - 1):
        plt.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], color='b')
    
    # Set axis limits
    plt.xlim([-1, 11])
    plt.ylim([-1, 11])
    
    # Show plot
    plt.show()


if __name__ == "__main__":
    # Define start and goal positions
    start = np.array([0, 0])
    goal = np.array([10, 10])

    # Define obstacles
    obstacles = [Rectangle(np.array([3, 3]), 2, 2), Rectangle(np.array([7, 7]), 2, 2)]

    # Define number of ants
    k = 10

    # Define alpha and beta parameters
    alpha = 0.5
    beta = 0.5

    # Define number of iterations
    num_iterations = 1000

    # Initialize ACO_RRTStar planner
    planner = ACO_RRTStar(start, goal, obstacles, k, alpha, beta)

    # Run planner
    path = planner.run(num_iterations)

    # Plot results
    plot_path(path, obstacles, start, goal)

