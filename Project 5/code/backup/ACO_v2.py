import numpy as np
import matplotlib.pyplot as plt

# Define the problem
start = (0, 0)
goal = (10, 10)
obstacles = [(5, 5), (6, 5), (5, 6), (6, 6)]
num_ants = 10
alpha = 1
beta = 5
evaporation_rate = 0.5
Q = 100
max_iterations = 100

# Define the ACO algorithm
class Ant:
    def __init__(self):
        self.visited_nodes = [start]
        self.current_node = start

    def choose_next_node(self, pheromone, distance):
        unvisited_nodes = [(i, j) for i in range(11) for j in range(11) if (i, j) not in self.visited_nodes and (i, j) not in obstacles]
        probabilities = [0] * len(unvisited_nodes)
        for i, node in enumerate(unvisited_nodes):
            probabilities[i] = pheromone[self.current_node][node] ** alpha * (1.0 / (distance[self.current_node][node] + 0.0001)) ** beta
        probabilities = probabilities / np.sum(probabilities)
        next_node = unvisited_nodes[np.random.choice(len(unvisited_nodes), p=probabilities)]
        self.visited_nodes.append(next_node)
        self.current_node = next_node

def update_pheromone(pheromone, ants, distances):
    pheromone *= evaporation_rate
    for ant in ants:
        for i in range(len(ant.visited_nodes) - 1):
            node1, node2 = ant.visited_nodes[i], ant.visited_nodes[i + 1]
            pheromone[node1][node2] += Q / (distances[node1][node2] + 0.0001)

def find_shortest_path(pheromone):
    current_node = start
    path = [current_node]
    while current_node != goal:
        next_node = max([(node, pheromone[current_node][node]) for node in pheromone[current_node]], key=lambda x: x[1])[0]
        path.append(next_node)
        current_node = next_node
    return path

# Run the ACO algorithm
distance = np.zeros((11, 11))
for i in range(11):
    for j in range(11):
        distance[i][j] = np.linalg.norm(np.array((i, j)) - np.array(goal))
pheromone = np.ones((11, 11)) / 10
shortest_path = None
shortest_distance = float('inf')
for iteration in range(max_iterations):
    ants = [Ant() for i in range(num_ants)]
    for ant in ants:
        for i in range(10):
            ant.choose_next_node(pheromone, distance)
        if ant.visited_nodes[-1] == goal:
            distance_travelled = sum([distance[ant.visited_nodes[i]][ant.visited_nodes[i+1]] for i in range(len(ant.visited_nodes)-1)])
            if distance_travelled < shortest_distance:
                shortest_path = ant.visited_nodes
                shortest_distance = distance_travelled
    update_pheromone(pheromone, ants, distance)
    if iteration % 10 == 0:
        print("Iteration %d: shortest distance = %.2f" % (iteration, shortest_distance))

# Plot the result
fig, ax = plt
ax.plot([node[0] for node in shortest_path], [node[1] for node in shortest_path], 'r')
ax.plot([node[0] for node in obstacles], [node[1] for node in obstacles], 'ko')
ax.plot(start[0], start[1], 'bo')
ax.plot(goal[0], goal[1], 'bo')
ax.set_xlim([-1, 11])
ax.set_ylim([-1, 11])
ax.set_xticks(np.arange(0, 11, 1))
ax.set_yticks(np.arange(0, 11, 1))
ax.grid()
ax.set_aspect('equal')
plt.show()


