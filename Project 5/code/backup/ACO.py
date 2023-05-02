import numpy as np
import matplotlib.pyplot as plt

# Define the problem
num_cities = 20
cities = np.random.rand(num_cities, 2) * 10
pheromone = np.ones((num_cities, num_cities)) / num_cities
alpha = 1
beta = 5
evaporation_rate = 0.5
num_ants = 10
Q = 100
max_iterations = 100

# Define the ACO algorithm
class Ant:
    def __init__(self, start_city):
        self.visited_cities = [start_city]
        self.current_city = start_city

    def choose_next_city(self, pheromone, alpha, beta):
        unvisited_cities = [city for city in range(num_cities) if city not in self.visited_cities]
        probabilities = [0] * len(unvisited_cities)
        for i, city in enumerate(unvisited_cities):
            probabilities[i] = pheromone[self.current_city, city] ** alpha * (1.0 / np.linalg.norm(cities[self.current_city] - cities[city])) ** beta
        probabilities = probabilities / np.sum(probabilities)
        next_city = np.random.choice(unvisited_cities, p=probabilities)
        self.visited_cities.append(next_city)
        self.current_city = next_city

def update_pheromone(pheromone, ants):
    pheromone *= evaporation_rate
    for ant in ants:
        for i in range(num_cities - 1):
            city1, city2 = ant.visited_cities[i], ant.visited_cities[i + 1]
            pheromone[city1, city2] += Q / np.linalg.norm(cities[city1] - cities[city2])

def find_shortest_path(pheromone):
    path = [0]
    current_city = 0
    while len(path) < num_cities:
        next_city = np.argmax(pheromone[current_city])
        path.append(next_city)
        current_city = next_city
    return path

# Run the ACO algorithm
shortest_path = None
shortest_distance = float('inf')
for iteration in range(max_iterations):
    ants = [Ant(np.random.randint(num_cities)) for i in range(num_ants)]
    for ant in ants:
        for i in range(num_cities - 1):
            ant.choose_next_city(pheromone, alpha, beta)
        distance = np.linalg.norm(cities[ant.visited_cities[-1]] - cities[ant.visited_cities[0]])
        if distance < shortest_distance:
            shortest_path = ant.visited_cities
            shortest_distance = distance
    update_pheromone(pheromone, ants)
    if iteration % 10 == 0:
        print("Iteration %d: shortest distance = %.2f" % (iteration, shortest_distance))

# Plot the result
fig, ax = plt.subplots()
ax.set_xlim([0, 10])
ax.set_ylim([0, 10])
ax.set_aspect('equal')
for i in range(num_cities):
    ax.plot(cities[i, 0], cities[i, 1], 'bo')
for i in range(num_cities - 1):
    ax.plot(cities[shortest_path[i:i+2], 0], cities[shortest_path[i:i+2], 1], 'r')
ax.plot(cities[shortest_path[0], 0], cities[shortest_path[0], 1], 'r')
plt.show()

# Path: ACO
