import numpy as np
import random

class AntColony:
    def __init__(self, cities,
                 no_connect = set(),
                  n_ants=100,
                    n_iterations=100,
                      alpha=1,
                        beta=2,
                          evaporation=0.5,
                            q=100):
        self.cities = cities  # List of cities (coordinates)
        self.n_ants = n_ants  # Number of ants
        self.n_iterations = n_iterations  # Number of iterations
        self.alpha = alpha  # Pheromone influence
        self.beta = beta  # Distance influence
        self.evaporation = evaporation  # Pheromone evaporation rate
        self.q = q  # Constant for pheromone deposit
        self.pheromones = np.ones((len(cities), len(cities)))  # Initialize pheromone matrix
        self.distances = self.calculate_distances(no_connect)  # Distance matrix

    def calculate_distances(self,no_connect):
        """Calculate the distance matrix for the cities."""
        n = len(self.cities)
        distances = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i != j and (i,j) not in no_connect:
                    distances[i][j] = np.linalg.norm(np.array(self.cities[i]) - np.array(self.cities[j]))
                else:
                    distances[i][j] = np.inf  # Prevent division by zero

        return distances

    def probability(self, current, unvisited):
        """Calculate probabilities for choosing the next city."""
        pheromone = self.pheromones[current][unvisited] ** self.alpha
        distance = (1 / self.distances[current][unvisited]) ** self.beta

        probs = pheromone * distance
        total_prob = np.sum(probs)

        if total_prob == 0 or np.isnan(total_prob):  # Avoid division by zero
            return np.ones(len(unvisited)) / len(unvisited)  # Equal probabilities

        return probs / total_prob


    def run(self):
        """Run the ACO algorithm for TSP."""
        best_path = None
        best_distance = float("inf")

        for _ in range(self.n_iterations):
            all_paths = []
            all_distances = []

            for _ in range(self.n_ants):
                path = []
                unvisited = list(range(len(self.cities)))
                # current = random.choice(unvisited)  # Random starting city
                current = 0
                path.append(current)
                unvisited.remove(current)

                while unvisited:
                    probs = self.probability(current, unvisited)
                    next_city = np.random.choice(unvisited, p=probs)
                    path.append(next_city)
                    unvisited.remove(next_city)
                    current = next_city

                path.append(path[0])  # Return to start]
                distance = sum(self.distances[path[i], path[i+1]] for i in range(len(path) - 1))
                all_paths.append(path)
                all_distances.append(distance)

                if distance < best_distance:
                    best_distance = distance
                    best_path = path


            # Update pheromones
            self.pheromones *= (1 - self.evaporation)  # Evaporation
            for path, dist in zip(all_paths, all_distances):
                for i in range(len(path) - 1):
                    self.pheromones[path[i]][path[i+1]] += self.q / dist
        best_path_coors = [self.cities[i] for i in best_path]

        return best_path, best_distance,best_path_coors

if __name__ == "__main__":

    # Example: Solve TSP with ACO
    # cities = [[-13553498.81, 4381376.72], [-13554115.72, 4381530.1], [-13553391.63, 4381581.2], [-13553213.71, 4381438.6], [-13554213.71, 4381438.6]]
    cities =[(-3.828721497698752, 50.85117709698622, 10), (-3.356614792757713, 50.55669103627278, 10), (-3.669419086873676, 50.54784734524204, 10), (-3.336952037288932, 50.583031970728904, 10), (-3.609534752323134, 50.07494938405071, 10), (-3.161519257054485, 50.10352500091346, 10), (-3.9908298794847736, 50.52797102808983, 10), (-3.1405855704099332, 50.493577922737565, 10), (-3.9158861893561587, 50.96949141435576, 10), (-3.38695371098159, 50.287953525421024, 10)]
    # cities = [(random.randint(0, 1000), random.randint(0, 1000)) for _ in range(100)]
    aco = AntColony(cities)
    best_path, best_distance,best_path_coordss = aco.run()
    print("Best Distance:", best_distance)
