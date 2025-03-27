from itertools import permutations
import numpy as np
from pprint import pprint
import time

class BruteForce:
    def __init__(self, points):
        self.points = points
        self.distances = self.calculate_distances()  # Distance matrix

    def calculate_distances(self):
        """Calculate the distance matrix for the cities."""
        n = len(self.points)
        distances = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i != j:
                    distances[i][j] = np.linalg.norm(np.array(self.points[i]) - np.array(self.points[j]))
                else:
                    distances[i][j] = np.inf  # Prevent division by zerof
        return distances
     
    def bruteForce(self):
        points = {i: point for i, point in enumerate(self.points)}
        point_indexes = list(range(len(self.points)))
        point_indexes.append(0)
        all_routes = []
        shortest_route = [float('inf'), []]

        for perm in permutations(point_indexes):
            if perm[0] == 0 and perm[-1] == 0:
                all_routes.append(perm)
        for route in all_routes:
            route_distance = self.find_distance(route)
            if route_distance < shortest_route[0]:
                shortest_route = [route_distance, route]
        # pprint(shortest_route)



    def find_distance(self, route):
        total_distance = 0
        for i in range(len(route) - 1):
            total_distance+=self.distances[route[i]][route[i + 1]]
        return total_distance
        

if __name__ == '__main__':
    bruteforce = BruteForce([
        [-13553450.00, 4381400.00],
        [-13554090.20, 4381520.30],
        [-13553380.10, 4381570.50],
        [-13553200.75, 4381445.85],
        [-13554200.60, 4381430.25],
        [-13553420.35, 4381385.70],
        [-13554150.45, 4381510.85],
        [-13553370.80, 4381590.40],
        [-13553250.90, 4381450.60],
        [-13554190.25, 4381425.40],
    ])
    start = time.time()

    bruteforce.bruteForce()
    print(time.time()-start)