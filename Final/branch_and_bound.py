import numpy as np
import heapq
import time

class Node:
    def __init__(self, level, path, reduced_matrix, cost):
        self.level = level
        self.path = path
        self.reduced_matrix = reduced_matrix
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost  # Ensures lower cost nodes are processed first

class BnB:
    def __init__(self, points):
        self.points = points
        self.distances = self.calculate_distances()
        self.best_cost = float('inf')
        self.best_path = None
        self.n = len(points)

    def calculate_distances(self):
        """Calculate the distance matrix for the cities."""
        n = len(self.points)
        distances = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i != j:
                    distances[i][j] = np.linalg.norm(np.array(self.points[i]) - np.array(self.points[j]))
                else:
                    distances[i][j] = np.inf  # Prevents traveling to itself
        return distances

    def reduce_matrix(self, distances):
        """Reduces the cost matrix and returns the reduced matrix and its reduction cost."""
        reduced_matrix_cost = 0
        reduced_matrix = distances.copy()

        # Row reduction
        for i in range(reduced_matrix.shape[0]):
            min_val = np.min(reduced_matrix[i])
            if min_val != np.inf:  # Don't subtract if already infinite
                reduced_matrix_cost += min_val
                reduced_matrix[i] -= min_val

        # Column reduction
        for i in range(reduced_matrix.shape[1]):
            min_val = np.min(reduced_matrix[:, i])
            if min_val != np.inf:
                reduced_matrix_cost += min_val
                reduced_matrix[:, i] -= min_val

        return reduced_matrix, reduced_matrix_cost

    def solve(self):
        """Solves the TSP using the Branch and Bound algorithm."""
        pq = []

        # Step 1: Reduce the original matrix
        reduced_matrix, cost = self.reduce_matrix(self.distances)

        # Step 2: Create the root node with city 0 as the start
        start = Node(level=0, path=[0], reduced_matrix=reduced_matrix, cost=cost)

        heapq.heappush(pq, start)

        while pq:
            node = heapq.heappop(pq)

            # Step 3: If all cities have been visited, compute final cost
            if node.level == self.n - 1:
                final_cost = node.cost + self.distances[node.path[-1]][0]  # Return to start

                if final_cost < self.best_cost:
                    self.best_cost = final_cost
                    self.best_path = node.path + [0]  # Properly append start city

                continue

            last_visited = node.path[-1]

            # Step 4: Explore all possible next cities
            for next_city in range(self.n):
                if next_city not in node.path:
                    # Step 5: Create a new matrix excluding the visited nodes
                    new_matrix = node.reduced_matrix.copy()
                    new_matrix[last_visited, :] = np.inf
                    new_matrix[:, next_city] = np.inf
                    new_matrix[next_city, last_visited] = np.inf

                    # Step 6: Reduce the new matrix and calculate the new cost
                    new_reduced_matrix, new_cost = self.reduce_matrix(new_matrix)
                    total_cost = node.cost + self.distances[last_visited][next_city] + new_cost

                    # Step 7: Add to priority queue if it's promising
                    if total_cost < self.best_cost:
                        new_node = Node(
                            level=node.level + 1,
                            path=node.path + [next_city],
                            reduced_matrix=new_reduced_matrix,
                            cost=total_cost,  # Fix: use total_cost, not new_cost
                        )
                        heapq.heappush(pq, new_node)

        return self.best_path, self.best_cost
    def real_distance(self, path):
        total_distance = 0
        for i in range(len(path) - 1):
            total_distance += self.distances[path[i]][path[i + 1]]
        return total_distance

# Example test case
branch_and_bound = BnB([
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
best_path, best_cost = branch_and_bound.solve()
print("Time:", time.time() - start)
print("Best Path:", best_path)
print("Best Cost:", branch_and_bound.real_distance(best_path))# Path: Final/branch_and_bound.py
