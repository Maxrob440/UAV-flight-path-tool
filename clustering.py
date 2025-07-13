import math
from Config import Config
from pathfinding import BnB

class ClusteringMethod:
    def __init__(self, cities:list[tuple[float, float]], 
                 human_location:tuple[float, float]):
        self.cities = cities
        self.human_location = human_location[:2] if human_location else None
        self.config = Config()
        self.clusters = []
    
    def distances_of_route(self):
        # does not use for human location, just the clusters
        if not self.human_location:
            raise ValueError("Human location must be set to calculate distances of route.")
        
        total_distance = 0
        for cluster in self.clusters:
            distance = math.dist(cluster[0], self.human_location)  # Distance from human location to first city in cluster
            for i in range(len(cluster)-1):
                distance += math.dist(cluster[i], cluster[i+1])
            distance += math.dist(cluster[-1], self.human_location)  # Distance from last city in cluster to human location
            # distance+= math.dist(cluster[-1], cluster[0])  # Closing the loop
            total_distance += distance
        # print(distance)
        return total_distance




class DistanceCluster(ClusteringMethod):

    def cluster(self):
        print("Clustering cities based on distance from human location")
        cluster_size = int(self.config.get_nested("clustering", "points_per_cluster"))
        human_location = self.human_location[:2]
        distances = {}
        for city in self.cities:
            distance = math.dist(city,human_location)
            distances[city] = distance

        sorted_cities = sorted(distances,key=distances.get)
        # print(sorted_cities)
        clusters = []
        while len(sorted_cities) > 0:
            cluster = []
            while len(cluster)< cluster_size and len(sorted_cities) > 0:
                next_city = sorted_cities.pop()
                if next_city not in cluster:
                    cluster.append(next_city)
            clusters.append(cluster)
        self.clusters = clusters
        return clusters

class TspCluster(ClusteringMethod):
    def __init__(self, cities:list[tuple[float, float]], 
                 best_path_coords:list[tuple[float, float]],
                 human_location:tuple[float, float] = None):
        super().__init__(cities, human_location=human_location)
        self.best_path_coords = best_path_coords
    
    def cluster(self):
        tsp_path = self.best_path_coords
        points_per_cluster = int(self.config.get_nested('clustering','points_per_cluster'))
        clustered_tsp = []
        current_cluster = []

        for ind,point in enumerate(tsp_path):
            if point not in self.cities and len(current_cluster) ==0: # Stops a* path from previous cluster being added to the next cluster
                continue
            current_cluster.append(point)
            count_real_points = sum(1 for p in current_cluster if p in self.cities)
            if count_real_points >= points_per_cluster:
                clustered_tsp.append(current_cluster)
                current_cluster = []
        if current_cluster:
            clustered_tsp.append(current_cluster)
        self.clusters = clustered_tsp
                    
                    

        
       
        
        
        return clustered_tsp


        

        
        

if __name__ == '__main__':
    cities = [(0,0),(1,1),(2,2)]
    best_path_coords = [(0,0),(1,1),(2,2)]
    config = Config()
    config.update_nested(['clustering', 'points_per_cluster'], 2)
    clusterer = DistanceCluster(cities=cities, human_location=(1,0))
    print(clusterer.cluster())
    print(clusterer.distances_of_route())

    clusterer = TspCluster(cities=cities,best_path_coords=best_path_coords)
    print(clusterer.cluster())
    print(clusterer.distances_of_route())

        