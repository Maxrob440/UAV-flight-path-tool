import pytest

from clustering import TspCluster, DistanceCluster
from Config import Config

config = Config()

def test_distance_cluster():
    config.update_nested(["clustering", "points_per_cluster"], 2)
    cities = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]
    human_location = (0,1)
    cluster_method = DistanceCluster(cities, human_location)
    clusters = cluster_method.cluster()
    
    assert len(clusters) ==3
    for cluster in clusters[:-1]:
        assert len(cluster) ==  4# doesnt include human location
    
    config.update_nested(["clustering", "points_per_cluster"], 20)
    cluster_method = DistanceCluster(cities, human_location)
    clusters = cluster_method.cluster()
    
    assert len(clusters) ==1
    assert len(clusters[0]) == 7  # all cities in one cluster

def test_distance_of_route():
    config.update_nested(["clustering", "points_per_cluster"], 6)

    cities = [(0,0), (1,0)]
    human_location = (0,1)
    cluster_method = DistanceCluster(cities, human_location)
    cluster_method.cluster()
    distance = cluster_method.distances_of_route()
    assert round(distance,1) == 3.4

def test_distance_of_route_TspCluster():
    config.update_nested(["clustering", "points_per_cluster"], 6)
    human_location = (0,1)

    cities = [(0,0), (1,0)]
    best_path_coords = [(0,0), (1,0), (0,1)]
    cluster_method = TspCluster(cities, best_path_coords,human_location)
    cluster_method.cluster()
    distance = cluster_method.distances_of_route()
    assert round(distance,1) == 3.4

def test_distance_of_route_error():
    cities = [(0,0), (1,0)]
    best_path_coords = [(0,0), (1,0), (0,1)]
    cluster_method = TspCluster(cities, best_path_coords)
    cluster_method.cluster()
    with pytest.raises(ValueError):
        distance = cluster_method.distances_of_route()

def test_tsp_cluster():
    config.update_nested(["clustering", "points_per_cluster"], 2)
    cities = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]
    best_path_coords = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]
    cluster_method = TspCluster(cities=cities, best_path_coords=best_path_coords)
    clusters = cluster_method.cluster()
    
    assert len(clusters) == 3
    for cluster in clusters[:-1]:
        assert len(cluster) == 4 # doesn't include human location


        
