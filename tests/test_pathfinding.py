import os
import sys
import numpy as np
import pytest

from Config import Config
from point_cloud import PointCloud

config = Config()

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from pathfinding import Pathfinder,Node

def test_distances_between_cities():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    assert round(pathfinder.distances[((0,0),(1,1))][1],4) == 1.4142
    assert pathfinder.distances[((0,0),(0,0))] == np.inf

def test_distances_between_cities_with_obstacles():
    obstacles = [[(-5,5),(5,5)]]
    cities = [(0,0),(0,10)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=obstacles,
                             pointcloud=None,
                             step=0.25,
                             current_buffer=0,
                             human_location=None)
    assert pathfinder.distances[((0,0),(0,10))][1] != 10.0
    assert pathfinder.distances[(0,0),(0,10)][1] <17 # pythagoream case would be ~14.1

def test_hurestic_h_function_for_a_star():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    tuples_to_test = [(0,0),(1,1),(0,1)]
    nodes = {tup:Node(pathfinder,tup,1) for tup in tuples_to_test}
    assert pathfinder.h(nodes[(0,0)],nodes[(0,1)]) == 1, 'Distance between (0,0) and (0,1) should be 1'
    assert round(pathfinder.h(nodes[(0,0)],nodes[(1,1)]),4) == 1.4142, 'Distance between (0,0) and (1,1) should be 1.4142'

def test_h_function_for_a_star_with_tuples_and_lists():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    assert pathfinder.h((0,0),(0,1)) == 1, 'Distance between (0,0) and (0,1) should be 1'
    assert pathfinder.h([0,0],[0,1]) == 1, 'Distance between (0,0) and (0,1) should be 1'

def test_h_function_invalid_XYZ_input():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    with pytest.raises(TypeError):
        pathfinder.h((0,0),(0,1,1))

def test_h_function_invalid_input_not_tuple_or_node():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    with pytest.raises(TypeError):
        pathfinder.h(1,2)


def test_reconstruct_path():
    came_from={(1, 0): (0, 0), (1, 1): (1, 0), (2, 1): (1, 1)}
    current=(2, 1)
    pathfinder = Pathfinder(cities=[(0, 0),(2,1),(1,0)],
                             obstacles=[],
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    path = pathfinder.reconstruct_path(came_from, current)
    print(path)
    assert path[0] == [(0, 0), (1, 0), (1, 1), (2, 1)]
    assert path[1] == 3

def test_reconstruct_path_list_not_tuple():
    came_from={(1, 0): (0, 0), (1, 1): (1, 0), (2, 1): (1, 1)}
    current=[2, 1]
    pathfinder = Pathfinder(cities=[(0, 0),(2,1)],
                             obstacles=None,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    with pytest.raises(TypeError):
        path = pathfinder.reconstruct_path(came_from, current)


def test_a_star_no_obstacles():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    path, cost = pathfinder.a_star((0,0),(1,1))
    assert path == [(0, 0), (1, 1)]
    assert round(cost,4) == 1.4142

def test_a_star_direct_with_obstacles():
    obstacles = [[(10,10),(20,10)]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    path, cost = pathfinder.a_star((0,0),(1,1))
    assert path == [(0, 0), (1, 1)]
    assert round(cost,4) == 1.4142

def test_a_star_node_input():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    start_node = Node(pathfinder,(0,0),1)
    end_node = Node(pathfinder,(1,1),1)
    with pytest.raises(TypeError):
        path, cost = pathfinder.a_star(start_node,end_node)

def test_a_star_no_direct_path():
    obstacles = [[(-5,5),(5,5)]]
    cities = [(0,0),(0,10)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=obstacles,
                             pointcloud=None,
                             step=0.25,
                             current_buffer=0,
                             human_location=None)
    path, cost = pathfinder.a_star((0,0),(0,10))

    assert path[-1] == (0, 10)
    assert path[0]== (0, 0)
    assert len(path)>2

def test_a_star_impossible():
    config.update_nested(['speed_related','maximum_recusive_depth'], '1')
    obstacles = [[(-1,-1),(-1,1),(1,1),(1,-1)]]
    cities = [(0,0),(0,10)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    path, cost = pathfinder.a_star((0,0),(0,10))
    assert path == []
    assert cost == np.inf

def test_calculate_grid_distances_empty_obstacles():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    pathfinder.calculate_grid_distances(1)
    assert round(pathfinder.distances[((0,0),(1,1))][1],4) == 1.4142
    assert pathfinder.distances[((0,0),(0,0))] == np.inf

def test_calculate_grid_distances_obstacles():
    obstacles = [[(0.5,-1),(0.5,1)]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                            obstacles=obstacles,
                            pointcloud=None,
                            step=1,
                            current_buffer=0,
                            human_location=None)
    pathfinder.calculate_grid_distances(1)
    assert pathfinder.distances[((0,0),(1,1))][1] >1 and pathfinder.distances[((0,0),(1,1))][1] < 4 # optimum will be 2.2
    assert len(pathfinder.distances[((0,0),(1,1))][0]) > 2
    assert pathfinder.distances[((0,0),(0,0))] == np.inf

def test_all_points_visible_on_interpolated_line():
    cities = [(0,0),(2,2)]
    empty_obstacles = [[]]
    config.update_nested(['current_map','folder_location'],'fake_path')
    pointcloud = PointCloud()
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=pointcloud,
                             step=1,
                             current_buffer=0,
                             human_location=(1,1))
    pathfinder.pointcloud.read_tif([(0,0,0),(1,1,0),(2,2,0)])

    
    assert pathfinder.all_points_visible_interpolated_line(cities[0],cities[1]) is True

def test_all_points_not_visible_on_interpolated_line():
    cities = [(0,0),(10,0)]
    human_location = (5,5,0)
    empty_obstacles = [[]]
    
    config.update_nested(['speed_related', 'DVLOS_interpolation_m'], '1')
    config.update_nested(['distances', 'height_above_ground_m'], '0')
    config.update_nested(['current_map','folder_location'],'fake_path')
    
    pointcloud = PointCloud()
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=pointcloud,
                             step=1,
                             current_buffer=0,
                             human_location=human_location)
    pathfinder.pointcloud.read_tif([(3,3,0),(4,3,0),(5,3,0),(6,3,0),(7,3,0)])
    print(config.config['distances']['height_above_ground_m'])

    assert pathfinder.all_points_visible_interpolated_line(cities[0],cities[1]) is False
    # pathfinder.pointcloud.show_point_cloud(cities=cities, human_location=human_location, dvlos=True)


    


def test_line_crosses_polygon():
    obstacles = [[(0.5,-1),(0.5,1)]]
    polygon = [(0.5,-1),(0.5,1)]
    cities=[(0,0),(1,0)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    print(type(cities[0]))
    assert pathfinder.line_crosses_polygon(cities[0],cities[1],polygon) is True

def test_line_crosses_polygon_cache():
    obstacles = [[(0.5,-1),(0.5,1)]]
    polygon = [(0.5,-1),(0.5,1)]
    cities=[(0,0),(1,0)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    print(type(cities[0]))
    assert pathfinder.line_crosses_polygon(cities[0],cities[1],polygon) is True
    assert pathfinder.line_crosses_polygon(cities[0],cities[1],polygon) is True
    assert pathfinder.line_crosses_polygon(cities[1],cities[0],polygon) is True








