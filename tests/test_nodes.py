import os
import sys
import numpy as np
import pytest

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from pathfinding import Node,Pathfinder

def test_node_creation():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    node = Node(pathfinder, (0, 0),1)
    assert node.coord == (0, 0), 'Node coordinates should be (0, 0)'

def test_node_distances():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    step=1
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=step,
                             current_buffer=0,
                             human_location=None)
    node_a = Node(pathfinder, (0, 0),step)
    node_a.calculate_node_distances()

    assert node_a.neighbours['left'] ==[(-1,0),1] 
    up_right = node_a.neighbours['upright']
    assert up_right[0] == (1, 1)
    assert round(up_right[1], 4) == 1.4142

    upupright = node_a.neighbours['upupright']
    assert upupright[0] == (1, 2)
    assert round(upupright[1], 4) == 2.2361

def test_node_distances_decimal_step():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    step = 0.5
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=step,
                             current_buffer=0,
                             human_location=None)
    node_a = Node(pathfinder, (0, 0),step=step)
    node_a.calculate_node_distances()

    assert node_a.neighbours['left'] ==[(-0.5,0),0.5] 
    up_right = node_a.neighbours['upright']
    assert up_right[0] == (0.5, 0.5)
    assert round(up_right[1], 4) == 0.7071

    upupright = node_a.neighbours['upupright']
    assert upupright[0] == (0.5, 1)
    assert round(upupright[1], 4) == 1.1180

def test_node_distances_large_step():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    step = 10
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=step,
                             current_buffer=0,
                             human_location=None)
    node_a = Node(pathfinder, (0, 0),step=step)
    node_a.calculate_node_distances()

    assert node_a.neighbours['left'] ==[(-10,0),10]
    up_right = node_a.neighbours['upright']
    assert up_right[0] == (10, 10)
    assert round(up_right[1], 4) == 14.1421

    upupright = node_a.neighbours['upupright']
    assert upupright[0] == (10, 20)
    assert round(upupright[1], 4) == 22.3607

def test_node_distances_nearest():
    empty_obstacles = [[]]
    cities = [(0,0),(10,10)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    nearest_points_example = {'left': (0, 0)}
    
    node_a = Node(pathfinder, (10,10),1,nearest_points=nearest_points_example)
    print(node_a.neighbours)
    node_a.calculate_node_distances()
    all_neighbours = node_a.neighbours.values()
    for neighbour in all_neighbours:
        if neighbour[0] == (0, 0):
            assert round(neighbour[1], 4) == 14.1421

def test_node_distances_across_obstacle():
    obstacles = [[(5,-5),(5,5)]]
    cities = [(0,0)]
    nearest_points_example = {'left': (0, 0)}

    pathfinder=Pathfinder(cities=cities,
                             obstacles=obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    node_a = Node(pathfinder, (10,10),1,nearest_points=nearest_points_example)
    print(node_a.neighbours)
    node_a.calculate_node_distances()
    assert node_a.neighbours['left'] == [(0,0),(np.inf)]

def test_node_distances_invalid_input():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    nearest_points_example = {'left': 0} # Forces an incorrect input should be (0,0)
    
    node_a = Node(pathfinder, (10,10),1,nearest_points=nearest_points_example)

    with pytest.raises(TypeError):
        node_a.calculate_node_distances()

def test_node_information():
    empty_obstacles = [[]]
    cities = []
    pathfinder = Pathfinder(cities=cities,
                            obstacles=empty_obstacles,
                            pointcloud=None,
                            step=1,
                            current_buffer=0,
                            human_location=None)
    node_a = Node(pathfinder, (0, 0),1)
    assert node_a.__repr__() == '(0, 0)'
    
def test_node_creation_not_tuple():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    with pytest.raises(TypeError):
        node = Node(pathfinder, 0,1)  # Should be a tuple (0, 0)

def test_node_creation_xyz_coordinate():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=1,
                             current_buffer=0,
                             human_location=None)
    with pytest.raises(ValueError):
        node = Node(pathfinder, (0, 0, 0),1)

def test_node_creation_negative_step():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=-1,
                             current_buffer=0,
                             human_location=None)
    with pytest.raises(ValueError):
        node = Node(pathfinder, (0, 0),-1)  # Step should be positive

def test_node_creation_zero_step():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step=0,
                             current_buffer=0,
                             human_location=None)
    with pytest.raises(ValueError):
        node = Node(pathfinder, (0, 0),0)  # Step should be positive

def test_node_creation_step_not_number():
    empty_obstacles = [[]]
    cities = [(0,0),(1,1)]
    pathfinder = Pathfinder(cities=cities,
                             obstacles=empty_obstacles,
                             pointcloud=None,
                             step='a',
                             current_buffer=0,
                             human_location=None)
    with pytest.raises(ValueError):
        node = Node(pathfinder, (0, 0),'1')  # Step should be a number


