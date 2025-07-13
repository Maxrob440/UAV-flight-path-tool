from unittest.mock import patch
from point_cloud import PointCloud
from Config import Config
import numpy as np
import pytest

config = Config()
config.load_config()
config.update_nested(['distances','height_above_ground_m'], '0')

def test_load_tif():
    config.update_nested(['current_map','folder_location'],'tests/test_files/tif')

    pointcloud_obj = PointCloud()
    pointcloud_obj.read_tif()
    # pointcloud_obj.show_point_cloud()
    assert len(pointcloud_obj.xyz) > 0

def test_attempt_load_tif_with_multiple():
    config.update_nested(['current_map','folder_location'],'tests/test_files/multiple_tifs')

    pointcloud_obj = PointCloud()
    with pytest.raises(ValueError):
        pointcloud_obj.read_tif()


def test_find_altitude_no_add():
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloud_obj = PointCloud()
    print(pointcloud_obj.xyz)
    pointcloud_obj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
    assert pointcloud_obj.find_altitude((1,1),0) == 1

def test_altitude_incorrect_form():
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloud_obj = PointCloud()
    pointcloud_obj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
    with pytest.raises(TypeError):
        pointcloud_obj.find_altitude([1,1],0)
    with pytest.raises(TypeError):
        pointcloud_obj.find_altitude((1,1),'0')

def test_find_altitude_add_positive():
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloud_obj = PointCloud()
    pointcloud_obj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])

    assert pointcloud_obj.find_altitude((1,1),10) == 11

def test_find_altitude_add_negative():
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloud_obj = PointCloud()
    pointcloud_obj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])

    assert pointcloud_obj.find_altitude((1,1),-1) == 0

def test_find_altitude_incorrect_form():
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloud_obj = PointCloud()
    with pytest.raises(ValueError):
        pointcloud_obj.find_altitude((1,1,1),0)




def test_find_altitude_nearest():
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloud_obj = PointCloud()
    pointcloud_obj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])

    assert pointcloud_obj.find_altitude((1.1,1.1),0) == 1
    assert pointcloud_obj.find_altitude((1.5,1.5),0) == 1
    assert pointcloud_obj.find_altitude((1.9,1.9),0) == 2


def test_distance_to_nearest_point():
    config.update_nested(['distances','voxel_size_m'], '2')
    config.update_nested(['distances','interpolation_distance_m'], '1.1')
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloud_obj = PointCloud()
    pointcloud_obj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
    print(pointcloud_obj.xyz)
    assert pointcloud_obj.distance_to_nearest_point([1,1,1]) == 0
    assert pointcloud_obj.distance_to_nearest_point([1,1,0]) == 1
    assert pointcloud_obj.distance_to_nearest_point([1,1,2]) == 1
    assert round(pointcloud_obj.distance_to_nearest_point([1,1.1,1]),5) == 0.1 # Fails without round by tiny amount

def test_interpolate_line():
    config.update_nested(['distances','interpolation_distance_m'], '1')
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloud_obj = PointCloud()
    pointcloud_obj.read_tif([[0,0,0],[1,1,0],[2,2,0],[3,3,0],[4,4,0],[5,5,0]])
    points = [[0,0,0],[10,0,0]]
    route = pointcloud_obj.interpolate_route(points)

    assert route[0] == (0,0,0)
    assert route[-1] == (10,0,0)
    assert 9<len(route) <11 # Not completely important that is exactly one meter gap, allow for 10% erorr
    assert all([x[2] for x in route]) == 0 # All heights should be 0



def test_points_visible_each_other():
    config.update_nested(['speed_related','DVLOS_interpolation_m'], '1')
    config.update_nested(['current_map','folder_location'],'tif path')
    
    point1 = [0,0,0]
    point2 = [10,0,0]
    pointcloud_obj = PointCloud()
    pointcloud_obj.read_tif([[5,0,0]])
    assert pointcloud_obj.two_points_visible(point1,point2) is False # Point should not be visible
    point2 = [4,0,0]
    assert pointcloud_obj.two_points_visible(point1,point2) is True # Point should be visible
    point2 = [4.1,0,0]
    assert pointcloud_obj.two_points_visible(point1,point2) is False # Although is visible is too close to point1 (less than 1 m)


def test_points_visible_each_other_without_hieght():
    config.update_nested(['distances','DVLOS_interpolation_distance_m'], '1')
    config.update_nested(['current_map','folder_location'],'tif path')

    point1= (0,0)
    point2 = (10,0)
    pointcloud_obj = PointCloud()
    pointcloud_obj.read_tif([[5,0,0]])

    assert pointcloud_obj.two_points_visible(point1,point2) is False # Point should not be visible
    point2 = (4,0)
    assert pointcloud_obj.two_points_visible(point1,point2) is True # Point should be visible


def test_nearest_point():
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloudobj = PointCloud()
    pointcloudobj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
    assert np.array_equal(pointcloudobj.nearest_point((1.1,1.1)),np.array([1,1,1]))
    assert np.array_equal(pointcloudobj.nearest_point((1.1,1.1,0)),np.array([1,1,1]))

def test_nearest_point_more_complex():
    config.update_nested(['current_map','folder_location'],'tests/test_files/complete_test/')

    pointcloudobj = PointCloud()
    pointcloudobj.read_tif()
    assert np.array_equal(pointcloudobj.nearest_point((9.9,9.9)),
                          pointcloudobj.nearest_point((9.9,9.9,255)))

def test_distance_to_nearest_point_XYZ_and_XY():
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloudobj = PointCloud()
    pointcloudobj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
    two_point_distance = pointcloudobj.distance_to_nearest_point((1.1,1.1))
    three_point_distance = pointcloudobj.distance_to_nearest_point((1.1,1.1,1))
    assert two_point_distance == three_point_distance, "Distance should be the same for 2D and 3D points"

def test_distance_to_nearest_point_invalid():
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloudobj = PointCloud()
    pointcloudobj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
    with pytest.raises(ValueError):
        pointcloudobj.distance_to_nearest_point((1.1,1.1,1.1,1.1))
        
@patch('matplotlib.pyplot.show')
def test_show_two_d_graph_blank(mock_show):
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloudobj = PointCloud()
    pointcloudobj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
    pointcloudobj.show_two_d_graph()

@patch('matplotlib.pyplot.show')
def test_show_two_d_graph_with_data(mock_show):
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloudobj = PointCloud()
    pointcloudobj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
    cities = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]
    human_location = (2, 2, 2)
    buffer = [[[1,1], [2,2], [3,3], [4,4]]]
    area = [[[0,0], [1,1], [2,2], [3,3]]]
    pointcloudobj.show_two_d_graph(cities=cities, human_location=human_location, buffer=buffer, area_coords=area)

@patch('open3d.visualization.draw_geometries')
def test_just_pointcloud_display(mock_draw_geometries):
    config.update_nested(['current_map','folder_location'],'tif path')

    pointcloudobj = PointCloud()
    pointcloudobj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
    pointcloudobj.show_point_cloud()
    