from Pointcloud import PointCloud
from Config import Config
import pytest

config = Config()
config.load_config()
config.config['distances']['height_above_ground_m'] = 0
config.save_config()

def test_load_tif():
    pointcloud_obj = PointCloud('tests/test_files/tif')
    pointcloud_obj.read_tif()
    # pointcloud_obj.show_point_cloud()
    assert len(pointcloud_obj.xyz) > 0


def test_find_altitude_no_add():
    pointcloud_obj = PointCloud('tif path')

    print(pointcloud_obj.xyz)
    pointcloud_obj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
    assert pointcloud_obj.find_altitude((1,1),0) == 1


def test_find_altitude_add_positive():
    pointcloud_obj = PointCloud('tif path')
    pointcloud_obj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])

    assert pointcloud_obj.find_altitude((1,1),10) == 11

def test_find_altitude_add_negative():
    pointcloud_obj = PointCloud('tif path')
    pointcloud_obj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])

    assert pointcloud_obj.find_altitude((1,1),-1) == 0

def test_find_altitude_incorrect_form():
    pointcloud_obj = PointCloud('tif path')
    with pytest.raises(ValueError):
        pointcloud_obj.find_altitude((1,1,1),0)




def test_find_altitude_nearest():
    pointcloud_obj = PointCloud('tif path')
    pointcloud_obj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])

    assert pointcloud_obj.find_altitude((1.1,1.1),0) == 1
    assert pointcloud_obj.find_altitude((1.5,1.5),0) == 1
    assert pointcloud_obj.find_altitude((1.9,1.9),0) == 2


def test_distance_to_nearest_point():
    pointcloud_obj = PointCloud('tif path')
    pointcloud_obj.read_tif([[0,0,0],[1,1,1],[2,2,2],[3,3,3],[4,4,4]])
    print(pointcloud_obj.xyz)
    assert pointcloud_obj.distance_to_nearest_point([1,1,1]) == 0
    assert pointcloud_obj.distance_to_nearest_point([1,1,0]) == 1
    assert pointcloud_obj.distance_to_nearest_point([1,1,2]) == 1
    assert round(pointcloud_obj.distance_to_nearest_point([1,1.1,1]),5) == 0.1 # Fails without round by tiny amount

def test_interpolate_line():
    config.config['distances']['interpolation_distance_m'] = 1
    config.save_config()
    pointcloud_obj = PointCloud('tif path')
    pointcloud_obj.read_tif([[0,0,0],[1,1,0],[2,2,0],[3,3,0],[4,4,0],[5,5,0]])
    points = [[0,0,0],[10,0,0]]
    route = pointcloud_obj.interpolate_route(points)

    assert route[0] == (0,0,0)
    assert route[-1] == (10,0,0)
    assert 9<len(route) <11 # Not completely important that is exactly one meter gap, allow for 10% erorr
    assert all([x[2] for x in route]) == 0 # All heights should be 0



def test_points_visible_each_other():
    point1 = [0,0,0]
    point2 = [10,0,0]
    pointcloud_obj = PointCloud('tif_path')
    pointcloud_obj.read_tif([[5,0,0]])
    assert pointcloud_obj.two_points_visible(point1,point2) is False # Point should not be visible
    point2 = [4,0,0]
    assert pointcloud_obj.two_points_visible(point1,point2) is True # Point should be visible
    point2 = [4.1,0,0]
    assert pointcloud_obj.two_points_visible(point1,point2) is False # Although is visible is too close to point1 (less than 1 m)


def test_points_visible_each_other_without_hieght():
    point1= (0,0)
    point2 = (10,0)
    pointcloud_obj = PointCloud('tif_path')
    pointcloud_obj.read_tif([[5,0,0]])

    assert pointcloud_obj.two_points_visible(point1,point2) is False # Point should not be visible
    point2 = (4,0)
    assert pointcloud_obj.two_points_visible(point1,point2) is True # Point should be visible


