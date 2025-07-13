import pytest
import numpy as np
from Driver import Driver
from Config import Config

config = Config()
config.set_default()


def test_driver_initialization_no_params():
    driver = Driver()
    assert driver is not None, "Driver should be initialized successfully"

def test_loading_of_config_file():
    config.config['current_map']['folder_location'] = 'TEST'
    config.save_config()
    driver = Driver()

    assert driver.config.config['current_map']['folder_location'] == 'TEST', "Driver should load the config file correctly"

def test_loading_standing_locations_xyz():
    driver = Driver()
    config.update_nested(['current_map','folder_location'],'tests/test_files/single_standing_location')
    driver.load_standing_locations()
    assert driver.standing_locations == [(1.1,1.1,0)], "Driver should load standing locations correctly"
    driver = Driver()

    config.update_nested(['current_map','folder_location'],'tests/test_files/multiple_standing_locations')
    driver.load_standing_locations()
    assert driver.standing_locations == [(0,0,0), (1,1,1)], "Driver should load multiple standing locations correctly"
    
    driver = Driver()

    config.update_nested(['current_map','folder_location'],'tests/test_files/empty_standing_location')

    driver.load_standing_locations()
    assert driver.standing_locations == [], "Driver should handle empty standing locations correctly"

def test_XY_standing_locations():
    config.config['distances']['human_height_above_ground_m'] = 0
    config.update_nested(['current_map','folder_location'],'tests/test_files/XY_standing_location_multiple')

    config.save_config()
    driver = Driver()
    driver.pointcloudholder.read_tif([(0,0,0), (1,1,1)])
    driver.load_standing_locations()
    assert driver.standing_locations == [(0,0,0), (1,1,1)], "Driver should load XY standing locations correctly"

def test_loading_standing_locations_with_no_valid_txt_file():
    config.update_nested(['current_map','folder_location'],'tests/test_files/no_available_standing_locations')

    driver = Driver()
    driver.load_standing_locations()
    assert driver.standing_locations == []

def test_loading_standing_locations_with_non_valid_txt_file():
    config.update_nested(['current_map','folder_location'],'tests/test_files/invalid_coords_XY')

    driver = Driver()
    driver.load_standing_locations()
    assert driver.standing_locations == []

    config.update_nested(['current_map','folder_location'],'tests/test_files/invalid_coords_XYZ')
    driver = Driver()
    driver.load_standing_locations()
    assert driver.standing_locations == []
    

def test_load_standing_locations_from_params():
    driver = Driver()
    driver.load_standing_locations([(0,0,0), (1,1,1)])
    assert driver.standing_locations == [(0,0,0), (1,1,1)]


def test_generate_points_from_parameters():
    driver = Driver()
    driver.generate_points_random(points=[(0,0,0), (1,1,1)])
    assert driver.cities == [(0,0,0), (1,1,1)]

    driver.generate_points_random(starting_point=(-1,-1,-1),points=[(0,0,0), (1,1,1)])
    assert driver.cities == [(-1,-1,-1),(0,0,0), (1,1,1)], "Driver should generate points correctly"


def test_load_shape_file_from_tif():
    config.update_nested(['current_map','folder_location'],'tests/test_files/test_shp')

    driver = Driver()
    driver.load_shp_file()


def test_generate_points_inside_square():
    config.update_nested(['random_point_generation','distance_to_nearest_point_m'],0)
    config.update_nested(['random_point_generation','number_of_points_per_area'],8)

    driver = Driver()
    driver.buffer_coords=[[(0,0),(1,1),(1,0),(0,1)]]
    driver.generate_points_random()
    assert len(driver.cities)==8
    for point in driver.cities:
        assert point[0] >= 0 and point[0] <= 1
        assert point[1] >= 0 and point[1] <= 1

def test_generate_points_inside_square_forcing_distance_below_minimum():
    config.update_nested(['random_point_generation','distance_to_nearest_point_m'],100)
    config.update_nested(['random_point_generation','number_of_points_per_area'],8)

    # config.config['distances']['distance_to_nearest_point_m'] = 100
    # config.config['distances']['number_of_points_per_area'] =8
    # config.save_config()

    driver = Driver()
    driver.buffer_coords=[[(0,0),(1,1),(1,0),(0,1)]]
    driver.generate_points_random()
    assert len(driver.cities)==1



def test_generate_points_inside_square_with_DVLOS():
    # config.config['distances']['number_of_points_per_area'] =8
    config.config['distances']['voxel_size_m'] = 1
    config.config['speed_related']['DVLOS_m'] = 1
    config.config['distances']['interpolation_distance_m'] = 1
    # config.config['distances']['distance_to_nearest_point_m'] = 0
    config.config['distances']['height_above_ground_m'] = 0
    config.config['distances']['human_height_above_ground_m']=0
    config.config['distances']['grid_size_m'] = 1
    config.save_config()



    driver= Driver()
    driver.buffer_coords=[[(0,0), (10,0), (10,5), (0,5),(0,0)]]
    tif = [(5,x,0) for x in np.linspace(0,5,10)]
    driver.pointcloudholder.read_tif(tif)
    driver.folder_path='tests/test_files/single_standing_location'
    driver.load_standing_locations()

    # standing_location = driver.standing_locations[0]
    driver.generate_points_standard()
    # driver.pointcloudholder.show_two_d_graph(driver.cities, standing_location,buffer=driver.buffer_coords)
    # driver.pointcloudholder.show_point_cloud(driver.cities,standing_location)
    # driver.generate_points_random()

    for point in driver.cities:
        assert point[0] >= 0 and point[0] <= 10
        assert point[1] >= 0 and point[1] <= 5

def test_solve_tsp_brute():
    driver = Driver()
    driver.cities = [(0,0,0), (1,1,1), (2,2,2)]
    driver.pointcloudholder.read_tif(driver.cities)
    driver.solve_tsp()
    assert driver.best_path_coords[0] == (0,0,0)
    assert driver.best_path_coords[-1] == (0,0,0)
    assert len(driver.best_path_coords) == 4

def test_solve_tsp_ant():
    driver = Driver()
    driver.cities = [(0,0,0), (1,1,1), (2,2,2), (3,3,3), (4,4,4), (5,5,5), (6,6,6), (7,7,7), (8,8,8), (9,9,9)]
    driver.pointcloudholder.read_tif(driver.cities)
    driver.solve_tsp()
    assert driver.best_path_coords[0] == (0,0,0)
    assert driver.best_path_coords[-1] == (0,0,0)
    assert len(driver.best_path_coords) == 11 #ten points and first one twice

def test_generate_transects():
    driver = Driver()
    driver.buffer_coords=[[(0,0),(50,50),(50,0),(0,50)]]
    driver.cities = [(1,1),(2,2)]
    driver.clustered = [[(1,1),(2,2)]]
    driver.create_transects()
    print(driver.transects)
    assert len(driver.transects[0]) == 2
    assert (1,1) in driver.transects[0]
    assert len(driver.transects[0][(1,1)]) == 1 #First point doessnt get a transect
    assert len(driver.transects[0][(1,1)][0]) == 2

    assert (2,2) in driver.transects[0]
    assert len(driver.transects[0][(2,2)]) == 7 #All other points do
    assert len(driver.transects[0][(2,2)][0]) == 2
    assert driver.transects[0][(2,2)][1]==(2,2)

        
def test_solve_transect_route():
    driver=Driver()
    driver.transects=[{(1,1,0):[(1,1)],
                      (2,2,0):[(2,2),(3,3),(4,4)]}]
    driver.best_path_coords=[(1,1,0),(2,2,0)]
    driver.cluster_points()
    driver.solve_transect_route()
    assert len(driver.transect_path)==4
    assert driver.transect_path[0][0]==(1,1)
    assert driver.transect_path[0][1] is False
    assert driver.transect_path[-1][0]==(2,2) or driver.transect_path[-3][0]==(2,2)
    assert driver.transect_path[-2][0]==(3,3)
    for point in driver.transect_path[1:]:
        assert point[1] is True
        
def test_solve_transect_route_more_than_twenty_points():
    config.update_nested(['clustering','points_per_cluster'],25)

    driver = Driver()
    driver.buffer_coords=[[(0,0),(200,0),(200,200),(0,200),(0,0)]]
    cities = [(i, i, 0) for i in range(21)]
    driver.cities = cities
    driver.best_path_coords = [(i, i, 0) for i in range(21)]
    driver.cluster_points()
    driver.pointcloudholder.read_tif(cities)
    driver.create_transects()
    driver.solve_transect_route()
    assert len(driver.transect_path) > 20, "Transect path should contain more than 20 points" 

def test_solve_transect_route_with_fillers():
    driver=Driver()
    driver.transects=[{(1,1,0):[(1,1)],
                      (2,2,0):[(2,2),(3,3),(4,4)],
                      (5,5,0):[(5,5)]}]
    driver.best_path_coords=[(1,1,0),(2,2,0),(5,5,0)]
    driver.solve_transect_route()
    assert len(driver.transect_path)==5
    assert driver.transect_path[-1]==[(5,5),False]

def test_route_length():
    driver = Driver()
    assert round(driver.route_length([[(0,0),False], [(1,1),True], [(2,2),False]]),1)==2.8
    assert driver.route_length([[(-1,0),False],[(0,0),True]])==1

def test_clustering_sizes():
    config.update_nested(['clustering','points_per_cluster'], 2)
    driver = Driver()
    driver.cities = [(0,0,0), (1,1,1), (2,2,2), (3,3,3)]
    driver.best_path_coords = [(0,0,0), (1,1,1), (2,2,2), (3,3,3)]
    driver.cluster_points()
    
    assert len(driver.clustered) == 2, "Clustering should create two clusters"
    assert len(driver.clustered[0]) == 2, "First cluster should contain two points"
    assert len(driver.clustered[1]) == 2, "Second cluster should contain two points"
 
    config.update_nested(['clustering','points_per_cluster'], 4)
    driver.cities = [(0,0,0), (1,1,1), (2,2,2), (3,3,3), (4,4,4)]
    driver.best_path_coords = [(0,0,0), (1,1,1), (2,2,2), (3,3,3), (4,4,4)]
    driver.cluster_points()
    assert len(driver.clustered) == 2, "Clustering should create two clusters"
    assert len(driver.clustered[0]) == 4, "First cluster should contain four points"
    assert len(driver.clustered[1]) == 1, "Second cluster should contain one point"

def test_switching_clusters():
    config.update_nested(['clustering','points_per_cluster'], 2)
    driver = Driver()
    driver.cities = [(0,0,0), (1,1,1), (2,2,2), (3,3,3)]
    driver.best_path_coords = [(0,0,0), (1,1,1), (2,2,2), (3,3,3)]
    driver.cluster_points()
    driver.cycle_cluster()
    assert len(driver.clustered) == 2, "Clustering should create two clusters"
    expected = [(2,2,2), (3,3,3)]
    assert driver.current_cluster == 1
    assert driver.clustered[1] == expected, "Cities should be switched to the next cluster"

    driver.cycle_cluster()
    assert driver.current_cluster == 0, "Current cluster should switch back to the first cluster"
    expected = [(0,0,0), (1,1,1)]
    assert driver.clustered[0] == expected, "Cities should switch back to the first cluster"

def test_clustering_with_standing_location():
    config.update_nested(['clustering','points_per_cluster'], 2)
    driver = Driver()
    driver.standing_locations = [(0,0,0)]
    driver.cities = [(1,1,1), (2,2,2), (3,3,3)]
    driver.best_path_coords = [(0,0,0),(1,1,1), (2,2,2), (3,3,3)]
    driver.cluster_points()
    for cluster in driver.clustered:
        assert cluster[0] == (0,0), "Standing location should be included in the first cluster"
    
    assert len(driver.clustered[0])==3, 'Cluster should have two points and the starting location'
    assert len(driver.clustered[1])== 2, 'Cluster should have one point and the starting location'

def test_a_star_when_in_clustering():
    config.update_nested(['clustering','points_per_cluster'], 2)
    driver = Driver()
    driver.buffer_coords =[[(0,0),(10,10)]]
    driver.cities = [(10,0)]
    driver.best_path_coords = [(10,0)]
    driver.standing_locations = [(0,10)]
    driver.cluster_points()

    assert len(driver.clustered[0]) > 2, 'Cluster should have one point and the starting location, and contain the path between them around the obstacle'

def test_correct_generation_of_transects_in_clusters():
    config.update_nested(['clustering','points_per_cluster'], 2)
    driver = Driver()
    driver.cities = [(1,1),(2,2)]
    driver.standing_locations = [(0,0,0)]
    driver.clustered = [[(0,0),(1,1)],[(0,0),(2,2)]]
    driver.create_transects()
    print(driver.transects)
    assert driver.transects[0][(0,0)]==[(0,0)]
    assert len(driver.transects[0][(1,1)])==7
    assert len(driver.transects[1])==0,'Transects for the second cluster should not be created'

    driver.cycle_cluster()
    driver.create_transects()
    assert len(driver.transects[0])==0
    assert len(driver.transects[1])==2
    assert driver.transects[1][(0,0)]==[(0,0)]
    assert len(driver.transects[1][(2,2)])==7


