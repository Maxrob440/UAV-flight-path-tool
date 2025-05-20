import pytest
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
    driver = Driver(folder_path='tests/test_files/single_standing_location')
    driver.load_standing_locations()
    assert driver.standing_locations == [(0,0,0)], "Driver should load standing locations correctly"


    driver = Driver(folder_path='tests/test_files/multiple_standing_locations')
    driver.load_standing_locations()
    assert driver.standing_locations == [(0,0,0), (1,1,1)], "Driver should load multiple standing locations correctly"

    driver = Driver(folder_path='tests/test_files/empty_standing_location')
    driver.load_standing_locations()
    assert driver.standing_locations == [], "Driver should handle empty standing locations correctly"

def test_XY_standing_locations():
    config.config['distances']['human_height_above_ground_m'] = 0
    config.save_config()
    driver = Driver(folder_path='tests/test_files/XY_standing_location_multiple')
    driver.pointcloudholder.read_tif([(0,0,0), (1,1,1)])
    driver.load_standing_locations()
    assert driver.standing_locations == [(0,0,0), (1,1,1)], "Driver should load XY standing locations correctly"

def test_loading_standing_locations_with_no_valid_txt_file():
    driver = Driver(folder_path='tests/test_files/no_available_standing_locations')
    driver.load_standing_locations()
    assert driver.standing_locations == []

def test_loading_standing_locations_with_non_valid_txt_file():
    driver = Driver(folder_path='tests/test_files/invalid_coords_XY')
    driver.load_standing_locations()
    assert driver.standing_locations == []
    
    driver = Driver(folder_path='tests/test_files/invalid_coords_XYZ')
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
    driver = Driver('tests/test_files/test_shp')
    driver.load_shp_file()


def test_generate_points_inside_square():
    config.config['distances']['distance_to_nearest_point_m'] = 0
    config.config['distances']['number_of_points_per_area'] =8
    config.save_config()

    driver = Driver()
    driver.buffer_coords=[[(0,0),(1,1),(1,0),(0,1)]]
    driver.generate_points_random()
    assert len(driver.cities)==8
    for point in driver.cities:
        assert point[0] >= 0 and point[0] <= 1
        assert point[1] >= 0 and point[1] <= 1

def test_generate_points_inside_square_forcing_distance_below_minimum():
    config.config['distances']['distance_to_nearest_point_m'] = 100
    config.config['distances']['number_of_points_per_area'] =8
    config.save_config()

    driver = Driver()
    driver.buffer_coords=[[(0,0),(1,1),(1,0),(0,1)]]
    driver.generate_points_random()
    assert len(driver.cities)==1



def test_generate_points_inside_square_with_DVLOS():
    # config.config['distances']['number_of_points_per_area'] =8
    # config.config['speed_related']['interpolation_distance_m'] = 0.1
    config.config['speed_related']['DVLOS_m'] = 1
    config.config['distances']['interpolation_distance_m'] = 1
    config.config['distances']['distance_to_nearest_point_m'] = 0
    config.config['distances']['height_above_ground_m'] = 0
    config.config['distances']['human_height_above_ground_m']=0
    config.save_config()

    driver= Driver()
    driver.buffer_coords=[[(0,0),(10,1),(10,0),(0,1)]]
    driver.pointcloudholder.read_tif([(5,0,0),(5,0.1,0),(5,0.25,0),(5,0.4,0),(5,0.5,0),(5,0.6,0),(5,0.75,0),(5,0.9,0),(5,1,0)])
    driver.folder_path='tests/test_files/single_standing_location'
    driver.load_standing_locations()

    driver.generate_points_random()
    # standing_location = driver.standing_locations[0]
    # driver.pointcloudholder.show_point_cloud(driver.cities,standing_location)

    assert len(driver.cities)==8
    for point in driver.cities:
        assert point[0] >= 0 and point[0] <= 5
        assert point[1] >= 0 and point[1] <= 1

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
    driver.create_transects([(1,1),(2,2)])
    print(driver.transects)
    assert len(driver.transects) == 2
    assert (1,1) in driver.transects
    assert len(driver.transects[(1,1)]) == 1 #First point doessnt get a transect
    assert len(driver.transects[(1,1)][0]) == 2

    assert (2,2) in driver.transects
    assert len(driver.transects[(2,2)]) == 3 #All other points do
    assert len(driver.transects[(2,2)][0]) == 2
    assert driver.transects[(2,2)][0]==(2,2)
    for transect in driver.transects[(2,2)]:
        assert transect[0]<50 and transect[0]>0
        assert transect[1]<50 and transect[1]>0
        
def test_solve_transect_route():
    driver=Driver()
    driver.transects={(1,1,0):[(1,1)],
                      (2,2,0):[(2,2),(3,3),(4,4)]}
    driver.best_path_coords=[(1,1,0),(2,2,0)]
    driver.solve_transect_route()
    assert len(driver.transect_path)==4
    assert driver.transect_path[0][0]==(1,1)
    assert driver.transect_path[0][1] is False
    assert driver.transect_path[-1][0]==(2,2) or driver.transect_path[-3][0]==(2,2)
    assert driver.transect_path[-2][0]==(3,3)
    for point in driver.transect_path[1:]:
        assert point[1] is True

def test_solve_transect_route_with_fillers():
    driver=Driver()
    driver.transects={(1,1,0):[(1,1)],
                      (2,2,0):[(2,2),(3,3),(4,4)],
                      (5,5,0):[(5,5)]}
    driver.best_path_coords=[(1,1,0),(2,2,0),(5,5,0)]
    driver.solve_transect_route()
    assert len(driver.transect_path)==5
    assert driver.transect_path[-1]==[(5,5),False]

def test_route_length():
    driver = Driver()
    assert round(driver.route_length([[(0,0),False], [(1,1),True], [(2,2),False]]),1)==2.8
    assert driver.route_length([[(-1,0),False],[(0,0),True]])==1