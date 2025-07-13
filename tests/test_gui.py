from GUI import Gui
from Driver import Driver
from Config import Config
import os
from unittest.mock import patch

import matplotlib

matplotlib.use('Agg')  # Use a non-interactive backend for testing

def test_gui_initialization():
    gui = Gui()
    assert gui is not None
    assert gui.config is not None

def test_add_to_terminal():
    gui= Gui()
    gui.add_to_terminal("Test message")
    assert gui.terminal['text'] == "Test message"

def test_generate_picture_none():
    gui = Gui()
    output_path = gui.config.config['io']['output_folder']
    graph_name = gui.config.config['io']['graph_picture_name']
    if os.path.exists(os.path.join(output_path, graph_name)):
        os.remove(os.path.join(output_path, graph_name))
    gui.generate_picture(buffer=False,
                         area=False,
                         standing_location=False,
                         cities=False)

    assert os.path.exists(os.path.join(output_path, graph_name))
    

def test_generate_picture_with_all():
    gui = Gui()
    gui.driver = Driver()
    gui.driver.buffer_coords = [[(0, 0), (1, 1)]]
    gui.driver.area_coords = [[(-1, 0), (1, 1)]]
    gui.driver.standing_locations = [(1,1)]
    gui.driver.cities = [(-1,-1)]
    
    output_path = gui.config.config['io']['output_folder']
    graph_name = gui.config.config['io']['graph_picture_name']
    if os.path.exists(os.path.join(output_path, graph_name)):
        os.remove(os.path.join(output_path, graph_name))
    fig = gui.generate_picture(buffer=True,
                         area=True,
                         standing_location=True,
                         cities=True,
                         test=True)

    assert fig is not None
    assert len(fig.axes[0].lines)==2
    assert len(fig.axes[0].collections)==2
    assert os.path.exists(os.path.join(output_path, graph_name))

def test_load_shape_file_with_params():
    
    gui = Gui()
    gui.config.config['distances']['buffer_m'] = 1 # Default is 30 which for shape file is too much
    gui.config.save_config()
    gui.load_shapefile('tests/test_files/test_shp')
    assert len(gui.driver.buffer_coords) == 1
    assert len(gui.driver.area_coords) == 1

def test_load_shape_file_from_config():
    gui = Gui()
    gui.config.config['distances']['buffer_m'] = -1 # Default is 30 which for shape file is too much
    gui.config.config['current_map']['folder_location'] = 'tests/test_files/test_shp'
    gui.config.save_config()
    gui.load_shapefile()
    assert len(gui.driver.buffer_coords) == 1
    assert len(gui.driver.area_coords) == 1

def test_load_shape_file_from_config_no_folder():
    gui = Gui()
    # gui.config.config['current_map']['folder_location'] = ''
    gui.config.update_nested(['current_map','folder_location'],'')
    # gui.config.save_config()
    gui.load_shapefile()

    assert gui.terminal['text'] == "Invalid folder selected, please try again"

def test_load_load_shape_file_not_found():
    gui = Gui()
    gui.config.config['distances']['buffer_m'] = -10 # Default is 30 which for shape file is too much
    gui.config.update_nested(['current_map','folder_location'],'fake_path')
    gui.config.save_config()
    gui.load_shapefile()
    assert gui.terminal['text'] == "No such file or directory: 'fake_path'"


@patch('open3d.visualization.draw_geometries')
def test_integration_correct_orders(mock_draw_geometries):
    gui=Gui()
    gui.config.update_nested(['distances', 'voxel_size_m'], 1)
    gui.config.update_nested(['current_map', 'folder_location'], 'tests/test_files/complete_test')
    gui.config.update_nested(['distances', 'buffer_m'], 5) # Default is 30 which for shape file is too much
    gui.config.update_nested(['distances', 'distance_to_nearest_point_m'], 0.1)
    gui.config.update_nested(['hexagonal_grid_generation', 'grid_size_m'], 20)
    gui.config.update_nested(['speed_related', 'DVLOS_interpolation_m'], 10)
    gui.config.update_nested(['distances','height_above_ground_m'],5)
    gui.config.update_nested(['distances','transect_length_m'],2)
    gui.config.update_nested(['distances','interpolation_distance_m'],1)
    
    
    gui.load_shapefile()
    gui.generate_points()
    gui.solve_tsp()
    gui.create_clusters()
    gui.generate_transects()
    gui.solve_transects()
    gui.save_output()
    gui.view_threed()

    assert gui.driver.transects is not None
    assert gui.driver.transect_path is not None
    assert isinstance(gui.driver.transect_path[0][1],bool)
    assert len(gui.driver.transect_path[0][0]) == 2


@patch('open3d.visualization.draw_geometries')
def test_view_3d_TSP_path_without_transects(mock_draw_geometries):
    gui = Gui()
    gui.config.update_nested(['distances', 'voxel_size_m'], 0.1)
    gui.config.update_nested(['current_map', 'folder_location'], 'tests/test_files/complete_test')
    gui.config.update_nested(['distances', 'buffer_m'], 5) # Default is 30 which for shape file is too much
    gui.config.update_nested(['distances', 'distance_to_nearest_point_m'], 0.1)
    gui.config.update_nested(['hexagonal_grid_generation', 'grid_size_m'], 20)
    
    
    gui.load_shapefile()
    gui.generate_points()
    gui.solve_tsp()
    gui.view_threed()
    assert gui.terminal['text'] == '3D view generated'

def test_save_output_of_just_TSP_path():
    gui = Gui()
    gui.config.update_nested(['distances', 'voxel_size_m'], 0.1)
    gui.config.update_nested(['current_map', 'folder_location'], 'tests/test_files/complete_test')
    gui.config.update_nested(['distances', 'buffer_m'], 5) # Default is 30 which for shape file is too much
    gui.config.update_nested(['distances', 'distance_to_nearest_point_m'], 0.1)
    gui.config.update_nested(['hexagonal_grid_generation', 'grid_size_m'], 20)
    
    
    # gui.config.config['distances']['voxel_size_m'] = 0.1
    # gui.config.config['current_map']['folder_location'] = 'tests/test_files/complete_test'
    # gui.config.config['distances']['buffer_m'] = 5 # Default is 30 which for shape file is too much
    # gui.config.config['distances']['distance_to_nearest_point_m'] = 0.1
    # gui.config.config['distances']['grid_size_m'] = 20

    # gui.config.save_config()
    gui.load_shapefile()
    gui.generate_points()
    gui.solve_tsp()
    gui.save_output()

    # assert os.path.exists(os.path.join(gui.config.config['io']['output_folder'], 'TSP_path.txt'))

@patch('open3d.visualization.draw_geometries')
def test_transect_generation_after_viewing_threed(mock_draw_geometries):
    gui= Gui()
    gui.config.config['current_map']['folder_location'] = 'tests/test_files/complete_test'
    gui.config.config['distances']['voxel_size_m'] = 0.1
    gui.config.config['distances']['buffer_m'] = 5 # Default is 30 which for shape file is too much
    gui.config.config['distances']['distance_to_nearest_point_m'] = 0.1
    gui.config.config['hexagonal_grid_generation']['grid_size_m'] = 20

    gui.config.save_config()
    gui.load_shapefile()
    gui.generate_points()
    gui.solve_tsp()
    gui.create_clusters()
    gui.view_threed()
    gui.generate_transects()
    assert gui.terminal['text'] == 'Transects generated'

@patch('open3d.visualization.draw_geometries')
def test_transect_generation_after_viewing_threed_with_transect_route(mock_draw_geometries):
    gui= Gui()
    gui.config.config['current_map']['folder_location'] = 'tests/test_files/complete_test'
    gui.config.config['distances']['voxel_size_m'] = 0.1
    gui.config.config['distances']['buffer_m'] = 5 # Default is 30 which for shape file is too much
    gui.config.config['distances']['distance_to_nearest_point_m'] = 0.1
    gui.config.config['hexagonal_grid_generation']['grid_size_m'] = 20

    gui.config.save_config()
    gui.load_shapefile()
    gui.generate_points()
    gui.solve_tsp()
    gui.create_clusters()
    gui.generate_transects()
    gui.solve_transects()
    gui.view_threed()
    gui.generate_transects()
    assert gui.terminal['text'] == 'Transects generated'

def test_loading_standing_locations():
    gui = Gui()
    gui.config.config['current_map']['folder_location'] = 'tests/test_files/complete_test'
    gui.config.config['distances']['voxel_size_m'] = 0.1
    gui.config.config['distances']['buffer_m'] = 5 # Default is 30 which for shape file is too much
    gui.config.config['distances']['distance_to_nearest_point_m'] = 0.1
    gui.config.save_config()

    gui.load_shapefile()
    gui.generate_points()
    
    assert gui.driver.standing_locations is not None
    assert len(gui.driver.standing_locations) == 1
    assert gui.driver.standing_locations[0][0:2] == (50,50)

def test_route_generation_after_moving_standing_location():
    gui = Gui()
    gui.config.config['current_map']['folder_location'] = 'tests/test_files/complete_test_with_multiple_standing_locations'
    gui.config.config['distances']['buffer_m'] = 5 # Default is 30 which for shape file is too much
    gui.config.config['distances']['distance_to_nearest_point_m'] = 0.1
    gui.config.config['hexagonal_grid_generation']['grid_size_m'] = 20
    
    gui.config.save_config()

    gui.load_shapefile()
    gui.generate_points()
    gui.solve_tsp()

    gui.load_next_standing_location()
    gui.generate_points()
    gui.solve_tsp()

    current_location = gui.driver.standing_locations[gui.driver.current_standing_id]

    assert current_location[:2] in gui.driver.best_path_coords

def test_route_generation_standing_outside_area():
    gui = Gui()
    gui.config.config['current_map']['folder_location'] = 'tests/test_files/standing_locations_outside_shp'
    gui.config.config['distances']['buffer_m'] = 5 # Default is 30 which for shape file is too much
    gui.config.config['distances']['distance_to_nearest_point_m'] = 0.1
    gui.config.config['hexagonal_grid_generation']['grid_size_m'] = 20
    
    gui.config.save_config()

    gui.load_shapefile()
    gui.generate_points()
    
    
    gui.solve_tsp()

    assert len(gui.driver.best_path_coords) >1

def test_double_press_of_tsp():
    gui = Gui()
    gui.config.update_nested(['current_map','folder_location'], 'tests/test_files/complete_test')
    gui.config.update_nested(['distances','buffer_m'], 5) # Default is 30
    gui.config.update_nested(['point_creation','random_point_generation'],True)
    gui.config.update_nested(['random_point_generation','distance_to_nearest_point_m'], 0)
    gui.config.update_nested(['random_point_generation','number_of_points_per_area'],16)


    gui.load_shapefile()
    gui.generate_points()
    gui.solve_tsp()
    gui.solve_tsp()  # Simulate double press



    assert gui.terminal['text'] == 'TSP solved'

def test_flyable_area():
    gui = Gui()
    gui.config.update_nested(['current_map','folder_location'], 'example data/bryanston')
    gui.config.update_nested(['point_creation','flyable_areas'],True)

    gui.load_shapefile()
    gui.generate_points()
    gui.solve_tsp()
    gui.create_clusters()
    gui.generate_transects()
    gui.solve_transects()




    
