from saveoutput import Converter
import pandas as pd
from Config import Config
import shutil
import os
import geopandas as gpd
from shapely.geometry import LineString, Point



def test_convert_mercader_to_lat_long():
    pass

def test_create_bearings():
    converter = Converter("dummy.tif")
    converter.lat_long_coords=[(50.71496966354666, -3.5107196508254517,0,True),(51.49133881869516, -0.18442072115086947,0,True)]
    bearings = converter.create_bearings()
    assert len(bearings) == 2
    assert bearings[1] == 0
    assert round(bearings[0],0) == 68

def test_create_csv():
    config = Config()
    config.update_nested(['io', 'output_folder'], 'TEST_OUTPUT')
    config.update_nested(['io', 'specific_folder_name'], 'Testing_saving_csv')
    config.update_nested(['io', 'output_CSV_file_name'], 'test_square_path_')

    converter = Converter("dummy.tif")
    converter.lat_long_coords=[(50.71496966354666, -3.5107196508254517,0,True),(51.49133881869516, -0.18442072115086947,0,False)]
    converter.bearings=[68,0]
    converter.create_csv("1",'1')
    df = pd.read_csv("TEST_OUTPUT/Testing_saving_csv/test_square_path_1_1.csv")
    assert len(df) == 2
    assert df.iloc[0]['latitude'] == 50.71496966354666
    assert round(df.iloc[0]['longitude'],5) == round(-3.5107196508254517,5)
    assert df.iloc[0]['altitude(m)'] == 0
    assert round(df.iloc[0]['heading'],0) == 68
    assert df.iloc[0]['photo_timeinterval'] ==2
    assert df.iloc[0]['speed(m/s)'] == 0.8
    
    assert df.iloc[1]['latitude'] == 51.49133881869516
    assert round(df.iloc[1]['longitude'],5) == round(-0.18442072115086947,5)
    assert df.iloc[1]['altitude(m)'] == 0
    assert round(df.iloc[1]['heading'],0) == 0
    assert df.iloc[1]['photo_timeinterval'] ==-1
    assert df.iloc[1]['speed(m/s)'] == 0

    # Clean up
    shutil.rmtree("TEST_OUTPUT/Testing_saving_csv/")
        
def test_save_shp():
    config = Config()
    config.update_nested(['io', 'output_folder'], 'TEST_OUTPUT')
    config.update_nested(['io', 'specific_folder_name'], 'Testing_saving_shp')
    config.update_nested(['io', 'output_transect_file_name'], 'test_transects')

    converter = Converter("dummy.tif")
    transects = [{
        '(0,0)': [(0,0),(1,1),(2,1)],
        '(1,1)': [(1,1),(2,2),(3,3)],
    }]
    converter.save_transects(transects,'1',0)

    assert os.path.exists("TEST_OUTPUT/Testing_saving_shp/test_transects_1.shp")

    data = gpd.read_file("TEST_OUTPUT/Testing_saving_shp/test_transects_1.shp")
    assert len(data) == 2
    assert data.iloc[0]['geometry'] == LineString([(0,0), (1,1), (2, 1)])
    assert data.iloc[1]['geometry'] == LineString([(1,1), (2,2), (3, 3)])

    # Clean up
    shutil.rmtree("TEST_OUTPUT/Testing_saving_shp/")

def test_save_dem_shp():
    config = Config()
    config.update_nested(['io', 'output_folder'], 'TEST_OUTPUT')
    config.update_nested(['io', 'specific_folder_name'], 'Testing_copying_dem_shp')

    converter = Converter("dummy.tif")
    folder_path = "tests/test_files/complete_test"
    converter.save_dem_shp(folder_path)
    tif_name = [path for path in os.listdir(folder_path) if path.endswith('.tif')][0]
    shp_name = [path for path in os.listdir(folder_path) if path.endswith('.shp')][0]
    assert os.path.exists(f"TEST_OUTPUT/Testing_copying_dem_shp/{shp_name}")
    assert os.path.exists(f"TEST_OUTPUT/Testing_copying_dem_shp/{tif_name}")


    # Clean up
    shutil.rmtree("TEST_OUTPUT/Testing_copying_dem_shp/")

