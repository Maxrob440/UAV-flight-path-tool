import geopandas as gpd
from shapely.geometry import LineString
import os
from Config import Config

class ShapefileGenerator:
    def __init__(self):
        self.config = Config()
        self.to_add = []
    
    def save(self):
        gdf = gpd.GeoDataFrame(geometry=self.to_add)
        gdf.set_crs("EPSG:4326", inplace=True)
        gdf.to_file(self.config.config['io']['output_folder'] + '/Transects.shp')


class TransectGenerator(ShapefileGenerator):
    def __init__(self):
        super().__init__()
    
    def add_transect(self, transect):
        """
        Add a transect to the list of transects to be saved.
        """
        tlinestring = LineString(transect)
        self.to_add.append(tlinestring)