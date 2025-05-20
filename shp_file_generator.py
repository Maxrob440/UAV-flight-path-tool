import geopandas as gpd
from shapely.geometry import LineString
import os
from Config import Config

class ShapefileGenerator:
    def __init__(self):
        self.config = Config()
        self.to_add = []
    
    def save(self):
        gdf = gpd.GeoDataFrame({
            'id': range(len(self.to_add)),
            'geometry': self.to_add
        })
        gdf.set_crs("EPSG:27700", inplace=True)
        gdf.to_file(self.config.config['io']['output_folder'] + '/Transects.shp')


class TransectGenerator(ShapefileGenerator):
    def __init__(self):
        super().__init__()
    
    def add_transect(self, transect):
        """
        Add a transect to the list of transects to be saved.
        """
        print(transect)
        tlinestring = LineString(transect)
        self.to_add.append(tlinestring)

if __name__ == "__main__":
    transect_gen = TransectGenerator()
    transect_gen.add_transect([(0, 0), (1, 1),(1,0)])
    transect_gen.add_transect([(0, 25), (1, 23),(1,11)])
    transect_gen.save()