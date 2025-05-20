import geopandas as gpd
from shapely.geometry import LineString
import os
from Config import Config

class ShapefileGenerator:
    def __init__(self,flight_no):
        self.config = Config()
        self.to_add = []
        self.flight_no = flight_no
    
    def save(self):
        gdf = gpd.GeoDataFrame({
            'id': range(len(self.to_add)),
            'geometry': self.to_add
        })
        gdf.set_crs("EPSG:27700", inplace=True)
        
        output_folder = self.config.config['io']['output_folder']
        specific_folder = self.config.config['io']['specific_folder_name']
        transect_file_name = self.config.config['io']['output_transect_file_name']
        output_path = os.path.join(output_folder, specific_folder)
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        file_path = os.path.join(output_path, transect_file_name)
        file_path = file_path+f'_{self.flight_no}.shp'
        gdf.to_file(file_path)


class TransectGenerator(ShapefileGenerator):

    def __init__(self,flight_no):
        super().__init__(flight_no)
    def add_transect(self, transect):
        """
        Add a transect to the list of transects to be saved.
        """
        print(transect)
        tlinestring = LineString(transect)
        self.to_add.append(tlinestring)

if __name__ == "__main__":
    transect_gen = TransectGenerator(0)
    transect_gen.add_transect([(0, 0), (1, 1),(1,0)])
    transect_gen.add_transect([(0, 25), (1, 23),(1,11)])
    transect_gen.save()