import geopandas as gpd

class Shp_Opener:
    def __init__(self,shp_path=None,trans_path=None,area_path=None):
        self.path = shp_path
        # self.plots = gpd.read_file(shp_path)
        # self.trans_path = trans_path
        self.area_path = area_path

    def get_trans(self):
        trans_points = []
        points = gpd.read_file(self.trans_path)
        for point in points['geometry']:
            coords = list(point.coords)#
            for cood in coords:
                trans_points.append([cood[0],cood[1]])
            # trans_points.append([coords[0][0],coords[0][1]])
        return trans_points

    def get_points(self):        
        line_points = []
        plots = gpd.read_file(self.path)
        for points in plots['geometry']:
            x=points.x
            y=points.y
            # print(x,y)
            # print(z)
            line_points.append([x,y])
        return line_points
    
    def get_area(self):
        area_points = []
        plots = gpd.read_file(self.area_path)
        
        for geom in plots['geometry']:
            if geom.geom_type == 'Polygon':  # Ensure it's a Polygon
                x, y = geom.exterior.xy  # Extract x and y coordinate lists
                area_points.extend(zip(x, y))  # Convert them into (x, y) pairs
            elif geom.geom_type == 'MultiPolygon':  # If MultiPolygon, handle separately
                for poly in geom.geoms:
                    x, y = poly.exterior.xy
                    area_points.extend(zip(x, y))
        self.get_buffered_area(36)
        # print(area_points)
        return area_points
    
    def get_buffered_area(self,buffer):
        buffered_area_points = []
        area = gpd.read_file(self.area_path)
        area['geometry'] = area['geometry'].buffer(buffer)
        # print(area)
        for geom in area['geometry']:
            if geom.geom_type == 'Polygon':  # Ensure it's a Polygon
                x, y = geom.exterior.xy  # Extract x and y coordinate lists
                buffered_area_points.extend(zip(x, y))  # Convert them into (x, y) pairs
            elif geom.geom_type == 'MultiPolygon':  # If MultiPolygon, handle separately
                for poly in geom.geoms:
                    x, y = poly.exterior.xy
                    buffered_area_points.extend(zip(x, y))
        return buffered_area_points
        

 
if __name__ == '__main__':
    main = Shp_Opener('Data/Data/M345/Additional_Data/1_M345_plots.shp','Data/Data/M345/Additional_Data/2_transects_M345.shp')
    print(len(main.get_trans()))
    print(len(main.get_points()))