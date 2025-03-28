import os
import two_d_graphing
import cities
import numpy as np
import pointcloud
import saveoutput

class Main():
    def __init__(self):
        self.dem_location = None
        self.no_points = None
        self.shp_location = None
        self.outline = None
        self.buffer = None
        self.pointcloud = None
        self.twod = None
        self.threed = None
    def get_all_inputs(self):
        self.dem_location = self.dem_input()
        self.shp_location = self.shp_input()
        self.no_points = self.points_input()
        self.twod, self.threed = self.type_input()
        # self.dem_location = 'Data/T691/6_OFO_T691_dtm.tif'
        # self.shp_location = 'Data/T691/2_OFO_T691_bdy.shp'
        # self.no_points = 8

    def dem_input(self):
        '''Gets the file location for the DEM file'''
        while True:
            try:
                dem_location = input(
                    '''Enter the location of the DEM file:\n(a) for default location: Data/MGAT_01201/Base_Data/01201_dem.tif\nOr the route to the DEM file\n'''
                    )
                if dem_location == 'a':
                    dem_location= 'Data/MGAT_01201/Base_Data/01201_dem.tif'
                if os.path.exists(dem_location):
                    return dem_location
                else:
                    raise FileNotFoundError
            except FileNotFoundError:
                print('Please enter a valid file location')


    def points_input(self):
        '''Gets the number of points to generate'''
        while True:
            try:
                no_points = int(input('Enter the number of points: '))
                return no_points
            except ValueError:
                print('Please enter a valid number')

    def shp_input(self):
        '''Gets the file location for the shapefile'''
        while True:
            try:
                shp_location = input(
                    '''Enter the location of the shapefile:\n(a) for default location: Data/MGAT_01201/Base_Data/2_MGAT_01201_bdy.shp\nOr the route to the shapefile\n''')
                if shp_location == 'a':
                    shp_location = 'Data/MGAT_01201/Base_Data/2_MGAT_01201_bdy.shp'
                if os.path.exists(shp_location):
                    return shp_location
                else:
                    raise FileNotFoundError
            except FileNotFoundError:
                print('Please enter a valid file location')

    def type_input(self):
        '''Gets the method of viewing 3d/2d'''
        while True:
            type_input = input('Enter the type of view (2/3): ')
            if type_input == '2':
                return True,False
            elif type_input == '3':
                return False,True
            else:
                print('Please enter a valid type (2/3)')

    def load_shp_file(self):
        '''Loads the shapefile and creates a buffer inside it'''
        shp = cities.Shp_Opener(area_path=self.shp_location)
        self.outline = shp.get_area()
        print('Successfully loaded area shapefile, number of points: ',len(self.outline))
        self.buffer = shp.get_buffered_area(-30)
        print('Successfully loaded buffered area shapefile, number of points: ',len(self.buffer))

    def visualize(self):
        '''Visualizes the area and the points'''


        outline = np.array(self.outline)
        buffer = np.array(self.buffer)
        graph_to_show = two_d_graphing.Graph2D(outline,buffer)

        
        graph_to_show.plot() # Plots area shape
        graph_to_show.plot_buffer()# Plots buffer shape


        unfiltered_points = graph_to_show.generate_random_points(150)
        graph_to_show.remove_unneeded_points(unfiltered_points,self.no_points)
        
        graph_to_show.plot_points()
        graph_to_show.find_path(False)

        # graph_to_show.plot_separated_points()

        graph_to_show.create_transcets()
        graph_to_show.find_transect_route(self.twod)

        fastest_route = graph_to_show.transect_path
        if self.threed:
            self.threed_input(fastest_route)
        if self.twod:
            graph_to_show.show()
        if not threed and not twod:
            self.threed_input(fastest_route,show=False)

    def threed_input(self,fastest_route,show=True):
        '''Visualizes the 3D point cloud'''
        print('Loading point cloud')
        print('Creating 3D point cloud')
        self.pointcloud = pointcloud.PointCloud(self.dem_location)
        self.pointcloud.read_tif()
        self.pointcloud.form_kdtree()
        self.pointcloud.display_path(fastest_route)
        self.save_output(fastest_route)
        if show:
            self.pointcloud.show()
    
    def save_output(self,fastest_route):
        fastest_route = [[x,y,z] for x,y in fastest_route for z in [self.pointcloud.find_altitude([x,y],30)]]
        connections = [[i,i+1] for i in range(len(fastest_route)-1)]

        interpolated_route,_ = self.pointcloud.create_interpolated_line(fastest_route,connections,interpolate=20)
        interpolated_route_with_plot =[]
        seen_interpolated = []
        # //TODO fix this
        for point in interpolated_route: #UGLY CODE fixing a bug elsewhere by removing duplicates
            if point not in seen_interpolated:
                seen_interpolated.append(point)
        interpolated_route = seen_interpolated        
        for point in interpolated_route:
            if point in fastest_route:
                x,y,z = point
                interpolated_route_with_plot.append([x,y,z,True])
            else:
                x,y,z = point
                interpolated_route_with_plot.append([x,y,z,False])
        # interpolated_route_with_plot = [x,y,z,True for x,y,z in interpolated_route if [x,y,z] in fastest_route]

        print('Saving output')
        saver = saveoutput.Converter(self.dem_location)
        saver.convert_mercader_to_lat_long(interpolated_route_with_plot)
        saver.convert_into_correct_form()
        saver.create_csv()





    
if __name__ == "__main__":
    main = Main()

    main.get_all_inputs()
    main.load_shp_file()
    main.visualize(twod=True,threed=False)
    # print(main.dem_location,main.no_points,main.shp_location)