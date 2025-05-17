
import os
import random
import numpy as np
import geopandas as gpd
import math
from pprint import pprint

from Config import Config
from Pointcloud import PointCloud
from pathfinding import Pathfinder,AntColony,BruteForce
import matplotlib.pyplot as plt

Coordinate = tuple[float, float]
Coordinates = list[Coordinate]
NumpyCoordinates = np.ndarray[Coordinate]
class Driver:
    '''
    The main driver class for the program.\n
    This class handles the loading of data, generating points, and solving the TSP problem.\n
    It uses the PointCloud class to handle point cloud data and the Pathfinder class to solve the TSP problem.\n
    It also uses the Config class to handle configuration settings.\n
    '''
    def __init__(self, 
                 folder_path:str='',
                 number_points:int=8,
                 number_areas:int=1,)->None:
        '''
        Constructor for the Driver class.\n
        Parameters:
        folder_path (str): Path to the folder containing input data.
        number_points (int): Number of points to be generated.
        number_areas (int): Number of areas to be processed.
        '''
        self.config = Config()
        self.config.load_config()
        self.pointcloudholder = PointCloud(folder_path)
        self.folder_path = folder_path
        self.number_points = number_points
        self.number_areas = number_areas
        self.standing_locations = []
        self.area_coords = []
        self.buffer_coords = []
        self.cities = []
        self.best_path_coords = []
        self.best_path_coords_interpolated = []
        self.transects = []
        self.transect_path=[]
        self.current_buffer = 0
        self.current_standing_id = 0
    
    def load_shp_file(self)->Coordinates:
        '''
        Loads the shapefile and creates the outline and buffer.
        '''
        possible_files = [file for file in os.listdir(self.folder_path) if file.endswith('.shp')]
        if len(possible_files) != 1:
            raise ValueError(f"There should be exactly one .shp file in the folder, found: {len(possible_files)}")
        shp_file = os.path.join(self.folder_path, possible_files[0])
        buffer_distance = float(self.config.config['distances']['buffer_m'])

        areas = gpd.read_file(shp_file)
        area_coords = []
        buffer_coords = []
        for geom in areas['geometry']:
            if geom and not geom.is_empty:
                if geom.geom_type == 'Polygon':
                    area_coords.append(list(zip(*geom.exterior.xy)))
                    buffered = geom.buffer(buffer_distance)
                    if buffered.is_empty:
                        continue
                    if buffered.geom_type == 'Polygon':
                        buffer_coords.append(list(zip(*buffered.exterior.xy)))
                    elif buffered.geom_type == 'MultiPolygon':
                        for part in buffered.geoms:
                            buffer_coords.append(list(zip(*part.exterior.xy)))

                elif geom.geom_type == 'MultiPolygon':
                    for poly in geom.geoms:
                        area_coords.append(list(zip(*poly.exterior.xy))) 
                        buffered = poly.buffer(buffer_distance)
                        if buffered.is_empty:
                            continue
                        if buffered.geom_type == 'Polygon': buffer_coords.append(list(zip(*buffered.exterior.xy)))
                        elif buffered.geom_type == 'MultiPolygon':
                            for part in buffered.geoms:
                                buffer_coords.append(list(zip(*part.exterior.xy)))
        self.area_coords = area_coords
        self.buffer_coords = buffer_coords
        print('Area and buffer coordinates loaded.')
        return area_coords
    
    def load_standing_locations(self,standing_locations=None)->Coordinates:
        '''
        Opens the first .txt file in the folder and loads the standing locations.\n
        The file should contain coordinates in the format x,y,z or x,y.\n
        If z is not provided, the altitude is calculated using the pointcloudholder.\n
        '''
        if standing_locations is not None:
            self.standing_locations = standing_locations
            return
    
        human_height = float(self.config.config['distances']['human_height_above_ground_m'])

        possible_files = [file for file in os.listdir(self.folder_path) if file.endswith('.txt')]
        if len(possible_files) != 1:
            print('No standing locations file found.')
            print('Random will be generated')
            return
        file = os.path.join(self.folder_path, possible_files[0])

        # file=os.path.join(self.folder_path, 'standing_locations.txt')
        if not os.path.exists(file):
            raise ValueError("Standing locations file not found.")
        
        with open(file, 'r') as f:
            lines = f.readlines()

        for line in lines:
            coords = line.strip().split(',')

            if len(coords) == 3:
                try:
                    x, y, z = float(coords[0]), float(coords[1]),float(coords[2])
                    self.standing_locations.append((x, y, z))

                except ValueError:
                    print(f"Invalid coordinates: {coords}")
            if len(coords) == 2:
                try:
                    x, y = float(coords[0]), float(coords[1])
                    self.standing_locations.append((x, y,self.pointcloudholder.find_altitude((x,y),human_height)))
                except ValueError:
                    print(f"Invalid coordinates: {coords}")


    
    def clean_buffers(self, number_areas:int)->Coordinates:
        '''
        Reduces number of buffers to the number of areas wanted.\n
        '''
        return self.buffer_coords
        def polygon_area(polygon:list[float])->float:
            """
            Calculate the area of a simple polygon using the Shoelace theorem.
            """
            n = len(polygon)
            if n < 3:
                return 0  # Not a valid polygon
            area = 0
            for i in range(n):
                x1, y1 = polygon[i]
                x2, y2 = polygon[(i + 1) % n]  # Next vertex (loop back at the end)
                area += x1 * y2 - x2 * y1

            return abs(area) / 2  # Take absolute value and divide by 2
        
        buffer_coords_areas = []
        for polygon in self.buffer_coords:
            area = polygon_area(polygon)
            buffer_coords_areas.append([polygon,area])
        buffer_coords_areas = sorted(buffer_coords_areas,key=lambda x: x[1],reverse=True) # Order by area
        self.buffer_coords = [x[0] for x in buffer_coords_areas[:number_areas]] 
        print('Buffers cleaned.')
        return buffer_coords_areas[:number_areas]
    

    def inside_shape(self,
                     x:float,
                         y:float,
                         buffer:Coordinates)->bool:
            '''
            Check if a point (x, y) is inside a polygon defined by the buffer coordinates.'''
            
            if not isinstance(x, float) or not isinstance(y, float):
                raise ValueError("x and y must be floats.")
            if not isinstance(buffer, list) or not all(len(coord) == 2 for coord in buffer):
                raise ValueError("Buffer must be a list of tuples with 2 elements each.")
            

            inside = False

            n = len(buffer)

            if n < 3:  # A valid polygon must have at least 3 points
                return False#Check correct
            for i in range(n):
                x1, y1 = buffer[i]
                x2, y2 = buffer[(i + 1) % n]

                if (y1 < y <= y2) or (y2 < y <= y1):  # Check if y is between y1 and y2
                    if x1 + (y - y1) / (y2 - y1) * (x2 - x1) < x:
                        inside = not inside  # Toggle inside status

            if inside:
                return True  # if inside return

            return False  # Not inside any buffer
    def generate_points(self,starting_point:Coordinate=None,points=None)->Coordinates:
        '''
        Generates random points inside the buffer area.\n

        '''
        if points:
            if not isinstance(points, list) or not all(isinstance(point, tuple) and len(point) == 3 for point in points):
                raise ValueError("Points must be a list of tuples with 3 elements each.")
            

        buffer_id = self.current_buffer
        uav_height = float(self.config.config['distances']['height_above_ground_m'])

        def outside_minimum_distance(to_plot:NumpyCoordinates,
                                     planned_plot:np.ndarray[Coordinate])->bool:
            '''
            Calculates distances of all points to the planned plot and checks if they are greater than the minimum distance.\n
            Returns True if all the distances are greater than the minimum distance.\n
            '''
            min_distance = float(self.config.config['distances']['distance_to_nearest_point_m'])
            if len(to_plot) == 0:
                return True
            distance = np.linalg.norm(to_plot - planned_plot,axis=1)
            if distance.min() > min_distance:
                return True
            return False
        if points is not None:
            self.number_points = len(points)
            self.cities = points
            if starting_point is not None:
                self.number_points += 1
                self.cities.insert(0,starting_point)
            return points

        
        x_values = [x[0] for x in self.buffer_coords[buffer_id]]
        y_values = [y[1] for y in self.buffer_coords[buffer_id]]
        min_x, max_x = min(x_values), max(x_values)
        min_y, max_y = min(y_values), max(y_values)
        
        cities = []
        if starting_point is not None:
            cities.append(starting_point)
        attempts=0
        max_attempts= int(self.config.config['distances']['attempts_to_find_point_in_area'])
        while len(cities) < self.number_points:
            if attempts > max_attempts: # If too many attempts, break the loop
                print('Max to generate points inside of area reached.')
                break
            attempts += 1

            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            if not self.inside_shape(x,y,self.buffer_coords[buffer_id]):
                continue
            if not outside_minimum_distance(np.array(cities),np.array([x,y])):
                continue
            if self.standing_locations:
                if not self.pointcloudholder.two_points_visible(self.standing_locations[self.current_standing_id],(x,y,self.pointcloudholder.find_altitude((x,y),uav_height))):
                    continue
            cities.append((x, y))
        self.cities = cities
        return cities
    
    def create_transects(self,ordered_points:Coordinates):
        transect_length = float(self.config.config['distances']['transect_length_m'])
        transects = {}
        for point in ordered_points:
            while True:
                transect = []
                random_angle = np.random.uniform(0,2*np.pi)
                transect.append(point)
                second_point = (point[0] + transect_length*np.cos(random_angle),point[1] + transect_length*np.sin(random_angle))
                transect.append(second_point)
                angle = random_angle + np.pi / 2 if random.randint(0, 1) == 0 else random_angle - np.pi / 2

                third_point = (second_point[0] + transect_length*np.cos(angle + np.pi),second_point[1] + transect_length*np.sin(angle + np.pi))
                transect.append(third_point)
                if self.inside_shape(third_point[0],third_point[1],self.buffer_coords[self.current_buffer]) and self.inside_shape(second_point[0],second_point[1],self.buffer_coords[self.current_buffer]):
                    break
            if point in self.cities and point!=self.cities[0]:
                transects[point]=transect
            else:
                transects[point]=[point]
        self.transects = transects

    def solve_transect_route(self):
        '''
        1. Creates all possible permutations of the transects enter at first point leave at 3rd or the opposite
        2. Finds the shortest path out of these permutations
        3. If plot = True, plots the shortest path
        '''
        possible_perms=[[]]

        for i in self.best_path_coords:
            issingle = len(self.transects[i]) == 1
            if issingle:
 
                new_possible_perms = [x + [[self.transects[i][0],False]] for x in possible_perms]
            else:
                new_possible_perms = [x + [[self.transects[i][0],True],[self.transects[i][1],True],[self.transects[i][2],True]] for x in possible_perms]
                new_possible_perms += [x + [[self.transects[i][2],True],[self.transects[i][1],True],[self.transects[i][0],True]] for x in possible_perms]
            possible_perms = new_possible_perms
        routes = []
        #Compare the length of each possible path

        for path in possible_perms:
            length = self.route_length(path)
            routes.append((path,length))

        sorted_routes = sorted(routes, key=lambda x: x[1], reverse=False)


        shortest_path = sorted_routes[0][0]
        #Adds to the transect path
        self.transect_path=shortest_path
        camera_path = []
        


        



    def route_length(self,route):
        """
        Calculates the total length of a given route.
        :param route: A list of points (x,y) in the order they are visited
        
        """
        total_distance = 0.0
        for i in range(len(route) - 1):
            
            x1, y1 = route[i][0]
            x2, y2 = route[i + 1][0]
            total_distance += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)  # Euclidean distance
        
        return total_distance

    def solve_tsp(self)->None:
        '''
        Solves the TSP problem using the A* algorithm.\n
        '''
        cutoff = int(self.config.config['speed_related']['brute_force_cutoff'])
        a_star_step = float(self.config.config['speed_related']['A*_grid_size_m'])
        if self.standing_locations:
            standing_location = self.standing_locations[self.current_standing_id]
        else:
            standing_location = None

        if len(self.cities) <=cutoff:
            pathfinder = BruteForce(self.cities,self.buffer_coords,self.pointcloudholder,standing_location,a_star_step,self.current_buffer)
        else:
            pathfinder = AntColony(self.cities,self.buffer_coords,self.pointcloudholder,standing_location,a_star_step,self.current_buffer)
        print('Starting TSP solver...')
        path,distance = pathfinder.solve()
        self.best_path_coords = path
        self.best_path_coords_interpolated = self.pointcloudholder.interpolate_route(path)
        print('TSP solved.')


example_cities = [(2023505, 5753392), (2023702, 5753752), (2023289, 5753823)]
if __name__ == "__main__":
    driver = Driver(
        folder_path='Data/MGAT_01201/Base_Data',
    )
    driver.load_shp_file()
    driver.clean_buffers(1)
    driver.load_standing_locations()
    driver.generate_points()    


    driver.solve_tsp()
    driver.pointcloudholder.show_point_cloud(
        cities=driver.cities,
        human_location=driver.standing_locations[driver.current_standing_id],
        dvlos = True,
        best_path_coords=driver.best_path_coords_interpolated)

    