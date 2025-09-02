
import os
import random
import numpy as np
import geopandas as gpd
import math
from pprint import pprint

from collections import defaultdict

from clustering import DistanceCluster,TspCluster
from Config import Config
from point_cloud import PointCloud
from pathfinding import Pathfinder,AntColony,BruteForce,Christofides,BnB,HeldKarp
import matplotlib.pyplot as plt
from shp_file_generator import TransectGenerator
from transect_solver import BruteTransectSolver,RandomTransectSolver

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
    def __init__(self)->None:
        '''
        Constructor for the Driver class.\n
        Parameters:
        folder_path (str): Path to the folder containing input data.
        number_areas (int): Number of areas to be processed.
        '''
        self.config = Config() # holds all info about the configuration
        self.config.load_config()
        self.pointcloudholder = PointCloud() # all 3D stuff
        self.standing_locations = [] # a list of tuples that holds all the possible standing locations
        self.area_coords = [] # a list of lists of tuples that holds the coordinates of the area polygons
        self.buffer_coords = [] # list of list of tuples that holds buffers
        self.cities = [] # The points which the UAV will visit
        self.best_path_coords = [] # The best found path visiting all cities
        self.best_path_coords_interpolated = []
        self.clustered=[] # The clusters of points
        self.current_cluster = 0
        self.transects = []
        self.transect_path=[]
        self.current_buffer = 0
        self.current_standing_id = 0
        self.transect_distance_cache = {}
        self.grid_distances = None
        self.flyable_area = []
        self.current_flyable_area=0
    
    def load_shp_file(self,path=None)->Coordinates:
        '''
        Loads the shapefile and creates the outline and buffer.
        '''
        if path is not None:
            self.config.config['current_map']['folder_location'] = path
            self.config.save_config()
        
        folder_path = self.config.get_nested('current_map','folder_location')
        try:
            possible_files = [file for file in os.listdir(folder_path) if file.endswith('.shp') and "standing" not in file]
        except FileNotFoundError as e:
            raise FileNotFoundError(f"No such file or directory: '{folder_path}'")
        
        flyable = self.config.get_nested('point_creation','flyable_areas')
                    
        if len(possible_files) > 2:
            raise ValueError(f"There should be either 1/2 .shp file in the folder, found: {len(possible_files)}")
        if len(possible_files) == 1 and flyable:
            raise ValueError("There should be 2 shapefiles in the folder if flyable areas are enabled.")
        
        if len(possible_files) == 2 and not flyable:
            if 'flyable' in possible_files[0]:
                possible_files.pop(0)
            elif 'flyable' in possible_files[1]:
                possible_files.pop(1)
            
            
        if len(possible_files) == 2 and flyable:
            if 'flyable' in possible_files[0]:
                shp_file = os.path.join(folder_path, possible_files[0])
                areas = gpd.read_file(shp_file)
                possible_files.pop(0)
            elif 'flyable' in possible_files[1]:
                shp_file = os.path.join(folder_path, possible_files[1])

                areas = gpd.read_file(shp_file)
                possible_files.pop(1)
            else:
                raise ValueError("No flyable area found in the shapefile names.")
            # self.config.update_nested(['point_creation','flyable_areas'],True)
            for geom in areas['geometry']:
                print(geom.geom_type)
                if geom and not geom.is_empty:
                    if geom.geom_type == 'Polygon':
                        self.flyable_area.append(list(zip(*geom.exterior.xy)))
                    

        output_folder_name = possible_files[0].split('.')[0]
        output_folder_name = output_folder_name.split('_bdy')[0]
        self.config.config['io']['specific_folder_name'] = output_folder_name
        self.config.config['io']['output_transect_file_name'] = f"{output_folder_name}_transects"
        self.config.config['io']['output_CSV_file_name'] = f"{output_folder_name}_path_"
        self.config.save_config()

        shp_file = os.path.join(folder_path, possible_files[0])
        buffer_distance = float(self.config.get_nested('distances','buffer_m'))
        # buffer_distance = float(self.config.config['distances']['buffer_m'])

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
    
    def load_standing_locations(self,standing_locations=None,path = None)->Coordinates:
        '''
        Opens the first .txt file in the folder and loads the standing locations.\n
        The file should contain coordinates in the format x,y,z or x,y.\n
        If z is not provided, the altitude is calculated using the pointcloudholder.\n
        '''
        if standing_locations is not None:
            self.standing_locations = standing_locations
            return
        human_height = float(self.config.get_nested('distances','human_height_above_ground_m'))
        if path is None:
            human_height = float(self.config.get_nested('distances','human_height_above_ground_m'))
            folder_path = self.config.get_nested('current_map','folder_location')
            # human_height = float(self.config.config['distances']['human_height_above_ground_m'])

            possible_files = [file for file in os.listdir(folder_path) if file.endswith('.txt')]
            if len(possible_files) != 1:
                print('No standing locations file found.')
                print('Random will be generated')
                return
            file = os.path.join(folder_path, possible_files[0])
        else:
            file = path
        # file=os.path.join(self.folder_path, 'standing_locations.txt')
        if not os.path.exists(file):
            raise ValueError("Standing locations file not found.")
        
        if file.endswith('.shp'):
            gdf = gpd.read_file(file)
            # human_height = float(self.config.config['distances']['human_height_above_ground_m'])

            for geom in gdf['geometry']:
                if geom and not geom.is_empty:
                    if geom.geom_type == 'Point':
                        x, y = geom.x, geom.y
                        z = self.pointcloudholder.find_altitude((x, y),human_height)
                        self.standing_locations.append((x, y, z))
                    elif geom.geom_type == 'MultiPoint':
                        for point in geom.geoms:
                            x, y = point.x, point.y
                            z = self.pointcloudholder.find_altitude((x, y),human_height)
                            self.standing_locations.append((x, y, z))
        elif file.endswith('.txt'):
            
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
        Check if a point (x, y) is inside a polygon defined by the buffer coordinates.
        '''
        
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
    
    def generate_points_standard(self,starting_point:Coordinate=None,points=None)->Coordinates:
        '''
        Creates a rectangle around the buffer and produces alinear set of points inside the buffer.\n
        '''
        if points:
            if not isinstance(points, list) or not all(isinstance(point, tuple) and len(point) == 3 for point in points):
                raise ValueError("Points must be a list of tuples with 3 elements each.")
            
        buffer_id = self.current_buffer

        uav_height = float(self.config.get_nested('distances','height_above_ground_m'))
        # uav_height = float(self.config.config['distances']['height_above_ground_m'])
        flyable = self.config.get_nested('point_creation','flyable_areas')
        if flyable:
            area = self.flyable_area[self.current_flyable_area]
        else:
            area = self.buffer_coords[buffer_id]
        x_values = [x[0] for x in area]
        y_values = [y[1] for y in area]
        min_x, max_x = min(x_values), max(x_values)
        min_y, max_y = min(y_values), max(y_values)
        
        height_of_rectangle = max_y - min_y
        width_of_rectangle = max_x - min_x

        distance_between_points_in_hexagons = float(self.config.get_nested('hexagonal_grid_generation','grid_size_m'))
        no_rows = int(height_of_rectangle / distance_between_points_in_hexagons)
        no_columns = int(width_of_rectangle / distance_between_points_in_hexagons)
        points = []

        if starting_point is not None:
            points.append(starting_point)
        if not self.standing_locations==[]:
            standing_location = self.standing_locations[self.current_standing_id]
            human_height = float(self.config.get_nested('distances','human_height_above_ground_m'))
            # human_height = float(self.config.config['distances']['human_height_above_ground_m'])
            standing_location_xyz = (standing_location[0],standing_location[1],self.pointcloudholder.find_altitude((standing_location[0],standing_location[1]),human_height))
        for i in range(no_rows):
            for j in range(no_columns):

                x = min_x + j * distance_between_points_in_hexagons
                y = min_y + i * distance_between_points_in_hexagons
                if i %2==1:
                    x=x+(distance_between_points_in_hexagons/2)
                
                if self.inside_shape(x,y,area):
                    if any(self.inside_shape(x,y,area) for area in self.area_coords):
                        if self.standing_locations:
                            if not self.pointcloudholder.two_points_visible(standing_location_xyz,(x,y,self.pointcloudholder.find_altitude((x,y),uav_height))):
                                continue
                        if points is not None:
                            if (x,y) in points:
                                continue
                        if starting_point is not None:
                            if (x,y) == starting_point:
                                continue
                        points.append((x, y))
        self.cities = points
        return points

    def load_points(self,path):
        '''
        Loads points from a given path.\n
        The file should contain coordinates in the format x,y,z or x,y.\n
        If z is not provided, the altitude is calculated using the pointcloudholder.\n
        '''
        self.cities = []
        self.grid_distances = None
        if not os.path.exists(path):
            raise ValueError("Points file not found.")
        
        human_height = float(self.config.get_nested('distances','human_height_above_ground_m'))
        # human_height = float(self.config.config['distances']['human_height_above_ground_m'])

        if path.endswith('.shp'):
            gdf = gpd.read_file(path)
            for geom in gdf['geometry']:
                if geom and not geom.is_empty:
                    if geom.geom_type == 'Point':
                        x, y = geom.x, geom.y
                        self.cities.append((x,y))
                    elif geom.geom_type == 'MultiPoint':
                        for point in geom.geoms:
                            x, y = point.x, point.y
                            self.cities.append((x, y))
        elif path.endswith('.txt'):
            
            with open(path, 'r') as f:
                lines = f.readlines()

            for line in lines:
                coords = line.strip().split(',')

                if len(coords) == 3:
                    try:
                        x, y, z = float(coords[0]), float(coords[1]),float(coords[2])
                        self.cities.append((x, y))

                    except ValueError:
                        print(f"Invalid coordinates: {coords}")
                if len(coords) == 2:
                    try:
                        x, y = float(coords[0]), float(coords[1])
                        self.cities.append((x, y))
                    except ValueError:
                        print(f"Invalid coordinates: {coords}")
        print('Points loaded.')
        self.cities.append(self.standing_locations[self.current_standing_id][:2])

    def generate_points_random(self,starting_point:Coordinate=None,points=None)->Coordinates:
        '''
        Generates random points inside the buffer area.\n

        '''
        if points:
            if not isinstance(points, list) or not all(isinstance(point, tuple) and len(point) == 3 for point in points):
                raise ValueError("Points must be a list of tuples with 3 elements each.")
            

        buffer_id = self.current_buffer
        uav_height = float(self.config.get_nested('distances','height_above_ground_m'))
        # uav_height = float(self.config.config['distances']['height_above_ground_m'])

        def outside_minimum_distance(to_plot:NumpyCoordinates,
                                     planned_plot:np.ndarray[Coordinate])->bool:
            '''
            Calculates distances of all points to the planned plot and checks if they are greater than the minimum distance.\n
            Returns True if all the distances are greater than the minimum distance.\n
            '''
            min_distance = float(self.config.get_nested('random_point_generation','distance_to_nearest_point_m'))
            # min_distance = float(self.config.config['distances']['distance_to_nearest_point_m'])
            if len(to_plot) == 0:
                return True
            distance = np.linalg.norm(to_plot - planned_plot,axis=1)
            if distance.min() > min_distance:
                return True
            return False
        if points is not None:
            self.cities = points
            if starting_point is not None:
                self.cities.insert(0,starting_point)
            return points

        flyable = self.config.get_nested('point_creation','flyable_areas')
        if flyable:
            area = self.flyable_area[self.current_flyable_area]
        else:
            area = self.buffer_coords[buffer_id]
        x_values = [x[0] for x in area]
        y_values = [y[1] for y in area]

        min_x, max_x = min(x_values), max(x_values)
        min_y, max_y = min(y_values), max(y_values)
        
        cities = []
        if starting_point is not None:
            cities.append(starting_point)
        attempts=0
        max_attempts = int(self.config.get_nested('random_point_generation','attempts_to_find_point_in_area'))
        # max_attempts= int(self.config.config['distances']['attempts_to_find_point_in_area'])
        while len(cities) < int(self.config.get_nested('random_point_generation','number_of_points_per_area')):
            if attempts > max_attempts: # If too many attempts, break the loop
                print('Max to generate points inside of area reached.')
                break
            attempts += 1

            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            if not any(self.inside_shape(x,y,buffer) for buffer in self.buffer_coords):
                continue
            if not outside_minimum_distance(np.array(cities),np.array([x,y])):
                continue
            if self.standing_locations:
                if not self.pointcloudholder.two_points_visible(self.standing_locations[self.current_standing_id],(x,y,self.pointcloudholder.find_altitude((x,y),uav_height))):
                    continue
            cities.append((x, y))
        self.cities = cities
        return cities
    

    
    def create_transects(self):
        transect_length = float(self.config.get_nested('distances','transect_length_m'))
        # transect_length = float(self.config.config['distances']['transect_length_m'])
        current_cluster = self.current_cluster
        ordered_points = self.clustered[current_cluster]
        
        transects = [{} for _ in range(len(self.clustered))]
        flyable = self.config.get_nested('point_creation','flyable_areas')
        if flyable:
            boundary = self.flyable_area[self.current_flyable_area]
        else:
            if self.buffer_coords != []:
                boundary = self.buffer_coords[self.current_buffer]
            else:
                boundary = []
        # transects = {}
        for point in ordered_points:
            if point == ordered_points[0]:
                transects[current_cluster][point] = [point]
                continue
            if self.standing_locations:
                if point[:2] == self.standing_locations[self.current_standing_id][:2]:
                    transects[current_cluster][point] = [point]
                    continue
            while True:
                transect = []
                random_angle = np.random.uniform(0,2*np.pi)
                inverse_angle = (random_angle + np.pi)% (2 * np.pi)  # Inverse angle for the transect
                leading_length = 5
                leading_point = (point[0] + leading_length*np.cos(inverse_angle),point[1] + leading_length*np.sin(inverse_angle))
                transect.append(leading_point)
                transect.append(point)
                second_point = (point[0] + transect_length*np.cos(random_angle),point[1] + transect_length*np.sin(random_angle))
                mid_point = ((point[0] + second_point[0]) / 2, (point[1] + second_point[1]) / 2)
                transect.append(mid_point)
                transect.append(second_point)
                angle = random_angle + np.pi / 2 if random.randint(0, 1) == 0 else random_angle - np.pi / 2
                third_point = (second_point[0] + transect_length*np.cos(angle + np.pi),second_point[1] + transect_length*np.sin(angle + np.pi))
                mid_point2= ((second_point[0] + third_point[0]) / 2, (second_point[1] + third_point[1]) / 2)
                transect.append(mid_point2)
                transect.append(third_point)
                ex_length = leading_length + transect_length
                leading_point2 = (second_point[0] + ex_length*np.cos(angle + np.pi),second_point[1] + ex_length*np.sin(angle + np.pi))
                # leading_point2 = (third_point[0] + leading_length*np.cos(angle),third_point[1] + leading_length*np.sin(angle))
                transect.append(leading_point2)

                if self.buffer_coords ==[]:
                    break
            
                if self.inside_shape(third_point[0],third_point[1],boundary) and self.inside_shape(second_point[0],second_point[1],boundary):
                    break
            
            if point in self.cities: #and point!=self.cities[0] removed, unsure why was here
                transects[current_cluster][point]=transect

                # transect_generator.add_transect(transect)
            else:
                transects[current_cluster][point]=[point]
        self.transects = transects
        # transect_generator.save()

    def solve_transect_route(self):
        '''
        1. Creates all possible permutations of the transects enter at first point leave at 3rd or the opposite
        2. Finds the shortest path out of these permutations
        3. If plot = True, plots the shortest path
        '''
        standing_location = None
        if self.standing_locations:
            standing_location = self.standing_locations[self.current_standing_id]

        if len(self.transects[self.current_cluster]) < 20:
            transect_solver = BruteTransectSolver(self.transects[self.current_cluster],
                                                  self.best_path_coords,
                                                  standing_location)

        else:
            print('Too many transects to brute force')
            transect_solver = RandomTransectSolver(self.transects[self.current_cluster],
                                                  self.best_path_coords,
                                                  standing_location)
        shortest_path = transect_solver.solve()

        #Adds to the transect path
        if self.standing_locations:
            if shortest_path[-1][0]==self.standing_locations[self.current_standing_id][:2]:
                shortest_path.pop()
           
        self.transect_path=shortest_path
        


        



    def route_length(self,route):
        """
        Calculates the total length of a given route.
        :param route: A list of points (x,y) in the order they are visited
        
        """
        total_distance = 0.0
        for i in range(len(route) - 1):
            total_distance += math.dist(route[i][0], route[i + 1][0])

        return total_distance
    

        

    def solve_tsp(self)->None:
        '''
        Solves the TSP problem using the A* algorithm.\n
        '''
        cutoff = int(self.config.get_nested('speed_related','brute_force_cutoff'))
        a_star_step = float(self.config.get_nested('speed_related','A*_grid_size_m'))
        flyable = self.config.get_nested('point_creation','flyable_areas')

        if flyable:
            boundary = self.flyable_area
            boundary_no = 0
        else:
            boundary = self.buffer_coords
            boundary_no = self.current_buffer

        if self.standing_locations:
            standing_location = self.standing_locations[self.current_standing_id]
        else:
            standing_location = None

        approx_pathfinder = None
        distances = None
        if self.grid_distances is not None:
            distances = self.grid_distances
        if len(self.cities) <=cutoff:
            pathfinder = HeldKarp(self.cities,boundary,self.pointcloudholder,standing_location,a_star_step,boundary_no,distances)
        else:
            pathfinder = AntColony(self.cities,boundary,self.pointcloudholder,standing_location,a_star_step,boundary_no,distances =distances)
            distances = pathfinder.distances
            self.grid_distances = pathfinder.distances
            approx_pathfinder = Christofides(self.cities,boundary,self.pointcloudholder,standing_location,a_star_step,boundary_no,distances)
        print('Starting TSP solver...')
        path,distance = pathfinder.solve()
        if approx_pathfinder:
            approx_path,approx_distance = approx_pathfinder.solve()
            if approx_distance< distance:
                path = approx_path
                print('Christofides is shorter than antcolony, using Christofides')

        self.best_path_coords = path
        self.best_path_coords_interpolated = self.pointcloudholder.interpolate_route(path)
        print('TSP solved.')

    def cycle_cluster(self):
        self.current_cluster=(self.current_cluster +1)%len(self.clustered)
    
    def cycle_buffer(self):
        flyable = self.config.get_nested('point_creation','flyable_areas')
        print(len(self.flyable_area))
        if flyable:
            self.current_flyable_area=(self.current_flyable_area +1)%len(self.flyable_area)
        else:
            self.current_buffer=(self.current_buffer +1)%len(self.buffer_coords)

    def cluster_points(self):
        ''' Decides which method to follow, TSP based if the TSP is solved first, other if not '''
        a_star_step = float(self.config.get_nested('speed_related','A*_grid_size_m'))
        human_location = None
        if self.standing_locations!=[]:
            human_location = self.standing_locations[self.current_standing_id]

        if self.best_path_coords:
            print('Clustering points based on TSP path')
            clusterer = TspCluster(cities=self.cities,
                                   best_path_coords=self.best_path_coords,
                                   human_location=human_location)
            self.clustered = clusterer.cluster()
            if not human_location:
                return self.clustered
            # raise NotImplementedError("TSP path is available but no human location is set.")
        else:
            print('no TSP')
            if self.standing_locations:
                clusterer = DistanceCluster(cities=self.cities,
                                            human_location=human_location)
                self.clustered = clusterer.cluster()
                for ind,cluster in enumerate(self.clustered):
                    pathfinder =HeldKarp(cluster,self.buffer_coords,self.pointcloudholder, human_location,a_star_step,self.current_buffer)
                    path_distance = pathfinder.solve()
                    path, distance = path_distance
                    self.clustered[ind] = path
                
                for cluster in self.clustered:
                    for point in cluster:
                        self.best_path_coords.append(point)

        if self.standing_locations!=[]:
            human_location = self.standing_locations[self.current_standing_id]
            for cluster in self.clustered:
                if cluster[0]!=human_location and cluster[0]!=human_location:
                    
                            # cluster.insert(0,human_location)
                    # if cluster[-1]!=human_location:
                    #     cluster.append(human_location)
                    a_star_step = float(self.config.get_nested('speed_related','A*_grid_size_m'))
                    pathfinder = Pathfinder(self.cities,self.buffer_coords,self.pointcloudholder,human_location,a_star_step,self.current_buffer,self.grid_distances)
                    self.grid_distances = pathfinder.distances
                    path_distance = pathfinder.a_star(human_location,cluster[0])
                    path,distance = path_distance
                    if distance == np.inf:
                        raise ValueError("No path found between human location and first point in cluster.")
                    path = path[:-1]  # Remove the last point as it is the same as the first point in the cluster
                    for point in reversed(path):
                        cluster.insert(0,point)  # Insert the path in reverse order to maintain the correct order

    # def tsp_cluster(self):
    #     human_location = None
    #     if self.standing_locations:
    #         human_location = self.standing_locations[self.current_standing_id]

    #     tsp_path = self.best_path_coords
    #     points_per_cluster = int(self.config.get_nested('clustering','points_per_cluster'))
    #     clustered_tsp = []
    #     current_cluster = []
    #     # while tsp_path:
    #     for ind,point in enumerate(tsp_path):
    #         if point not in self.cities and len(current_cluster) ==0: # Stops a* path from previous cluster being added to the next cluster
    #             continue
    #         current_cluster.append(point)
    #         count_real_points = sum(1 for p in current_cluster if p in self.cities)
    #         if count_real_points >= points_per_cluster:
    #             clustered_tsp.append(current_cluster)
    #             current_cluster = []
    #     if current_cluster:
    #         clustered_tsp.append(current_cluster)
                    
                    
    #         # cluster = tsp_path[:points_per_cluster]
    #         # tsp_path = tsp_path[points_per_cluster:]
    #         # clustered_tsp.append(cluster)
        
    #     if human_location:
    #         for cluster in clustered_tsp:
    #             if cluster[0]!=human_location and cluster[0]!=human_location[:2]:
                    
    #                         # cluster.insert(0,human_location)
    #                 # if cluster[-1]!=human_location:
    #                 #     cluster.append(human_location)
    #                 a_star_step = float(self.config.get_nested('speed_related','A*_grid_size_m'))
    #                 pathfinder = Pathfinder(self.cities,self.buffer_coords,self.pointcloudholder,human_location,a_star_step,self.current_buffer)
    #                 path_distance = pathfinder.a_star(human_location,cluster[0])
    #                 path,distance = path_distance
    #                 if distance == np.inf:
    #                     raise ValueError("No path found between human location and first point in cluster.")
    #                 path = path[:-1]  # Remove the last point as it is the same as the first point in the cluster
    #                 for point in reversed(path):
    #                     cluster.insert(0,point)  # Insert the path in reverse order to maintain the correct order
        
        
    #     self.clustered = clustered_tsp
        

    

if __name__ == "__main__":
    driver = Driver()
    driver.cities = [(0,0,0), (1,1,1), (2,2,2)]
    driver.standing_locations=[(0.5,0.5,0)]
    driver.pointcloudholder.read_tif(driver.cities)
    driver.solve_tsp()
    print(driver.best_path_coords)