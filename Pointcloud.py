import os
import numpy as np
import open3d as o3d
import rasterio
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
from Config import Config

XYCoordinate = tuple[float, float]
XYCoordinates = list[XYCoordinate]
XYZCoordinate = tuple[float, float, float]
XYZCoordinates = list[XYZCoordinate]

class PointCloud:
    '''
    PointCloud class to manage the point cloud data and perform operations on it.\n
    '''
    def __init__(self, tif_path:str):
        self.config = Config()
        self.point_cloud_holder = o3d.geometry.PointCloud()
        self.uav_pointcloud = o3d.geometry.PointCloud()
        self.xyz = []
        self.tif_path = tif_path
        self.kdtree = None
        self.dvlos_lines = []
    
    def read_tif(self,xyz = None)-> KDTree:
        '''
        Reads the tif file saving data into a kdtree\n
        '''

        if xyz is None:
            possible_files = [file for file in os.listdir(self.tif_path) if file.endswith('.tif')]
            if len(possible_files) != 1:
                raise ValueError("There should be exactly one .tif file in the folder.")
            tif_filepath = os.path.join(self.tif_path, possible_files[0])
            xyz = [] # List of points
            with rasterio.open(tif_filepath) as dataset:
                image = dataset.read(1)
                transform = dataset.transform
                for row in range(image.shape[0]):
                    for col in range(image.shape[1]):
                        point = image[row, col]
                        if point > -1000 and point < 2000:
                            x, y = transform * (col, row)
                            xyz.append([x,y,point])
        self.xyz = np.array(xyz)
        kdtree = KDTree(self.xyz[:,:2])
        self.kdtree = kdtree
        return kdtree
    
    def nearest_point(self,
                      point:XYCoordinate|XYZCoordinate,
                      )-> XYZCoordinate:
        '''
        Finds the nearest point in the point cloud or the nearest point otherwise.\n
        '''
        if (len(point) == 3):
            point = point[:2]
        _,closest_y = self.kdtree.query([point], k=1)
        nearest_point_coords = self.xyz[closest_y][0][0]
        return nearest_point_coords
    
    def distance_to_nearest_point(self,
                                  point:XYCoordinate)-> float:
        '''
        Finds and returns the distance to the nearest point.\n
        Only uses the x and y coordinates.\n
        '''
        nearest_point = self.nearest_point(point)
        return float(np.linalg.norm(np.array(point)-np.array(nearest_point)))
    
    def find_altitude(self,
                      point:XYCoordinate,
                      z_addition = 0)-> float:
        '''
        Finds the altitude of a point in the point cloud or the nearest point otherwise, then adds the z_addition to the height.\n
        '''
        if len(point) != 2:
            raise ValueError("The point should be a tuple of two coordinates (x, y).")
        if not isinstance(point, tuple):
            raise TypeError(f"The point should be a tuple of two coordinates (x, y), got {type(point)} : {point}.") 
        if not isinstance(z_addition,(int,float)):
            raise TypeError(f"z_addition should be a number, got {type(z_addition)} : {z_addition}.")
        _,_,height = self.nearest_point(point)
        return height + z_addition
    
    def two_points_visible(self,
                            point1:XYCoordinate|XYZCoordinate,
                            point2:XYCoordinate|XYZCoordinate,
                            plot = False)-> bool:
        '''
        Checks if two points are visible to each other\n
        '''
        minimum_dvlos = float(self.config.config['speed_related']['DVLOS_m'])
        uav_height = float(self.config.config['distances']['height_above_ground_m'])

        if len(point1)==2:
            point1 = (point1[0],point1[1], self.find_altitude(point1,uav_height))
        if len(point2)==2:
            point2 = (point2[0],point2[1], self.find_altitude(point2,uav_height))
            
        interpolation_distance = float(self.config.config['distances']['interpolation_distance_m'])

        length = int(np.linalg.norm(np.array(point1)-np.array(point2))/interpolation_distance)
        x= np.linspace(point1[0],point2[0],num=length)
        y= np.linspace(point1[1],point2[1],num=length)
        z= np.linspace(point1[2],point2[2],num=length)
        line_between_two_points = np.array([x,y,z]).T
        visible = True
        for i,_ in enumerate(line_between_two_points):
            if i %5 ==0:
                if plot:
                    self.add_line_to_pointcloud(point1,line_between_two_points[i])
            if self.distance_to_nearest_point(line_between_two_points[i]) < minimum_dvlos:
                visible = False
                break
        return visible
    
    def interpolate_route(self,
                          route)-> XYZCoordinates:
        bool_route = None
        interpolated_route = []
        height = float(self.config.config['distances']['height_above_ground_m'])
        if isinstance(route[0][1],bool):
            bool_route = route
            route = [point[0] for point in route]


        for i in range(len(route)-1):
                length = int(np.linalg.norm(np.array(route[i])-np.array(route[i+1]))/float(self.config.config['distances']['interpolation_distance_m']))
                interpolated_x = np.linspace(route[i][0],route[i+1][0],num=length)
                interpolated_y = np.linspace(route[i][1],route[i+1][1],num=length)
                interpolated_z = [self.find_altitude(point,height) for point in zip(interpolated_x,interpolated_y)]
                for j,_ in enumerate(interpolated_x):
                    interpolated_route.append((interpolated_x[j],interpolated_y[j],interpolated_z[j]))
        if bool_route:
            new_interpolated_route = []
            bool_route_coords = [point[0] for point in bool_route]

            for i in range(len(interpolated_route)):
                if interpolated_route[i][:2] in bool_route_coords:
                    # index_in_bool_route = bool_route.index(interpolated_route[i][:2])
                    new_interpolated_route.append((interpolated_route[i],True))
                else:
                    new_interpolated_route.append((interpolated_route[i],False))

            return new_interpolated_route
        return interpolated_route
        
        
    
    def show_two_d_graph(self,
                         cities:XYCoordinates = None,
                         human_location:XYCoordinate = None,
                         buffer:XYCoordinates = None,
                         area_coords:XYCoordinates = None)-> None:
        '''
        Shows a 2D matplot lib representation of the cities, the buffer and the area\n
        '''
        if not cities: cities = []
        if not human_location: human_location = []
        if not buffer: buffer = []
        if not area_coords: area_coords = []

        plt.scatter(human_location[0],human_location[1],marker='o',color='black')

        plt.scatter([x[0] for x in cities],[y[1] for y in cities],marker='o',color='red')

        for i in range(len(buffer)):
            plt.plot([x[0] for x in buffer[i]],[y[1] for y in buffer[i]],color='blue')

        for i in range(len(area_coords)):
            plt.plot([x[0] for x in area_coords[i]], [y[1] for y in area_coords[i]],color='green')


        plt.show()

    def show_point_cloud(self,
                         cities:XYCoordinates=None,
                         human_location:XYZCoordinate = None,
                         dvlos:bool=True,
                         best_path_coords:XYCoordinates = None)-> None:
        '''
        Shows the point cloud\n
        '''
        #Avoids putting lists into the pointcloud
        if not cities: cities = []
        if not human_location: human_location = []
        if not best_path_coords: best_path_coords = []

        uav_height = float(self.config.config['distances']['height_above_ground_m'])
        add_to_point_cloud = []
        self.point_cloud_holder.points = o3d.utility.Vector3dVector(self.xyz)
        # self.point_cloud_holder.paint_uniform_color([0,1,0])

        add_to_point_cloud.append(self.point_cloud_holder)
        if cities:
            cities = [[x,y,self.find_altitude((x,y),uav_height)] for x,y in cities]
            self.uav_pointcloud.points = o3d.utility.Vector3dVector(cities)
            self.uav_pointcloud.paint_uniform_color([0,0,0])
            add_to_point_cloud.append(self.uav_pointcloud)
        if human_location:
            human_location_pointcloud = o3d.geometry.PointCloud()
            human_location_pointcloud.points = o3d.utility.Vector3dVector([human_location])
            human_location_pointcloud.paint_uniform_color([0,0,1])
            add_to_point_cloud.append(human_location_pointcloud)
        if best_path_coords:
            interpolated_route = self.interpolate_route(best_path_coords)
            for i in range(len(interpolated_route)-1):
                self.add_line_to_pointcloud(interpolated_route[i][0],interpolated_route[i+1][0],[1,0,0])
        if dvlos and cities and human_location:

            for city in cities:
                self.add_line_to_pointcloud(human_location,city)
        if self.dvlos_lines:            
            for line in self.dvlos_lines:
                add_to_point_cloud.append(line)


        o3d.visualization.draw_geometries(add_to_point_cloud)

    def add_line_to_pointcloud(self,
                                  point1:XYCoordinate,
                                  point2:XYCoordinate,
                                  colour=None)-> None:
        '''
        Adds a line to the point cloud\n
        '''
        uav_height= float(self.config.config['distances']['height_above_ground_m'])

        if colour is None:
            colour = [1,1,1]
        if len(point1)==2:
            point1 = (point1[0],point1[1], self.find_altitude(point1,uav_height))
        if len(point2)==2:
            point2 = (point2[0],point2[1], self.find_altitude(point2,uav_height))

        line = np.array([point1,point2])
        dvlos_lines = o3d.geometry.LineSet()
        dvlos_lines.points = o3d.utility.Vector3dVector(line)
        dvlos_lines.lines = o3d.utility.Vector2iVector([[0,1]])
        dvlos_lines.paint_uniform_color(colour)
        self.dvlos_lines.append(dvlos_lines)


if __name__ == "__main__":
    point_cloud = PointCloud('Data/MGAT_01201/Base_Data/')
    point_cloud.show_point_cloud()
    