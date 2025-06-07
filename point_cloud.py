'''
Management of all 3D logic
'''
import os
import numpy as np
import open3d as o3d
import rasterio

import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for matplotlib

from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
from Config import Config
from voxel_grid import VoxelGrid

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
        self.grid = VoxelGrid((0,0,0),float(self.config.config['distances']['voxel_size_m']))

    def read_tif(self, xyz=None) -> KDTree:
        '''
        Reads the .tif file and builds a KDTree and Voxelises the points from valid elevation points.
        '''
        if xyz is None:
            possible_files = [file for file in os.listdir(self.tif_path) if file.endswith('.tif')]
            if len(possible_files) != 1:
                raise ValueError("There should be exactly one .tif file in the folder.")

            tif_filepath = os.path.join(self.tif_path, possible_files[0])

            with rasterio.open(tif_filepath) as dataset:
                image = dataset.read(1)
                transform = dataset.transform
                nodata = dataset.nodata

                rows, cols = np.indices(image.shape)

                rows_flat = rows.ravel()
                cols_flat = cols.ravel()
                elevation_flat = image.ravel()

                # mask for valid elevation values
                valid_mask = (elevation_flat > -1000) & (elevation_flat < 2000)
                if nodata is not None:
                    valid_mask &= (elevation_flat != nodata)

                # removes large and small values
                rows_valid = rows_flat[valid_mask]
                cols_valid = cols_flat[valid_mask]
                elevation_valid = elevation_flat[valid_mask]

                # affine transform to get X, Y coordinates
                xs, ys = rasterio.transform.xy(transform, rows_valid, cols_valid)
                xs = np.array(xs)
                ys = np.array(ys)

                self.xyz = np.column_stack((xs, ys, elevation_valid))

        else:
            self.xyz = np.array(xyz)
        for point in self.xyz:
            self.grid.insert(point)
        self.kdtree = KDTree(self.xyz[:, :2])
        return self.kdtree

    def nearest_point(self,
                      point:XYCoordinate|XYZCoordinate,
                      )-> XYZCoordinate:
        """Finds the nearest point in the pointcloud, if in form of XYCoordinate,
        it will use the kdtree, else uses the voxel grid.

        Args:
            point (XYCoordinate | XYZCoordinate): A tuple of (x, y) or (x, y, z).

        Returns:
            XYZCoordinate: The nearest point in the point cloud as a tuple (x, y, z).
        """        
        if len(point) == 3:
            nearest_point_coords = self.nearest_point_in_voxel(point)
            return nearest_point_coords
        _,closest_y = self.kdtree.query([point], k=1)
        nearest_point_coords = self.xyz[closest_y][0][0]
        return nearest_point_coords

    def nearest_point_in_voxel(self,
                               point:XYZCoordinate)-> XYZCoordinate:
        """A helper method for nearest_point, which finds the nearest point in voxel grid,
        has the ability to query adjacent voxels within DVLOS distance.\n

        Args:
            point (XYZCoordinate): A tuple of (x, y, z) coordinates.

        Raises:
            ValueError: No points found in the voxel or surrounding voxels for the given point, ensure voxel grid has been populated.

        Returns:
            XYZCoordinate: A tuple (x, y, z) of the nearest point in the voxel grid.
        """        
        points_in_voxel = self.grid.safe_query(point)
        nearest_point = ((0,0,0), float('inf')) # set default with infinte distance
        for possible_point in points_in_voxel:
            distance = np.linalg.norm(np.array(point)-np.array(possible_point))
            if distance < nearest_point[1]:
                nearest_point = (possible_point, distance)
        if nearest_point[1] == float('inf'):
            raise ValueError("No points found in the voxel for the given point.")

        return nearest_point[0]

    def distance_to_nearest_point(self,
                                  point:XYCoordinate|XYZCoordinate)-> float:
        """Provides the distance to the nearest point in the point cloud.
        If the point is in the form of XYCoordinate, it will use the kdtree,
        otherwise it will use the voxel grid to find the nearest point.\n

        Args:
            point (XYCoordinate | XYZCoordinate): A tuple of (x, y) or (x, y, z) coordinates.

        Raises:
            ValueError: If coordinates are not in the form of XYCoordinate or XYZCoordinate.

        Returns:
            float: Distance to the nearest point in the point cloud.
        """        
        if len(point) == 3:
            return self.distance_to_closest_point_in_voxel(point)
        elif len(point) == 2:
            nearest_point = self.nearest_point(point)
            z_value = self.find_altitude(point, 0)
            point = (point[0], point[1], z_value)
            return float(np.linalg.norm(np.array(point)-np.array(nearest_point)))
        else:
            raise ValueError("The point should be a tuple of two or three coordinates (x, y) or (x, y, z).")
        
    def distance_to_closest_point_in_voxel(self, point: XYZCoordinate) -> float:
        """
        Helper method for distance_to_nearest_point, which finds the distance to the closest point in the voxel grid.

        Args:
            point (XYCoordinate): A tuple of (x, y,z) coordinates.

        Returns:
            float: The shortest distance to the nearest point in the voxel grid.
        """        
        possible_points = self.grid.safe_query(point)

        if not possible_points:
            return float('inf')

        shortest_distance = float('inf')
        possible_points = np.array(possible_points)
        point = np.array(point)


        distances = np.linalg.norm(possible_points-point,axis = 1)
        shortest_distance = np.min(distances)


        return shortest_distance


    def find_altitude(self,
                      point:XYCoordinate,
                      z_addition:float = 0.0)-> float:
        """Finds the altitude of the nearest point in the point cloud,

        Args:
            point (XYCoordinate): Tuple of (x, y) coordinates.
            z_addition (float, optional): A altitude addition made to the point. Defaults to 0.

        Raises:
            ValueError: if point is not in form of XYCoordinate
            TypeError: if point is not a tuple
            TypeError: if z_addition is not a number

        Returns:
            float: The altitude of the nearest point in the point cloud plus the z_addition.
        """        
        if len(point) != 2:
            raise ValueError(f"The point should be a tuple of two coordinates (x, y). got {type(point)},{point}.")
        if not isinstance(point, tuple):
            raise TypeError(f"The point should be (x, y), got {type(point)} : {point}.")
        if not isinstance(z_addition,(int,float)):
            raise TypeError(f"z should be a number, got {type(z_addition)} : {z_addition}.")
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

        interpolation_distance = float(self.config.config['speed_related']['DVLOS_interpolation_m'])

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
            if self.distance_to_closest_point_in_voxel(line_between_two_points[i]) < minimum_dvlos:
                visible = False
                break
            # if self.distance_to_nearest_point(line_between_two_points[i]) < minimum_dvlos:
                # visible = False
                # break
        return visible

    def interpolate_route(self,
                          route)-> XYZCoordinates:
        '''
        produces an interpolated route from the given route.\n
        '''
        bool_route = None
        interpolated_route = []
        height = float(self.config.config['distances']['height_above_ground_m'])
        if isinstance(route[0][1],bool):
            bool_route = route
            route = [point[0] for point in route]

        interpolation_distance = float(self.config.config['distances']['interpolation_distance_m'])
        for i in range(len(route)-1):
            p1 = np.array(route[i])
            p2 = np.array(route[i+1])
            length = int(np.linalg.norm(p1-p2)/interpolation_distance)
            if length < 2:
                if len(route[i]) == 2:
                    p1 =(p1[0],p1[1],self.find_altitude((p1[0],p1[1]),height))
                    p2 =(p2[0],p2[1],self.find_altitude((p2[0],p2[1]),height))
                p1=tuple(p1)
                p2=tuple(p2)
                if p1 not in interpolated_route:
                    interpolated_route.append((p1[0],p1[1],p1[2]))
                if p2 not in interpolated_route:
                    interpolated_route.append((p2[0],p2[1],p2[2]))
                continue
            interpolated_x = np.linspace(route[i][0],route[i+1][0],num=length)
            interpolated_y = np.linspace(route[i][1],route[i+1][1],num=length)
            interpolated_z = [self.find_altitude(point,height)
                              for point in zip(interpolated_x,interpolated_y)]
            for j,_ in enumerate(interpolated_x):
                if tuple((interpolated_x[j],interpolated_y[j],interpolated_z[j])) not in interpolated_route:
                    interpolated_route.append((interpolated_x[j],interpolated_y[j],interpolated_z[j]))

                
        if bool_route:
            new_interpolated_route = []
            bool_route_coords = [point[0] for point in bool_route]

            for i,_ in enumerate(interpolated_route):
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
                         area_coords:XYCoordinates = None,
                         transects:XYCoordinates=None,
                         transect_path:XYZCoordinates=None)-> None:
        '''
        Shows a 2D matplot lib representation of the cities, the buffer and the area\n
        '''
        if transect_path is None:
            transect_path = []
        if transects is None:
            transects = []
        if not cities:
            cities = []
        if not human_location:
            human_location = []
        if not buffer:
            buffer = []
        if not area_coords:
            area_coords = []
            
        if transect_path:
            plt.plot([x[0][0] for x in transect_path],[y[0][1] for y in transect_path],color='black',marker='o',alpha=0.5)
        if human_location:
            plt.scatter(human_location[0],human_location[1],marker='o',color='black')
        if cities:
            plt.scatter([x[0] for x in cities],[y[1] for y in cities],marker='o',color='red')
        if transects:
            for transect in transects.values():
                plt.plot([x[0] for x in transect],[y[1] for y in transect],color='orange',marker='o')
        for i in range(len(buffer)):
            plt.plot([x[0] for x in buffer[i]],[y[1] for y in buffer[i]],color='blue')

        for i in range(len(area_coords)):
            plt.plot([x[0] for x in area_coords[i]], [y[1] for y in area_coords[i]],color='green')

        plt.axis('equal')
        plt.show()

    def show_point_cloud(self,
                         cities:XYCoordinates=None,
                         human_location:XYZCoordinate = None,
                         dvlos:bool=True,
                         best_path_coords:XYCoordinates = None,
                         buffer_coords:XYCoordinates=None)-> None:
        '''
        Shows the point cloud\n
        '''
        #Avoids putting lists into the pointcloud
        if not cities:
            cities = []
        if not human_location:
            human_location = []
        if not best_path_coords:
            best_path_coords = []
        if not buffer_coords:
            buffer_coords = []

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
                self.add_line_to_pointcloud(interpolated_route[i][0],
                                            interpolated_route[i+1][0],[1,0,0])
        if dvlos and cities and human_location:

            for city in cities:
                self.add_line_to_pointcloud(human_location,city)
        if buffer_coords:
            for point in range(len(buffer_coords)-1):
                self.add_line_to_pointcloud(buffer_coords[point],buffer_coords[point+1],[0,1,0])


        if self.dvlos_lines:
            for line in self.dvlos_lines:
                add_to_point_cloud.append(line)


        o3d.visualization.draw_geometries(add_to_point_cloud) # pylint:disable=no-member

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
    point_cloud = PointCloud('example data/hodhill')
    point_cloud.read_tif()
    point_cloud.show_point_cloud()
