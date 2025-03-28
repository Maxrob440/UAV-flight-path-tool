# Imports
import numpy as np
import open3d as o3d
import rasterio
import geopandas as gpd
from sklearn.neighbors import KDTree
TIF_PATH = 'Data/MGAT_01201/Base_Data/01201_dem.tif'

class PointCloud:
    def __init__(self,tif_path):
        self.point_cloud_holder = o3d.geometry.PointCloud()
        self.xyz = [] # List of points
        self.lines = []
        self.uav_points = o3d.geometry.PointCloud()
        self.KDTree = None
        self.tif_path = tif_path
    
    def read_tif(self):
        '''Reads the tif file and extracts the points, storing in self.xyz. Then adds to the point cloud'''
        with rasterio.open(self.tif_path) as dataset:
            image = dataset.read(1)
            transform = dataset.transform
            for row in range(image.shape[0]):
                for col in range(image.shape[1]):
                    point = image[row, col]
                    if point > -1000 and point < 2000:
                        x, y = transform * (col, row)
                        self.xyz.append([x,y,point])
        self.form_kdtree()
        self.point_cloud_holder.points = o3d.utility.Vector3dVector(self.xyz)

    def form_kdtree(self):
        '''Forms a KDTree from the point cloud'''
        xyz = np.asarray(self.xyz)
        self.KDTree = KDTree(xyz[:,:2])
        # closest_x,closest_y = self.KDTree.query([[1661720.0, 5412341.0]], k=1)
        # height = self.xyz[closest_y[0][0]][2]
        # print(height)
    
    def find_altitude(self, point:list, addition = 0)->float:
        '''
        Finds the altitude of a point in the point cloud or the nearest point otherwise.\n
        Point: [x,y]\n
        Addition: Add a constant to the altitude\n
        Tolerance: The tolerance for the search (MAX <2)\n
        '''
        if len(point) == 3: # Allows the point to be inputted in both x,y and x,y,z
            point = point[:2]
        closest_x,closest_y = self.KDTree.query([point], k=1)
        height = self.xyz[closest_y[0][0]][2]
        return height + addition
        
    
    def distance_to_nearest_point(self,point:list)->float:
        '''Finds the distance to the nearest point in the point cloud
        Point: [x,y,z]
        '''
        point_xy = point[:2]
        closest_x,closest_y = self.KDTree.query([point_xy], k=2)
        nearest_point = self.xyz[closest_y[0][0]]
        
        return np.linalg.norm(np.array(nearest_point)-np.array(point))



    def create_line(self,
                    points:list, 
                    connections:list, 
                    draw_point:bool = True,
                    colour:list = [1,0,0],
                    interpolate:int = None)->None:
        '''
        Creates a line from the  points
        plots: List of points [[x,y,z],[x,y,z]]
        connections: List of connections [[0,1],[1,2]]
        draw_point: True - draws points on the line
        '''
        if draw_point:
            self.create_point(points)
        if interpolate:
            points,connections = self.create_interpolated_line(points,connections,colour,interpolate)
        # else:
        #     line = o3d.geometry.LineSet()
        #     points = np.array(points)
        #     line.points = o3d.utility.Vector3dVector(points)
        #     line.lines = o3d.utility.Vector2iVector(connections)
        #     line.paint_uniform_color(colour)
        #     self.lines = line
        self.create_line_helper(points,connections,colour)


    def create_interpolated_line(self,
                                 points,
                                 connections,
                                 colour=[1,0,0],
                                 interpolate=8):
        ''' Creates interpolation between the points on the line, to ensure distance away from floor is more constant'''
        interpolated_points = []
        for i,_ in enumerate(connections):
            start = points[connections[i][0]]
            end = points[connections[i][1]]
            length = int(int(np.linalg.norm(np.array(start)-np.array(end)))/interpolate)
            x = np.linspace(start[0],end[0],length)
            y = np.linspace(start[1],end[1],length)
            z=[]
            for j in range(length):
                z_value = self.find_altitude([x[j], y[j]],30)  # Get z for each (x[j], y[j])
                z.append(z_value)
            for j in range(length):
                interpolated_points.append([x[j],y[j],z[j]])
        connections = [[i,i+1] for i in range(len(interpolated_points)-1)]
        # connections.append([len(interpolated_points)-1,0])
        return interpolated_points,connections
        

    def create_line_helper(self,points,connections,colour):
        # print(points)
        line = o3d.geometry.LineSet()
        points = np.array(points)
        line.points = o3d.utility.Vector3dVector(points)
        line.lines = o3d.utility.Vector2iVector(connections)
        line.paint_uniform_color(colour)
        self.lines = line



    def create_point(self,points:list,colour:list = None)->None:
        '''
        Creates points on the point cloud
        '''
        if colour is None:
            colour = [0,0,0]
        points_to_plot = []
        for point in points:
            points_to_plot.append(point)
        points = np.array(points_to_plot)
        self.uav_points.points = o3d.utility.Vector3dVector(points)
        self.uav_points.paint_uniform_color(colour)

    def points_visible(self,point,points)->list:
        '''Returns the points visible from the current point'''
        points_visible = []
        visible = True
        for i,_ in enumerate(points):
            length = np.linalg.norm(np.array(point)-np.array(points[i]))
            line = self.create_points_between_two_points(point, points[i], int(length))
            visible = True  # Assume the point is visible
            
            for line_point in line:
                if self.distance_to_nearest_point(line_point) < 1:
                    visible = False  # The point is blocked
                    break  # Stop checking further
            
            if visible:
                points_visible.append(points[i])  # Add only if fully visible
                # print('Can see')
        return points_visible
    
    def create_points_between_two_points(self,point1,point2,no_points)->list:
        '''Creates points between two points'''
        x1,y1,z1 = point1
        x2,y2,z2 = point2
        x = np.linspace(x1,x2,no_points)
        y = np.linspace(y1,y2,no_points)
        z = np.linspace(z1,z2,no_points)
        return [[x[i],y[i],z[i]] for i in range(no_points)]
    
    def display_path(self,best_path_coors):
        connections = [[i,i+1] for i in range(len(best_path_coors)-1)]
        points = [[x,y,z] for x,y in best_path_coors for z in [self.find_altitude([x,y],30)]]
        connections.append([len(best_path_coors)-1,0])
        self.create_line(points,connections,draw_point=True,interpolate=8)

    def show(self):
        '''Plots the point cloud'''
        add_to_point_cloud = []
        add_to_point_cloud.append(self.point_cloud_holder)
        if self.lines:
            add_to_point_cloud.append(self.lines)
        if self.uav_points:
            add_to_point_cloud.append(self.uav_points)



        o3d.visualization.draw_geometries(add_to_point_cloud)
        # o3d.visualization.draw_geometries([self.point_cloud_holder,self.lines,self.uav_points])

if __name__ == "__main__":
    cloud = PointCloud(TIF_PATH)
    cloud.read_tif()
    cloud.form_kdtree()
    # cloud.find_altitude([-13553043.795891073, 4381356.858124066],20)
    cloud.show()
