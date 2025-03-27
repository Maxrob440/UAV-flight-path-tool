import pointcloud as pcl
from cities import Shp_Opener
from ants import AntColony
import os

possible_files = [dir for dir in os.listdir('Data/Data') if os.path.isdir(f'Data/Data/{dir}')]
print(possible_files)
chosen_file = possible_files[3]
tif_file = [dir for dir in os.listdir(f'Data/Data/{chosen_file}/Base_Data') if dir.endswith('.tif')]
shp_file = [dir for dir in os.listdir(f'Data/Data/{chosen_file}/Additional_Data') if dir.endswith('.shp') and 'plots' in dir]
transect_file = [dir for dir in os.listdir(f'Data/Data/{chosen_file}/Additional_Data') if dir.endswith('.shp') and 'transect' in dir]
area_file= [dir for dir in os.listdir(f'Data/Data/{chosen_file}/Base_Data') if dir.endswith('.shp') and 'bdy' in dir]
class Main:
    def __init__(self):
        self.point_cloud = pcl.PointCloud(f'Data/Data/{chosen_file}/Base_Data/{tif_file[0]}')
        self.point_cloud.read_tif()
        self.path_finder = None
        self.shp_opener = Shp_Opener(f'Data/Data/{chosen_file}/Additional_Data/{shp_file[0]}',f'Data/Data/{chosen_file}/Additional_Data/{transect_file[0]}',f'Data/Data/{chosen_file}/Base_Data/{area_file[0]}')
        self.line_points = self.shp_opener.get_points()
        self.trans_points = self.shp_opener.get_trans()
        self.area_points = self.shp_opener.get_area()
    
    def find_path(self):
        self.path_finder = AntColony(self.line_points)
        best_path, best_distance,best_path_coors = self.path_finder.run()
        self.display_path(best_path_coors)
        self.save_path(best_path_coors)

    def save_path(self,best_path):
        with open('path.csv','w') as f:
            for x in best_path:
                f.write(f'{x[0]},{x[1]}\n')
    def find_path_transcect(self):

        connect_index = []
        no_connect = set()
        n = len(self.trans_points)
        for ind,trans in enumerate(self.trans_points):
            if ind % 3 ==2:
                for point in range(n):
                    if point != ind :
                        if point == ind-1 or point == ind+1:
                            pass
                        else:
                            no_connect.add((ind,point))
        self.path_finder = AntColony(self.trans_points,no_connect=no_connect)
        best_path, best_distance,best_path_coors = self.path_finder.run()
        self.display_path(best_path_coors)

    def display_path(self,best_path_coors):
        connections = [[i,i+1] for i in range(len(best_path_coors)-1)]
        points = [[x,y,z] for x,y in best_path_coors for z in [self.point_cloud.find_altitude([x,y],30)]]
        connections.append([len(best_path_coors)-1,0])
        self.point_cloud.create_line(points,connections,draw_point=True,interpolate=50)

    def identify_possible_locations(self, points, possible_locations):
        for x,_ in enumerate(points):
            human_height = [points[x][0],points[x][1],self.point_cloud.find_altitude(points[x],2)]
            points_visible = self.point_cloud.points_visible(human_height,points)
            if len(points_visible) == len(points):
                possible_locations.append(points[x])
                print('possible')
        if len(possible_locations) == 0:
            print('No points where every point is visible')
        else:
            print('Possible locations:')
            for x in possible_locations:
                # print(x)
                pass
    
main = Main()
main.find_path()
# main.find_path_transcect()

main.point_cloud.show()

