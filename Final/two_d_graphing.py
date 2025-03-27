import matplotlib.pyplot as plt
import numpy as np
import random
from ants import AntColony
from brute_force import BruteForce
from itertools import product
import math

class Graph2D:
    def __init__(self,shape,buffer):
        self.shape = shape
        self.buffer = buffer
        self.points = []
        self.transects = []
        self.ordered_points = []
        self.separated_points = []
        self.transect_path= None
        
    
    def plot(self):
        '''Adds the outline of the shape to the plot'''
        plt.plot(self.shape[:,0],self.shape[:,1], alpha=0.5)

    def plot_buffer(self):
        '''Adds the buffer to the plot, -25 from the shape file'''
        plt.plot(self.buffer[:,0],self.buffer[:,1], alpha=0.5)


    def show(self):
        '''Shows the plot in matplotlib'''
        plt.axis('equal')
        plt.legend(loc='upper left')
        plt.show()
    
    def generate_random_points(self,n):
        '''Generates n random points within the a rectangle around the shape'''
        smallest_x = min(self.shape[:,0])
        largest_x = max(self.shape[:,0])
        smallest_y = min(self.shape[:,1])
        largest_y = max(self.shape[:,1])
        x = np.random.uniform(smallest_x,largest_x,n)
        y = np.random.uniform(smallest_y,largest_y,n)
        return np.array(list(zip(x,y)))
    
    def remove_unneeded_points(self,points,n):
        '''
        1. Removes points that are not within the shape
        2. Removes points that are too close to each other
        3. Adds the first n of these to to_plot
        4. Sets self.points to to_plot

        '''
    
        def distance_to_other_points(to_plot,planned_plot,min_distance = 60):
            '''
            1. Gets all current points to plot calculates distance to the new point
            2. If any air within minimum distance, return False else True
            '''
            if len(to_plot) == 0:
                return True
            distance = np.linalg.norm(to_plot - planned_plot,axis=1)
            if distance.min() > min_distance:
                return True
            return False

        to_plot = []
        for point in points:
            if self.inside_shape(point):
                if distance_to_other_points(np.array(to_plot),point):
                    to_plot.append(point)
                    if len(to_plot) == n:
                        break
        self.points = np.array(to_plot)
        # plt.scatter(points[:,0],points[:,1])
    
    def inside_shape(self,point):
        '''Returns True if the point is within the shape, False otherwise'''
        x,y = point
        n = len(self.buffer)
        inside = False
        for i in range(n):
            x1,y1 = self.buffer[i]
            x2,y2 = self.buffer[(i+1)%n]
            if y1 < y and y2 >= y or y2 < y and y1 >= y:
                if x1 + (y - y1) / (y2 - y1) * (x2 - x1) < x:
                    inside = not inside
        return inside
    
    def plot_points(self):
        '''Adds the random points inside of the shape to the plot'''
        plt.scatter(self.points[:,0],self.points[:,1], alpha=0.5,s=10,c='purple')
    
    def create_transcets(self,distance = 25):
        '''
        1. Removes double point from the beggining / end of list
        2. For each point in the ordered points, create a transect (25m then 90* turn then 25m)
        3. Add the transect to self.transects if the transect is within the shape
        '''
        ordered_points = self.ordered_points 
        for point in ordered_points[:-1]:# Removes the double point at the begining / end of the list
            while True:
                transect = []
                random_angle = np.random.uniform(0,2*np.pi)
                transect.append(point)
                second_point = [point[0] + distance*np.cos(random_angle),point[1] + distance*np.sin(random_angle)]
                transect.append(second_point)
                angle = random_angle + np.pi / 2 if random.randint(0, 1) == 0 else random_angle - np.pi / 2

                third_point = [second_point[0] + distance*np.cos(angle + np.pi),second_point[1] + distance*np.sin(angle + np.pi)]
                transect.append(third_point)
                if self.inside_shape(third_point) and self.inside_shape(second_point):
                    break
            self.transects.append(transect)

    def plot_transcets(self):
        '''Adds the transects to the plot'''
        for transect in self.transects:
            plt.plot([transect[0][0],transect[1][0]],[transect[0][1],transect[1][1]])
            plt.plot([transect[1][0],transect[2][0]],[transect[1][1],transect[2][1]])

    def find_path(self,plot = True):
        ''' 
        1. Finds near optimum path between points
        2. separates into groups of 8
        3. If plot = True, plots the path
        '''
        ant_colony = AntColony(self.points)
        shortest_path = ant_colony.run()
        path_points = shortest_path[2]  # Extract the list of points from shortest_path

        # Unpack x and y coordinates
        x_values = [point[0] for point in path_points]
        y_values = [point[1] for point in path_points]
        self.ordered_points = path_points
        new_points = []
        for ind,points in enumerate(path_points):
            if len(new_points) == 7:
                new_points.append(points)
                self.separated_points.append(new_points)
                new_points = []
            else:
                new_points.append(points)    
        self.separated_points.append(new_points) 

        # Plot the path as a line
        if plot:
            plt.plot(x_values, y_values, marker='', linestyle='-', color='red',alpha=0.5)
        return shortest_path
    
    def find_transect_route(self,plot=True):
        '''
        1. Creates all possible permutations of the transects enter at first point leave at 3rd or the opposite
        2. Finds the shortest path out of these permutations
        3. If plot = True, plots the shortest path
        '''
        possible_perm = [[]]
        transects = self.transects

        for ind,_ in enumerate(transects):
            current_options = transects[ind][0],transects[ind][2]  # A list of 3 points (e.g., [A1, A2, A3])
            new_possible_perm = []
            
            # For each path in the current list of possible permutations, expand with the current options
            for option in possible_perm:
                choice1 = current_options[0]  # The first point of the current transect
                choice2 = current_options[1]  # The second point of the current transect
                
                new_possible_perm.append(option + [choice1,transects[ind][1],choice2])  # Add the new choice to the path
                new_possible_perm.append(option + [choice2,transects[ind][1],choice1])  # Add the new choice to the path
            
            possible_perm = new_possible_perm  # Update with new expanded options
        shortest_path = None
        shortest_length = float('inf')
        for path in possible_perm:
            length = self.route_length(path)
            if length < shortest_length:
                shortest_length = length
                shortest_path = path
        self.transect_path = shortest_path
        print(shortest_path,shortest_length)
        if plot:
            self.plot_fastest_transcets(shortest_path)

    def route_length(self,route):
        """
        Calculates the total length of a given route.
        
        """
        total_distance = 0.0
        
        for i in range(len(route) - 1):
            x1, y1 = route[i]
            x2, y2 = route[i + 1]
            total_distance += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)  # Euclidean distance
        
        return total_distance
    def plot_fastest_transcets(self,path_points):
        ''' Plots the fastest route between the transects'''
        x_values = [point[0] for point in path_points]
        y_values = [point[1] for point in path_points]
        plt.plot(x_values, y_values, marker='', linestyle='-', color='blue',label='Fastest Route transects',alpha=0.5)
    
    def plot_separated_points(self):
        '''Plots each cluster of points in a different color'''
        for ind,points in enumerate(self.separated_points):
            x_values = [point[0] for point in points]
            y_values = [point[1] for point in points]
            print(len(x_values))

            plt.plot(x_values, y_values, marker='', linestyle='-',label=f'Route: {ind}',alpha=0.5)

if __name__ == '__main__':
    shape = np.array([(2023854.8714770428, 5753817.783510998), (2023856.6475916533, 5753816.945746823), (2023858.3329843988, 5753815.937836503), (2023859.9114062735, 5753814.769497379), (2023861.3676395856, 5753813.451993504), (2023862.6876446742, 5753811.998027036), (2023863.8586952647, 5753810.421615783), (2023864.8695011646, 5753808.737958053), (2023865.710317114, 5753806.963286122), (2023866.373036739, 5753805.114709745), (2023866.8512707066, 5753803.210051195), (2023867.1404083252, 5753801.267673436), (2023867.237661996, 5753799.306303087), (2023867.1420940885, 5753797.344849876), (2023866.8546259806, 5753795.402224331), (2023866.378029175, 5753793.497155459), (2023865.7168985798, 5753791.648010181), (2023864.877608208, 5753789.872616255), (2023863.8682497246, 5753788.188090395), (2023862.6985544355, 5753786.610673249), (2023861.3797994666, 5753785.155572821), (2023819.046381466, 5753742.822154821), (2023817.5350904197, 5753741.457460812), (2023815.8928683428, 5753740.253510226), (2023814.1367384698, 5753739.222783184), (2023812.2849048006, 5753738.37596418), (2023744.317205041, 5753711.388789275), (2023715.559414576, 5753681.672405795), (2023714.219910855, 5753680.410042533), (2023712.7700270265, 5753679.276169475), (2023711.222042995, 5753678.28039005), (2023709.589069532, 5753677.431138089), (2023707.8849372335, 5753676.735606397), (2023706.1240793813, 5753676.199685833), (2023607.103135768, 5753650.977370006), (2023573.9571290135, 5753393.844711543), (2023567.6386409416, 5753327.500586792), (2023567.3560857142, 5753325.553987601), (2023566.8836806894, 5753323.644558533), (2023566.2259953064, 5753321.790768954), (2023565.3893911685, 5753320.010550039), (2023564.3819605084, 5753318.321121338), (2023563.2134479156, 5753316.738824214), (2023561.8951560792, 5753315.278963774), (2023560.4398364597, 5753313.955660833), (2023558.8615659487, 5753312.781715322), (2023557.1756107067, 5753311.768482481), (2023555.3982784972, 5753310.925763022), (2023553.546760948, 5753310.26170833), (2023551.6389672598, 5753309.782741617), (2023549.6933509766, 5753309.493495791), (2023547.7287314897, 5753309.396768644), (2023523.387016139, 5753309.396768644), (2023521.5144816982, 5753312.47917803), (2023511.1824559504, 5753313.552251908), (2023432.8656326523, 5753373.877372555), (2023431.250115839, 5753375.2648677565), (2023429.7912854447, 5753376.816270796), (2023428.5056810956, 5753378.513992494), (2023427.4078784515, 5753380.338784773), (2023391.424473152, 5753448.072253575), (2023390.3499572468, 5753450.460274784), (2023360.7165646465, 5753529.835433533), (2023360.03059677, 5753532.060306029), (2023359.607837407, 5753534.349822151), (2023356.415801088, 5753559.886112695), (2023302.8179988686, 5753600.765792354), (2023204.808580312, 5753639.9695597775), (2023202.924394447, 5753640.8391877385), (2023201.1404617806, 5753641.899373292), (2023199.4759881732, 5753643.138702456), (2023197.9488933855, 5753644.543832583), (2023196.575618155, 5753646.099636012), (2023195.3709471945, 5753647.789362925), (2023194.347850019, 5753649.594821685), (2023160.4811156206, 5753717.328290484), (2023159.7465721057, 5753718.980058152), (2023159.164174121, 5753720.6914035855), (2023158.738679699, 5753722.448345561), (2023142.8636479483, 5753803.940175211), (2023142.6045882958, 5753805.67003465), (2023142.497663854, 5753807.4159134375), (2023142.543692468, 5753809.164457674), (2023146.777034269, 5753869.489578325), (2023147.0099867717, 5753871.43632573), (2023147.43243364, 5753873.350909489), (2023148.0403150613, 5753875.214930011), (2023148.8277891553, 5753877.010473628), (2023149.7872881137, 5753878.720284751), (2023150.9095909307, 5753880.3279317), (2023152.1839120167, 5753881.817964618), (2023153.598004852, 5753883.176063945), (2023155.1382796783,5753884.389178031), (2023156.7899340987, 5753885.4456485715), (2023158.537095333, 5753886.33532264), (2023160.3629727587, 5753887.049650264), (2023162.250019272, 5753887.581766589), (2023237.2931836394, 5753904.821412457), (2023259.8448647903, 5753918.916213176), (2023261.74484434, 5753919.964856511), (2023263.74725714, 5753920.8014675975), (2023265.8285269064, 5753921.416196229), (2023267.9641488986, 5753921.80180463), (2023310.297566899, 5753927.09348188), (2023311.8581841134, 5753927.226749572), (2023472.7251725125, 5753934.635097721), (2023474.6271723893, 5753934.632153661), (2023476.52029173, 5753934.448547123), (2023478.3874091054, 5753934.08593865), (2023480.2116382495, 5753933.547607684), (2023481.9764807776, 5753932.838422905), (2023483.6659754002, 5753931.964798204), (2023517.150388648, 5753912.579085272), (2023581.5206145737, 5753926.778399815), (2023583.5208549122, 5753927.114311888), (2023585.5448313814, 5753927.24590818), (2023587.571728404, 5753927.171835292),(2023589.5807003656, 5753926.892855024), (2023591.5510860025, 5753926.411836549), (2023593.462620892, 5753925.733726898), (2023854.8714770428, 5753817.783510998)])
    buffer = np.array([(2023847.2376638427, 5753799.297708445), (2023804.9042458422, 5753756.964290445), (2023732.9374352396, 5753728.389233293), (2023701.187371741, 5753695.580834346), (2023589.0038140398, 5753667.005777194), (2023554.0787441898, 5753396.071901993), (2023547.7287314897, 5753329.396768644), (2023523.387016139, 5753329.396768644), (2023445.0701928409, 5753389.721889292), (2023409.0867875414, 5753457.455358094), (2023379.453394941, 5753536.830516843), (2023375.22005314, 5753570.6972512435), (2023312.7782615907, 5753618.322346494), (2023212.2363938391, 5753658.539093595), (2023178.3696594406, 5753726.272562394), (2023162.49462769, 5753807.764392044), (2023166.727969491, 5753868.089512696), (2023245.044792789, 5753886.081215345), (2023270.4448435903, 5753901.956247096), (2023312.7782615907, 5753907.247924346), (2023473.6452499898, 5753914.656272494), (2023513.8619970912, 5753891.372892595), (2023585.828807692, 5753907.247924346), (2023847.2376638427, 5753799.297708445)])
    graph = Graph2D(shape,buffer)
    graph.plot()
    graph.plot_buffer()
    random_points = graph.generate_random_points(150)
    graph.remove_unneeded_points(random_points,16)
    graph.plot_points()
    path = graph.find_path(False)
    # graph.plot_separated_points()
    graph.create_transcets()
    # graph.plot_transcets()
    graph.find_transect_route()
    graph.show()
     