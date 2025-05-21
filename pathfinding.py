from itertools import permutations
import random
import heapq
import numpy as np
import matplotlib.pyplot as plt
from Config import Config

XYCoordinate = tuple[float, float]
XYCoordinates = list[XYCoordinate]
XYZCoordinate = tuple[float, float, float]
XYZCoordinates = list[XYZCoordinate]
XYCoordinateDistance = tuple[XYCoordinate, float]
PathDistance = tuple[XYCoordinates, float]

class Node:
    '''
    Holds the coordinates of the node and the coordinates of the neighbours.\n
    The neighbours are stored in a dictionary with the direction as the key and a list containing the neighbours coordinates and the distance to the neighbour as the value. [(x,y),distance]\n
    The distance is calculated using the calculate_node_distances method.\n
    nearest_important_point is the a list of the nearest city to the node and the distance to the city.[(x,y),distance]\n
    '''
    def __init__(self,
                 pathfinder:object,
                  coord:XYCoordinate,
                  step:float,
                  nearest_points=None)->None:
        
        if not isinstance(coord,(tuple,list)):
            raise TypeError("Expected a tuple or list for coordinate")
        if len(coord) != 2:
            raise ValueError("Coord expected to be in (X,Y) form")
        
        if not isinstance(step,(float,int)) or step <= 0:
            raise ValueError('Step should be a positive number greater than 0')


        x,y= coord
                
        self.neighbours={
                    'left':[(x - step, y), None],
                    'right':[(x + step, y), None],
                    'up':[(x, y + step), None],
                    'down':[(x, y - step), None],
                    'upright':[(x + step, y + step), None],
                    'downright':[(x + step, y - step), None],
                    'upleft':[(x - step, y + step), None],
                    'downleft':[(x - step, y - step), None],
                    'upupleft':[(x - step, y + 2*step), None],
                    'upupright':[(x + step, y + 2*step), None],
                    'leftleftup':[(x - 2*step, y + step), None],
                    'leftleftdown':[(x - 2*step, y - step), None],
                    'downdownleft':[(x - step, y - 2*step), None],
                    'downdownright':[(x + step, y - 2*step), None],
                    'rightrightup':[(x + 2*step, y + step), None],
                    'rightrightdown':[(x + 2*step, y - step), None]
        }
        if nearest_points is not None:
            for key in nearest_points:
                self.neighbours[key] = [nearest_points[key],None]

        self.coord = coord
        self.pathfinder = pathfinder

    def __repr__(self):
        return f'{self.coord}'

    def calculate_node_distances(self)->None:
        '''
        Calculates the 2D distances to each neighbour stored in the nodes dictionary.
        If the line between the node and the neighbour crosses an obstacle, the distance is set to np.inf.
        '''
        for neighbour in self.neighbours.items():
            # if neighbour[0] == 'nearest_important_point':
            #     continue
            direction,coordinate = neighbour[0],neighbour[1][0]
            
            if not isinstance(coordinate,tuple):
                raise TypeError(f"Expected a tuple for coordinates, got {coordinate}")
                
            if any(self.pathfinder.line_crosses_polygon(self.coord,coordinate,obstacle) for obstacle in self.pathfinder.obstacles):
                self.neighbours[direction][1] = np.inf
            
            elif direction[0] is not None:
                self.neighbours[direction][1] = np.linalg.norm(np.array(self.coord)-np.array(coordinate))
        

class Pathfinder():
    '''
    Parent class for any pathfinding class.\n
    '''
    def __init__(self,
                 cities:XYCoordinates, 
                 obstacles:list[XYCoordinates],
                 pointcloud,
                 human_location:XYZCoordinate,
                 step:float,
                 current_buffer)->None:
        self.config = Config()
        self.cities = cities
        self.obstacles = obstacles
        self.pointcloud = pointcloud
        self.human_location = human_location
        self.number_of_times_grid_generated = 0
        self.grid_distances = []
        self.current_buffer = current_buffer
        self.line_crossing_cache={}

        self.distances = self.calculate_distances(step)
        self.dijkstras_extra_points = []
        self.grid_points = []
        self.best_path = None
        self.best_distance = float('inf')
        self.best_path_coors = None
    
    def reconstruct_path(self,
                         came_from:dict,
                         current:XYCoordinate)->PathDistance:
        '''
        Reconstructs the path from the start node to the goal node using the came_from dictionary.\n
        came_from is a dictionary that stores the previous node for each node in the optimum path. {(x,y): (x,y)}\n
        The path is reconstructed by starting from the goal node and following the previous nodes until the start node is reached.\n
        The path is returned as a list of nodes and the distance of the path.\n
        '''
        if not isinstance(current,tuple):
            raise TypeError(f"Current must be a hashable tuple, recived {type(current)}")

        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        optimum_path_distance = 0
        for i in range(len(total_path)-1):
            optimum_path_distance += np.linalg.norm(np.array(total_path[i])-np.array(total_path[i+1]))
        #Path is reconstructed in reverse order, so we need to reverse it again.
        total_path.reverse()
        return total_path,optimum_path_distance
    
    def h(self,
          node:Node,
          goal:Node):
        '''
        Heuristic function for the A* algorithm used to estimate the distance between the current node and the goal node.\n
        Returns the 2D euclidean distance between the current node and the goal node.\n
        Ignoring all obstacles and line of sight restrictions.\n
        '''
        if isinstance(node,Node) and isinstance(goal,Node):
            return np.linalg.norm(np.array(node.coord)-np.array(goal.coord))
        elif isinstance(node,(tuple,list)) and isinstance(goal,(tuple,list)):
            if len(node) != 2 or len(goal) != 2:
                raise TypeError("Coord expected to be in (X,Y) form")
            return np.linalg.norm(np.array(node)-np.array(goal))
        else:
            raise TypeError(f"Expected a Node or tuple for coordinates, both of same type, got {type(node)} and {type(goal)}")
    
    def a_star(self,
               start:XYCoordinate,
               goal:XYCoordinate,
               adjustment:int=1,
               step:float = 0.5,
               recusive_depth = 0)->PathDistance:
        '''
        A* algorithm for pathfinding.\n
        Input is a tuple of coordinates.\n
        Computationally expensive function\n

        First produces a grid of nodes with the given adjustment:
            Adjustment is the amount of space around the cities that will be used to create the grid.\n
            If a path is unable to be found, the function will call itself recursively with a higher adjustment value.\n
            This will create a larger grid and allow for more possible paths.\n
        '''
        ### MESSSYY
        if isinstance(start,Node):
            raise TypeError(f"Expected a tuple for coordinates, got {type(start)}")
        
        if self.obstacles == [[]]: # if no obstacles return the euclidean route
            return [start,goal],np.linalg.norm(np.array(start)-np.array(goal))
        if not self.line_crosses_polygon(start,goal,self.obstacles[self.current_buffer]): # if doesnt cross any obstacles return euclidean route
            return [start,goal],np.linalg.norm(np.array(start)-np.array(goal))


        if self.grid_distances== [] or recusive_depth > 0:
            self.number_of_times_grid_generated =self.number_of_times_grid_generated+1
            self.calculate_grid_distances(self.number_of_times_grid_generated,
                                          step/self.number_of_times_grid_generated)
        if isinstance(start,tuple):
            start = self.grid_distances[start]
            goal = self.grid_distances[goal]




        open_set = {start.coord}

        came_from = {}
        g_score = {node.coord: np.inf for node in self.grid_distances.values()}
        g_score[start.coord] = 0
        f_score = {node.coord: np.inf for node in self.grid_distances.values()}
        f_score[start.coord] = self.h(start,goal)

        while open_set:
            current = min(open_set, key=lambda o: f_score[o])

            if current == goal.coord:
                return self.reconstruct_path(came_from,current)
            open_set.remove(current)

            current_node = self.grid_distances[current]
            all_options = [x for x in current_node.neighbours.values()]
            for direction in all_options:
                if direction[0] not in self.grid_distances or direction[1] == np.inf:
                    pass
                else:
                    tentative_g_score = g_score[current] + direction[1]
                    if tentative_g_score < g_score[direction[0]]:
                        came_from[direction[0]] = current
                        g_score[direction[0]] = tentative_g_score
                        f_score[direction[0]] = g_score[direction[0]] + self.h(self.grid_distances[direction[0]],goal)
                        if direction[0] not in open_set:
                            open_set.add(direction[0])
        growth_multiplier = int(self.config.config['speed_related']['A*_grid_growth_multiplier'])
        maximum_recursive_depth = int(self.config.config['speed_related']['maximum_recusive_depth'])
        if self.number_of_times_grid_generated < maximum_recursive_depth:
            return self.a_star(start.coord,goal.coord,self.number_of_times_grid_generated,step,recusive_depth=recusive_depth+1)

        else:
            print('A STAR FAILED')
            debug = False
            if debug: #Debugging to show the grid to allow for debugging of the A* algorithm.
                for obstacle in self.obstacles:
                    x= [point[0] for point in obstacle]
                    y= [point[1] for point in obstacle]
                    plt.plot(x,y,c='r')
                plt.scatter(start.coord[0],start.coord[1],c='y',marker='o')
                plt.scatter(goal.coord[0],goal.coord[1],c='y',marker='o')
                x= [point[0] for point in self.cities]
                y= [point[1] for point in self.cities]
                plt.scatter(x,y,c='g',marker='o')
                x= [point[0] for point in self.grid_distances]
                y= [point[1] for point in self.grid_distances]
                plt.scatter(x,y,c='b', marker='o', s=1)
                plt.show()
            return [],np.inf
            

    def calculate_each_nearest(self)->None:
        '''
        For every generated node in the grid, find the nearest city and store it in the node.\n
        '''
        for node in self.grid_distances.values():
            nearest = []
            for city in self.cities:
                if any(self.line_crosses_polygon(node.coord,
                                                 city,
                                                 obstacle)
                                                 for obstacle in self.obstacles):
                    nearest.append((np.inf,city))
                else:
                    nearest.append((np.linalg.norm(np.array(node.coord)-np.array(city)),city))
            nearest.sort(key=lambda x: x[0])
            node.neighbours['nearest_important_point'] = [nearest[0][1],nearest[0][0]]


    def calculate_grid_distances(self,
                                 adjustment:int,
                                 step:float=0.5)->None:
        '''
        Creates a grid of nodes arounds the cities with the adjustment value as a margin.\n
        The step represents the distance between each node in (m) increasing step dramatically increases the runtime.\n
        '''

        def find_nearest_point_cardianal(self,point):
            x0,y0 = point
            candidates = {
                'up': [],
                'down': [],
                'left': [],
                'right': []
            }

            for node in self.grid_distances.values():
                x,y = node.coord
                dx,dy= x0-x,y0-y
                if dy > 0:
                    candidates['up'].append((x, y))
                if dy < 0:
                    candidates['down'].append((x, y))
                if dx > 0:
                    candidates['left'].append((x, y))
                if dx < 0:
                    candidates['right'].append((x, y))
            nearest = {}
            for items in candidates.items():
                if len(items[1]) > 0:
                    nearest[items[0]]=min(items[1], key=lambda x: np.linalg.norm(np.array(x) - np.array(point)))
            return nearest

        # shgould never be called from anythign but the a_star function so this code shouldnt be accessed
        if self.obstacles == [[]]:
            min_x = int(min(x[0] for x in self.cities) - adjustment)
            max_x = int(max(x[0] for x in self.cities) + adjustment)
            min_y = int(min(y[1] for y in self.cities) - adjustment)
            max_y = int(max(y[1] for y in self.cities) + adjustment)
        else:
            min_x = int(min(x[0] for x in self.obstacles[self.current_buffer]) - adjustment)
            max_x = int(max(x[0] for x in self.obstacles[self.current_buffer]) + adjustment)
            min_y = int(min(y[1] for y in self.obstacles[self.current_buffer]) - adjustment)
            max_y = int(max(y[1] for y in self.obstacles[self.current_buffer]) + adjustment)

        nodes = {}

        # create all nodes
        for x in np.arange(min_x, max_x + step, step):
            for y in np.arange(min_y, max_y + step, step):
                coord = (x, y)
                nodes[coord] = Node(
                    self,
                    coord = (x,y),
                    step=step)

       #Calculate distances to neighbours
        for node in nodes.values():
            node.calculate_node_distances()

        self.grid_distances = nodes

        needed_points = self.cities
        for point in needed_points:
            nearest_points = find_nearest_point_cardianal(self,point)
            if point not in nodes: # ensures the creation of the city points if miss aligned with grid
                x,y= point
                nodes[point] = Node(
                    self,
                    point,
                    step=step,
                    nearest_points=nearest_points)
                nodes[point].calculate_node_distances()

        self.calculate_each_nearest()
        
        print(f"Generated {len(nodes)} nodes.")
                    
        
    
    def calculate_distances(self,step)->dict[tuple[XYCoordinate,XYCoordinate],float]:
        '''
        Calculates the distances between all cities and stores them in a dictionary.\n
        if the cities are the same the distance is set to np.inf.\n
        if the line between two cities crosses an obstacle or the line of sight is blocked, the distance is calculated with A*.\n
        otherwise the distance is calculated using the euclidean distance.\n
        The distances are stored in a dictionary with the cities as the key (start,end) and the distance as the value.\n
        '''
        print('Calculating distances...')
        distances= {}
        n = len(self.cities)
        for i in range(n):
            for j in range(n):
                start = self.cities[i]
                end = self.cities[j]
                if i == j:
                    distances[(start,end)] = np.inf
                    continue
                if self.obstacles:
                    if any(self.line_crosses_polygon(start,end,obstacle) for obstacle in self.obstacles):
                        distances[(start,end)] = self.a_star(start,end,adjustment=1,step=step)
                        continue
                # elif not self.all_points_visible_interpolated_line(start,end):
                #     distances[(start,end)] = self.a_star(start,end,adjustment=1)
                distances[(start,end)] = [[start,end],np.linalg.norm(np.array(start) - np.array(end))]
        return distances

    def all_points_visible_interpolated_line(self,
                                             start:XYCoordinate,
                                             end:XYCoordinate)->bool:
        '''
        Generates a line between two points and checks if all points on the line are visible from the human location.\n
        Will create a point along the line every 10m.\n
        '''
        height_above_ground = float(self.config.config['distances']['height_above_ground_m'])
        length = int(np.linalg.norm(np.array(start)-np.array(end))/10)
        x= np.linspace(start[0],end[0],num=length)
        y= np.linspace(start[1],end[1],num=length)
        z= [self.pointcloud.find_altitude((x[i],y[i]),height_above_ground) for i in range(length)]
        line_between_two_points = np.array([x,y,z]).T
        for point_on_line in line_between_two_points:
            if not self.pointcloud.two_points_visible(self.human_location,point_on_line,plot=False):
                return False
        return True
    


    def on_segment(self,p, q, r):
            """Check if point q lies on the segment pr."""
            return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0])) and (min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))

    def orientation(self,p, q, r):
        """Return the orientation of the triplet (p, q, r)."""
        listed = [p, q, r]
        for i in listed:
            if not isinstance(i, tuple) or len(i) != 2:
                raise ValueError(f"Expected a tuple of length 2, got {i}")


        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0  # collinear
        elif val > 0:
            return 1  # clockwise
        else:
            return 2  # counterclockwise

    def do_intersect(self,p1, q1, p2, q2):
        """Returns True if the line segments p1q1 and p2q2 intersect."""
        if not isinstance(p1, tuple) or not isinstance(q1, tuple) or not isinstance(p2, tuple) or not isinstance(q2, tuple):
            raise ValueError(f"Expected tuples for coordinates got {p1}, {q1}, {p2}, {q2}")
        o1 = self.orientation(p1, q1, p2)
        o2 = self.orientation(p1, q1, q2)
        o3 = self.orientation(p2, q2, p1)
        o4 = self.orientation(p2, q2, q1)
        
        # General case
        if o1 != o2 and o3 != o4:
            return True
        
        # Special cases
        # p1, q1, p2 are collinear and p2 lies on segment p1q1
        if o1 == 0 and self.on_segment(p1, p2, q1):
            return True
        # p1, q1, p2 are collinear and q2 lies on segment p1q1
        if o2 == 0 and self.on_segment(p1, q2, q1):
            return True
        # p2, q2, p1 are collinear and p1 lies on segment p2q2
        if o3 == 0 and self.on_segment(p2, p1, q2):
            return True
        # p2, q2, p1 are collinear and q1 lies on segment p2q2
        if o4 == 0 and self.on_segment(p2, q1, q2):
            return True
        
        return False

    def line_crosses_polygon(self,
                             line_start:XYCoordinate, 
                             line_end:XYCoordinate,
                             polygon:XYCoordinates)->bool:
        """Check if the line segment (line_start, line_end) crosses the polygon."""
        if not isinstance(line_start, tuple) or not isinstance(line_end, tuple):
            raise ValueError("Expected tuples for coordinates got {line_start} and {line_end}")
        if not isinstance(polygon,list):
            raise ValueError("Expected a list for coordinates")
        
        cache = self.line_crossing_cache
        if (line_start,line_end) in cache:
            return cache[(line_start,line_end)]
        if (line_end,line_start) in cache:
            return cache[(line_end,line_start)]
        n = len(polygon)
        
        # Check all edges of the polygon
        for i in range(n):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % n]  # next vertex, wrap around to 0 if at the last vertex
            if self.do_intersect(line_start, line_end, p1, p2):
                # cache[(line_start,line_end)] = True
                return True
        # self.line_crossing_cache[(line_start,line_end)] = False
        return False
    
    
    
    
    def solve(self)->PathDistance:
        '''
        Method to solve the pathfinding problem.\n
        Needs to be implemented by a child class.\n
        Will return the path as a list of tuples of the coordinates and the distance of the path.\n
        '''    

class AntColony(Pathfinder):
    '''
    Ant Colony Optimization for solving the TSP approximately.\n
    Uses probabilistic decision-making based on pheromone trails and distances.\n
    Much faster than brute force for large problems but does not guarantee an optimal solution.
    '''
    def __init__(self, 
                 cities:XYCoordinates, 
                 obstacles:list[XYCoordinates], 
                 pointcloud,
                 human_location:XYZCoordinate, 
                 step:float,
                 current_buffer:int,
                 n_ants=10, n_iterations=50, alpha=1.0, beta=2.0, evaporation=0.5, q=100):
        super().__init__(cities, obstacles, pointcloud, human_location,step,current_buffer)
        self.n_ants = n_ants
        self.n_iterations = n_iterations
        self.alpha = alpha
        self.beta = beta
        self.evaporation = evaporation
        self.q = q
        self.pheromones = {
            (a, b): 1 for a in cities for b in cities if a != b
        }


    def probability(self, current, unvisited):
        pheromones = np.array([
            self.pheromones[(current, city)] ** self.alpha for city in unvisited
        ])
        distances = np.array([
            (1 / self.distances[(current, city)][1]) ** self.beta for city in unvisited
        ])
        probs = pheromones * distances
        total = np.sum(probs)
        if total == 0 or np.isnan(total):
            return np.ones(len(unvisited)) / len(unvisited)
        return probs / total

    def solve(self):
        print('Solving TSP with Ant Colony Optimization...')
        cities = self.cities
        for _ in range(self.n_iterations):
            all_paths = []
            all_distances = []

            for _ in range(self.n_ants):
                unvisited = cities[1:]
                current = cities[0]
                path = [current]

                while unvisited:
                    probs = self.probability(current, unvisited)
                    next_city = random.choices(unvisited, weights=probs, k=1)[0]


                    path.append(next_city)
                    unvisited.remove(next_city)
                    current = next_city

                path.append(cities[0])  # Return to start
                distance = sum(
                    self.distances[(path[i], path[i + 1])][1] for i in range(len(path) - 1)
                )
                all_paths.append(path)
                all_distances.append(distance)

                if distance < self.best_distance:
                    self.best_distance = distance
                    self.best_path = path

            # Evaporate and update pheromones
            for key in self.pheromones:
                self.pheromones[key] *= (1 - self.evaporation)
            for path, dist in zip(all_paths, all_distances):
                for i in range(len(path) - 1):
                    self.pheromones[(path[i], path[i + 1])] += self.q / dist

        # Expand best path into waypoints using pathfinding
        path_fillers = []
        for point in self.best_path:
            if not path_fillers:
                path_fillers.append(point)
                continue
            path_between = self.distances[(path_fillers[-1], point)][0]
            path_fillers.extend(path_between[1:])  # skip first point to avoid duplication

        self.best_path = path_fillers
        return self.best_path, self.best_distance

class BruteForce(Pathfinder):
    '''
    Exact solution for the TSP using brute force.\n
    Generates all possible permutations of the cities starting and ending at city 0.\n
    Time complexity is O(n!), not suitable where n > 8.\n
    '''
    def __init__(self, cities:XYCoordinates,
                  obstacles:list[XYCoordinates],
                  pointcloud:object,
                  human_location:XYZCoordinate,
                  step:float,
                  current_buffer:int
                  )->None:
        super().__init__(cities,obstacles,pointcloud,human_location,step,current_buffer=current_buffer)


    def solve(self)->tuple[XYCoordinates,float]:
        print('Solving TSP with brute force...')
        start_city = self.cities[0]
        remaining_cities = self.cities[1:]
        city_permutations = permutations(remaining_cities)
        for perm in city_permutations:
            path = [start_city] + list(perm) + [start_city]
            distance = 0
            for i in range(len(path) - 1):
                distance += self.distances[(path[i],path[i + 1])][1]
            if distance < self.best_distance:
                self.best_distance = distance
                self.best_path = path
        path_fillers = []


        for point in self.best_path:
            if not path_fillers:
                path_fillers.append(point)
                continue
            path_between = self.distances[(path_fillers[-1], point)][0]
            path_fillers.extend(path_between[1:])  # skip first point to avoid duplication
        self.best_path = path_fillers
        return self.best_path,self.best_distance

class Christofides(Pathfinder):
    '''
    Uses Christofides algorithm to solve the TSP approximatley with guarrentee of less than 1.5 times the optimal solution.\n
    '''
    def __init__(self,
                 cities:XYCoordinates,
                  obstacles:list[XYCoordinates],
                  pointcloud:object,
                  human_location:XYZCoordinate,
                  step:float,
                  current_buffer:int):
        super().__init__(cities,obstacles,pointcloud,human_location,step,current_buffer=current_buffer)

    def create_minimum_spanning_tree(self,
                                     start:tuple)->list[tuple[XYCoordinate,XYCoordinate]]:
        '''
        Transforms the graph of connections into a minimum spanning tree\n
        Uses Prim's algorithm to find the minimum spanning tree.\n
        The minimum spanning tree is a subset of the edges that connects all vertices with the minimum possible total edge weight.\n
        '''
        visited = {start}
        unvisited = set(self.cities) - {start}
        mst_edges = []

        while unvisited:
            shortest_edge = float('inf')
            shortest_edge_points = None

            for point1 in visited: #Find shortest connection between visited and unvisited
                for point2 in unvisited:
                    if self.distances[(point1, point2)][1] < shortest_edge:
                        shortest_edge = self.distances[(point1, point2)][1]
                        shortest_edge_points = (point1, point2)
            if not shortest_edge_points:
                raise ValueError('Graph is disconnected')
            
            mst_edges.append(shortest_edge_points)
            visited.add(shortest_edge_points[1])
            unvisited.remove(shortest_edge_points[1])
        return mst_edges
    
    def find_odd_verticies_mms(self,mst)->set:
        '''
        Finds any vertice that has been visited on odd number of times\n
        '''
        odd_vertices = set()
        for connection in mst:
            for point in connection:
                if point not in odd_vertices:
                    odd_vertices.add(point)
                else:
                    odd_vertices.remove(point)
        return odd_vertices
    
    def minimum_weight_matching(self,odd_vertices:set)->list[tuple[XYCoordinate,XYCoordinate]]:
        possible_matches = []
        possible_matches_distance = []

        for vert1 in odd_vertices:
            for vert2 in odd_vertices:
                if vert1 == vert2:
                    continue
                if (vert1,vert2) in possible_matches:
                    continue
                if (vert2,vert1) in possible_matches:
                    continue
                possible_matches.append((vert1,vert2))
                possible_matches_distance.append(((vert1,vert2),self.distances[(vert1,vert2)][1]))
        possible_matches_distance.sort(key=lambda x: x[1])
        matched = set()
        matching = []
        for (u,v),distance in possible_matches_distance:
            if u not in matched and v not in matched:
                matching.append(((u,v),distance))
                matched.add(u)
                matched.add(v)
        return matching

    def form_multigraph(self,mst,matched):
        multigraph_connections = []
        for connection in mst:
            multigraph_connections.append(connection)
        for connection in matched:
            multigraph_connections.append(connection[0])
        return multigraph_connections
    
    def eulerian_tour(self,multigraph):
        seen = []
        vertices = []
        connections = []

        #Make bidirections
        for connection in multigraph:
            connections.append(connection)
            connections.append((connection[1],connection[0]))

        connections = list(connections)


        for connection in connections:
            if connection not in seen:
                seen.append(connection)
                vertices.append(connection[0])
        return vertices

    def generate_tsp(self,connections):
        start = connections[0]
        unvisited = set(connections)
        unvisited.remove(start)
        tour = [start]
        
        current = start

        while unvisited:
            nearest, min_dist = None, float('inf')
            for neighbor in unvisited:
                dist = self.distances[(current, neighbor)][1]
                if dist < min_dist:
                    nearest, min_dist = neighbor, dist

            if nearest is None:
                break  
            
            tour.append(nearest)
            unvisited.remove(nearest)
            current = nearest
        
        # Return to start,very unoptimal, main cause of error
        tour.append(start)

        return tour


    def solve(self):
        mst = self.create_minimum_spanning_tree(self.cities[0])
        # print(mst)
        odd_vertices = self.find_odd_verticies_mms(mst)
        # print(odd_vertices)
        matching = self.minimum_weight_matching(odd_vertices)
        # print(matching)
        multigraph = self.form_multigraph(mst,matching)
        # print(multigraph)
        vertices = self.eulerian_tour(multigraph)
        # print(vertices)
        tsp = self.generate_tsp(vertices)
        print(tsp)
        distance = 0
        for i in range(len(tsp) - 1):
            distance += self.distances[(tsp[i],tsp[i + 1])][1]
        return tsp,distance



if __name__ == '__main__':
    cities = [(0, 0), (1, 2), (2, 4), (3, 1)]
    obstacles = [[(0.5, 0.5), (1.5, 0.5), (1.5, 1.5), (0.5, 1.5)]]
    pointcloud = None
    human_location = (0, 0, 0)
    step = 1
    current_buffer = 0

    pathfinder = Christofides(cities, obstacles, pointcloud, human_location, step, current_buffer)
    print(pathfinder.solve())

