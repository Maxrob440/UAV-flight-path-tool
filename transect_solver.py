import math
import random

class TransectSolver:
    def route_length(self,route):
        """
        Calculates the total length of a given route.
        :param route: A list of points (x,y) in the order they are visited
        
        """
        total_distance = 0.0
        for i in range(len(route) - 1):
            total_distance += math.dist(route[i][0], route[i + 1][0])

        return total_distance

class BruteTransectSolver(TransectSolver):
    def __init__(self, transects,
                 best_path_coords,
                 standing_location=None):
        self.transects = transects
        self.best_path_coords = best_path_coords
        self.standing_location = standing_location

    def solve(self):
        possible_perms = [[]]
        for i in self.best_path_coords:
            if i not in self.transects:
                continue
            transect = self.transects[i]
            issingle = len(transect) == 1
            if issingle:
                new_possible_perms = [x + [[transect[0],False]] for x in possible_perms]
            else:
                new_possible_perms= [x +[[coord,True] for coord in transect] for x in possible_perms]
                new_possible_perms+=[x + [[coord,True] for coord in reversed(transect)] for x in possible_perms]
                
                # new_possible_perms = [x + [[self.transects[i][0],True],[self.transects[i][1],True],[self.transects[i][2],True]] for x in possible_perms]
                # new_possible_perms += [x + [[self.transects[i][2],True],[self.transects[i][1],True],[self.transects[i][0],True]] for x in possible_perms]
            possible_perms = new_possible_perms

            if self.standing_location:
                for perm in possible_perms:
                    if perm[0]!=self.standing_location:
                        perm.insert(0,(self.standing_location[:2],False))

            routes = []
            #Compare the length of each possible path

        for path in possible_perms:
            length = self.route_length(path)
            routes.append((path,length))



        shortest_path = min(routes, key=lambda x: x[1])[0]
        return shortest_path
    
class RandomTransectSolver:
    def __init__(self, transects,
                best_path_coords,
                standing_location=None):
        self.transects = transects
        self.best_path_coords = best_path_coords
        self.standing_location = standing_location

    def solve(self):
        shortest_path=[]
        for i in self.best_path_coords:
            if len(self.transects[i]) == 1:
                shortest_path.append([self.transects[i][0],False])
            else:
                for point in self.transects[i]:
                    shortest_path.append([point,True])
        return shortest_path

class GreedyTransectSolver:
    def __init__(self, transects,
            best_path_coords,
            standing_location=None):
        self.transects = transects
        self.best_path_coords = best_path_coords
        self.standing_location = standing_location
    
    def solve(self):
        shortest_path = []
        for i in self.best_path_coords:
            if len(self.transects[i])==1:
                shortest_path.append((self.transects[i][0],False))
            else:
                if len(shortest_path)==0:
                    for point in self.transects[i]:
                        shortest_path.append((point,False))
                else:
                    dist_first_point = math.dist(shortest_path[-1][0],self.transects[i][0][:2])
                    dist_last_point = math.dist(shortest_path[-1][0],self.transects[i][-1][:2])
                
                    if dist_first_point <= dist_last_point:
                        shortest_path.extend([(x,True) for x in self.transects[i]])
                    else:
                        shortest_path.extend(reversed([(x,True) for x in self.transects[i]]))
        return shortest_path
    
