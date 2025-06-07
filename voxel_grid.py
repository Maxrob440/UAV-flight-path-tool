'''
Holds Voxel grid for spatial queries
'''
from collections import defaultdict
from Config import Config
import numpy as np
import math
XYZCoordinate  = tuple[float, float, float]


class VoxelGrid:
    '''
    A 3D voxel grid that allows for efficient spatial queries.
    '''
    def __init__(self,origin,voxel_size):
        self.origin = origin
        self.voxel_size = voxel_size
        self.grid = defaultdict(list)
        self.config = Config()
    
    def get_grid(self):
        return self.grid

    def get_voxel_index(self,point:XYZCoordinate)->XYZCoordinate:
        """
        Converts a point in 3D space to a voxel index in the grid.
        The voxel index is calculated by taking the difference between the point and the origin,
        then dividing by the voxel size. The result is floored to get the index of the voxel. 
        Args:
            point (XYZCoordinate): A tuple of (x,y,z)

        Returns:
            XYZCoordinate: a tuple representing the base point in the grid.
        """        
        x, y, z = point
        
        # x-self.origin[x] gives the distance from the origin to the point
        # then we floor divide by the voxel size to get the index

        voxel_x = (x - self.origin[0]) // self.voxel_size
        voxel_z = (z - self.origin[2]) // self.voxel_size
        voxel_y = (y - self.origin[1]) // self.voxel_size
        
        return (voxel_x, voxel_y, voxel_z)

    def insert(self,point:XYZCoordinate) -> None:
        """
        Adds a point to the voxel grid, the voxel index is calculated using the get_voxel_index method.

        Args:
            point (XYZCoordinate): tuple of (x,y,z) coordinates to be added to the grid.
        """
        if not isinstance(point, tuple|np.ndarray) or len(point) != 3:
            raise ValueError(f"Point must be a tuple of three floats (x, y, z), recieved {type(point)} of value {point}.")
        
        voxel_index = self.get_voxel_index(point)
        self.grid[voxel_index].append(point)

    def single_query(self,point:XYZCoordinate) -> list[XYZCoordinate]:
        """Returns all points inside of the voxel that contains the point.

        Args:
            point (XYZCoordinate): A tuple of (x,y,z) coordinates
        Raises:
            ValueError: If the point is not a tuple of three floats.
        Returns:
            list[XYZCoordinate]: All points inside of the voxel that contains the point.
        """        

        if not isinstance(point, tuple) or len(point) != 3:
            raise ValueError("Point must be a tuple of three floats (x, y, z).")
        
        voxel_index = self.get_voxel_index(point)
        return self.grid[voxel_index]

    def safe_query(self,point:XYZCoordinate) -> list[XYZCoordinate]:
        voxel = self.get_voxel_index(point)
        voxel_points = list(self.grid.get(voxel, []))  # Avoid KeyError if empty
        dvlos_distance = float(self.config.config['speed_related']['DVLOS_m'])
        seen = set(map(tuple, voxel_points))

        min_x, min_y, min_z = voxel
        search_range = math.ceil(dvlos_distance / self.voxel_size)

        for dx in range(-search_range, search_range + 1): #all possible x
            for dy in range(-search_range, search_range + 1): # all possible y
                for dz in range(-search_range, search_range + 1): # all possible z
                    neighbor_voxel = (
                        min_x + dx * self.voxel_size,
                        min_y + dy * self.voxel_size,
                        min_z + dz * self.voxel_size
                    )
                    if neighbor_voxel == voxel:
                        continue  # Already added

                    # query the centre of the neighbor voxel
                    query_result = self.single_query((
                        neighbor_voxel[0] + self.voxel_size / 2,
                        neighbor_voxel[1] + self.voxel_size / 2,
                        neighbor_voxel[2] + self.voxel_size / 2
                    ))

                    for pt in query_result:
                        if tuple(pt) not in seen and math.dist(pt, point) <= dvlos_distance: # only add if not seen and within DVLOS distance
                            voxel_points.append(pt)
                            seen.add(tuple(pt))

        return voxel_points


if __name__ == "__main__":
    grid = VoxelGrid((0, 0, 0), 1.0)
    random_points = [
        (0.5, 0.5, 0.5),
        (1.5, 1.5, 1.5),
        (2.5, 2.5, 2.5),
        (0.1, 0.1, 0.1),
        (1.9, 1.9, 1.9)
    ]
    for point_1 in random_points:
        grid.insert(point_1)

    print(grid.safe_query((0.5, 0.5, 0.5)))
