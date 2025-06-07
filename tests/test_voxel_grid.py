import pytest
import numpy as np
from Config import Config
from voxel_grid import VoxelGrid
config = Config()


def test_voxel_grid_insertion():
    voxel_grid = VoxelGrid((0, 0, 0), 10)
    voxel_grid.insert((5, 5, 5))
    grid = voxel_grid.get_grid()
    assert len(grid) == 1, "Grid should contain one voxel."
    voxel_grid.insert((5,5,4))
    assert len(grid) == 1, "Grid should contain one voxel."
    voxel_grid.insert((15, 5, 5))
    assert len(grid) == 2, "Grid should contain two voxels."
    voxel_grid.insert((15, 5, 5))
    assert len(grid) == 2, "Should be able to deal with duplicate points in the same voxel."

def test_voxel_get_voxel_index():
    voxel_grid = VoxelGrid((0, 0, 0), 10)
    index = voxel_grid.get_voxel_index((5, 5, 5))
    assert index == (0, 0, 0), "Voxel index should be (0, 0, 0) for point (5, 5, 5)."
    index = voxel_grid.get_voxel_index((15, 5, 5))
    assert index == (1, 0, 0), "Voxel index should be (1, 0, 0) for point (15, 5, 5)."

def test_voxel_single_query():
    voxel_grid = VoxelGrid((0, 0, 0), 10)
    voxel_grid.insert((5, 5, 5))
    voxel_grid.insert((15, 5, 5))
    
    result = voxel_grid.single_query((5, 5, 5))
    assert result == [(5, 5, 5)], "Query should return the point (5, 5, 5)."
    
    result = voxel_grid.single_query((15, 5, 5))
    assert result == [(15, 5, 5)], "Query should return the point (15, 5, 5)."
    
    result = voxel_grid.single_query((20, 20, 20))
    assert result == [], "Query should return an empty list for a point not in the grid."

def test_voxel_safe_query():
    config.config['speed_related']['DVLOS_m'] = 5
    config.save_config()
    
    voxel_grid = VoxelGrid((0, 0, 0), 10)
    voxel_grid.insert((5, 5, 5))
    voxel_grid.insert((11, 5, 5))
    
    result = voxel_grid.safe_query((9, 5, 5))
    assert result == [(5, 5, 5), (11, 5, 5)], "Safe query should return both points within DVLOS distance."
 
    result = voxel_grid.safe_query((15, 5, 5))
    assert result == [(11, 5, 5)], "Safe query should return the point (15, 5, 5)."
    
    result = voxel_grid.safe_query((20, 20, 20))
    assert result == [], "Safe query should return an empty list for a point not in the grid."

def test_voxel_grid_empty():
    voxel_grid = VoxelGrid((0, 0, 0), 10)
    assert voxel_grid.get_grid() == {}, "Grid should be empty upon initialization."

    result = voxel_grid.single_query((5, 5, 5))
    assert result == [], "Query on an empty grid should return an empty list."

    result = voxel_grid.safe_query((5, 5, 5))
    assert result == [], "Safe query on an empty grid should return an empty list."

def test_voxel_grid_error_checking_index():
    voxel_grid = VoxelGrid((0, 0, 0), 10)
    with pytest.raises(ValueError):
        voxel_grid.get_voxel_index((5, 5))
        
def test_voxel_grid_error_checking_query():
    voxel_grid = VoxelGrid((0, 0, 0), 10)
    with pytest.raises(ValueError):
        voxel_grid.single_query((5, 5))  # Should be a tuple of three floats

def test_voxel_grid_error_insert():
    voxel_grid = VoxelGrid((0, 0, 0), 10)
    with pytest.raises(ValueError):
        voxel_grid.insert((5, 5))  # Should be a tuple of three floats

def test_voxel_grid_insert_with_numpy():
    voxel_grid = VoxelGrid((0, 0, 0), 10)
    numpy_array = np.array([5, 5, 5])
    voxel_grid.insert(tuple(numpy_array))
    grid = voxel_grid.get_grid()
    assert len(grid) == 1, "Grid should contain one voxel after inserting a numpy array point."
    
    
    

