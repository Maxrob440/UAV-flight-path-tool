from pathfinding import AntColony,BruteForce

def test_ant_colony_solve():
    cities = [(0,0),(2,2)]
    obstacles = [[]]
    ant_colony = AntColony(cities=cities,
                            obstacles=obstacles,
                            pointcloud=None,
                            step=1,
                            current_buffer=0,
                            human_location=None)
    path, cost = ant_colony.solve()
    assert path == [(0, 0), (2, 2),(0,0)]

def test_brute_force_solve():
    cities = [(0,0),(2,2)]
    obstacles = [[]]
    brute_force = BruteForce(cities=cities,
                            obstacles=obstacles,
                            pointcloud=None,
                            step=1,
                            current_buffer=0,
                            human_location=None)
    path, cost = brute_force.solve()
    assert path == [(0, 0), (2, 2),(0,0)]
