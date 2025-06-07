# Configuration of UAV flight path software

## Distances
### buffer_m
States the distance from the border of the .shp file that the UAV is allowed to fly in. will always be a negative number.

### distance_to_nearest_point_m
Used in random point generation methods to which ensures that each point will be further apart than this distance.

### attempts_to_find_point_in_area
Used in random point generation to stop a infinite loop, increase when points need to be generated in small areas.

### height_above_ground_m
The distance the UAV will remain above the ground at all points in metres. Always a positive number.

### interpolation_distance_m
Used in the final generation of the path, forcing the drone to check its height above the ground at this distance. Decrease for more accurate following of terrain at the cost of more noise from pointcloud inaccuracies

### number_of_points_per_area
Used in random point generation to set the number of points to be generated in side each buffer area, If this number is not reached try reducing distance_to_nearest_point_m or increasing attempts_to_find_point_in_area.

### human_height_above_ground_m
States the human height which is then used for the DVLOS calculations. Must be more than DVLOS_m otherwise no points will be visible

### grid_size_m
Used in standard point generation to give the length of each side of the hexagon. To reduce number of points in an area increase this number. There number_of_points_per_area has no impact on this.

## current_map
### folder_location
Holds the last used folder to allow for faster restarts

### human_location
Holds a list of lists that can be used as human standing locations. Can be in form XY or XYZ assuming human_height_above_ground_m is set

## speed_related

### A*_grid_size_m
When A* is used to navigate through obstacles a grid of nodes is created. This value deems the distance between each node, the higher this value is the less optimal the route will be, however the faster the software will run.

### A*_grid_growth_multiplier
The number of expansions that the grid is allowed to take before failing CHECK THIS

### DVLOS_M
The distance allowed between any point on the path and the terrain. Lower this value is the closer to the terrain DVLOS can be. If set too low whilst interpolation_distance_m is high some terrain may be missed.

### DVLOS_interpolation_m
Distance between each point on the ray cast between two points when checking for visiblity 
### brute_force_cutoff
The maximum number of points where brute force will be attempted, maximum ~10. When brute force is not used a halistic method is used instead

### maximum_recusive_depth
The number of grids of increasing detail allowed to be produced before giving up.

## io
### output_folder
The path where all outputs will be saved

### graph_picture_name
The name of the photo used in the display

### input_folder
The path of the folder where all the inputs are taken from

### specific_folder_name
The name of the folder inside of the output_folder where all files will be saved to

### output_CSV_file_name
The name of the CSV file that will be imported into flylichi

### output_transect_file_name
The name of the .shp files holding information about the transects, and the buffers of the transects