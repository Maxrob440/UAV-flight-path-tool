# UAV PROJECT 2.1.1

Work completed in collaboration with Interpine NZ<br/>
This is an initial draft

## Requirements

Python 3.12.x is required for this software to run.

Each folder must contain:
- .tif file highlighting the area
- .shp file showing the areas to be assessed
- An optional but reccomended .txt file that contains mercader coordinates(x,y or x,y,z) of the available standing locations (allow DVLOS), examples in standing locations


## Installation

git clone https://github.com/Maxrob440/UAV-flight-path-tool
cd 
python3 -m venv venv
source ./venv/bin/activate
pip install -r requirements.txt

## Runtime

1. Run GUI.py
2. Use browse button to locate folder
3. From the top follow down pressing each button at least once (apart from the cycle buttons, and config)
4. Save output will save to the OUTPUT folder

![Instructions](README/Instructions.PNG)

A 3D view is useable that will display as shown

![ThreeDimentionView](README/Vertical_view.PNG)

### Configuration

Configuration is available from the bottom left of the window
Main parts are listed below:

- buffer_m: Controls the minimum distance a drone can be from a border
- distance_to_nearest_point_m: Controls the proximity of point generation - minimum distance between two points
- height_above_ground: the height above the ground that the drone will remain at
- number_of_points_per_area: the number of points that will be generated inside each buffer
- human_height_above_ground_m: used to determine DVLOS, do not set bellow DVLOS_m<br>

Defaults are provided that will effectively work in most circumstances

## Limitations

- No flight area implemented
- The final point in the csv output will always face north
- Ant colony config not implemented
- No valid input checks for the config page
- Default config still to be tuned

## Acknowledgements
I would like to express my gratitude to my supervisor, Dr Milto Miltiadou, for her invaluable guidance, support, and constructive feedback throughout the course of this project.
I am also grateful to Sam West, Susana Gonzalez and Jack Guo from Interpine Innovation for their collaboration and practical insights, which provided valuable context and relevance to the project. Their input helped bridge the gap between academic research and real-world forestry applications.
