# UAV PROJECT 2.0.0

## Requirements

Each folder must contain:

- .tif file highlighting the area
- .shp file showing the areas to be assessed
- An optional but reccomended .txt file that contains mercader coordinates(x,y or x,y,z) of the available standing locations (allow DVLOS), examples in standing locations
- All modules downloaded, (pip install -r requirements.txt)

## Runtime

1. Run GUI.py
2. Use browse button to locate folder
3. From the top follow down pressing each button at least once (apart from the cycle buttons, and config)
4. Save output will save to the OUTPUT folder

## Limitations

- No flight area implemented
- Currenty line of sight is only guarrenteed for each plot and not the journey there.
- The final point in the csv output will always face north
- Ant colony config not implemented
- No valid input checks for the config page
- Default config still to be tuned
