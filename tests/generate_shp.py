import geopandas as gpd
from shapely.geometry import Polygon

# Define square coordinates (clockwise or counter-clockwise)
square = Polygon([
    (0, 0),
    (0, 10),
    (10, 10),
    (10, 0),
    (0, 0)
])

# Create GeoDataFrame with one polygon
gdf = gpd.GeoDataFrame(index=[0], geometry=[square])
gdf.crs = "EPSG:4326"  # Set coordinate system to WGS84

# Save to shapefile
gdf.to_file("test_square.shp")
