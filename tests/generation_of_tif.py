import numpy as np
import rasterio
from rasterio.transform import from_origin

data = np.random.randint(0, 255, (100, 100)).astype('uint8')

transform = from_origin(0, 100, 1, 1)

with rasterio.open(
    'test_data.tif',
    'w',
    driver='GTiff',
    height=data.shape[0],
    width=data.shape[1],
    count=1,  # number of bands
    dtype=data.dtype,
    crs='EPSG:4326',  # WGS84 coordinate system
    transform=transform,
) as dst:
    dst.write(data, 1)
