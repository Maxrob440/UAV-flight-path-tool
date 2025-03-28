import rasterio
from pyproj import Transformer
import numpy as np
from math import atan2,radians, degrees, sqrt, sin, cos



class Converter:
    '''
    The converter class takes an input of the LAS file
    '''
    def __init__(self,tif_file):
        self.tif_file = tif_file
        self.lat_long_coords = []

    def show_crs(self):
        '''Shows the CRS information'''
        with rasterio.open(self.tif_file) as src:
            print(src.crs)
    
    def convert_mercader_to_lat_long(self,coords):
        inital_z = coords[0][2]-20
        for x,y,z in coords:
            to_latlong = Transformer.from_crs("EPSG:2193", "EPSG:4326", always_xy=True)
            lon, lat = to_latlong.transform(x, y)
            self.lat_long_coords.append((lat, lon,z-inital_z))


    def convert_into_correct_form(self):
        bearings = []

        def calculate_bearing(lat1,long1,lat2,long2):
            lat1,long1,lat2,long2 = map(radians,[lat1,long1,lat2,long2]) #Convert radians
            delta_long = long2 - long1

            x = sin(delta_long) * cos(lat2)
            y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_long)
            bearing = atan2(x,y)
            return (degrees(bearing)+360) % 360
    
        for ind,point in enumerate(self.lat_long_coords):
            if ind == len(self.lat_long_coords)-1:
                break
            lat1,long1,z = point
            lat2,long2,z = self.lat_long_coords[ind+1]
            bearings.append(calculate_bearing(lat1,long1,lat2,long2))
        self.bearings = bearings
        self.bearings.append(0) #Add 0 to the end of the list to make it the same length as lat_long_coords
        return bearings #Small errors here about a degree
    
    def create_csv(self):
        import pandas as pd
        df = pd.DataFrame(columns=["latitude","longitude","altitude(m)","heading","curvesize(m)","rotationdir","gimbalmode","gimbalpitchangle","actiontype1","actionparam1","actiontype2","actionparam2","actiontype3","actionparam3","actiontype4","actionparam4","actiontype5","actionparam5","actiontype6","actionparam6","actiontype7","actionparam7","actiontype8","actionparam8","actiontype9","actionparam9","actiontype10","actionparam10","actiontype11","actionparam11","actiontype12","actionparam12","actiontype13","actionparam13","actiontype14","actionparam14","actiontype15","actionparam15","altitudemode","speed(m/s)","poi_latitude","poi_longitude","poi_altitude(m)","poi_altitudemode","photo_timeinterval","photo_distinterval"])
        for ind,point in enumerate(self.lat_long_coords):
            df.loc[ind]=([point[0],point[1],point[2],self.bearings[ind],0.2,0,0,0,5,-90,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,0,0,0,0,0,0,-1,-1])
        df.to_csv('Drone_path.csv', index=False)
    





if __name__ == '__main__':
    test_converter = Converter('Data/T691/6_OFO_T691_dtm.tif')
    test_converter.show_crs()

    test_converter.convert_mercader_to_lat_long([(1589865.6318470084, 5389035.655016897,120), (1589889.9895173474, 5389049.741555616,100)])
    test_converter.convert_into_correct_form()
    test_converter.create_csv()
