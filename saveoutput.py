from math import atan2,radians, degrees, sin, cos
import os
import numpy as np
import shutil

import rasterio
from pyproj import Transformer
import pandas as pd
from Config import Config
from shp_file_generator import TransectGenerator


class Converter:
    '''
    The converter class takes an input of the tif file
    '''
    def __init__(self,tif_file):
        self.config = Config()
        self.tif_file = tif_file
        self.lat_long_coords = []
        self.bearings = []

    def show_crs(self):
        '''Shows the CRS information'''
        with rasterio.open(self.tif_file) as src:
            print(src.crs)
    
    def convert_mercader_to_lat_long(self,coords):
        '''Converts from the mercader (EPSG:2193) to lat long (EPSG:4326)'''
        inital_z = coords[0][0][2]-20
        for (x,y,z),plot in coords:
            to_latlong = Transformer.from_crs("EPSG:2193", "EPSG:4326", always_xy=True)
            lon, lat = to_latlong.transform(x, y)
            self.lat_long_coords.append((lat, lon,z-inital_z,plot))


    def create_bearings(self):
        ''' 
        This converts the lat long coords of the route into bearings
        The last point will be facing north

        '''
        bearings = []

        def calculate_bearing(lat1,long1,lat2,long2):
            '''Calculates the bearing between two points'''
            lat1,long1,lat2,long2 = map(radians,[lat1,long1,lat2,long2]) #Convert radians
            delta_long = long2 - long1

            x = sin(delta_long) * cos(lat2)
            y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_long)
            bearing = atan2(x,y)
            return (degrees(bearing)+360) % 360
    
        for ind,point in enumerate(self.lat_long_coords):
            if ind == len(self.lat_long_coords)-1:
                break
            lat1,long1,z,plot = point
            lat2,long2,z,plot = self.lat_long_coords[ind+1]
            bearings.append(calculate_bearing(lat1,long1,lat2,long2))

        self.bearings = bearings
        self.bearings.append(0) #Add 0 to the end of the list to make it the same length as lat_long_coords
        return bearings #Small errors here about a degree
    


    def create_csv(self,flight_no):
        '''
        Transforms the route data into a csv file that can be read by flylichi
        if the point is a part of the transets then the speed is 0.8 m/s and the photo interval is 2 seconds
        This is then added and exported
        '''
        df = pd.DataFrame(columns=["latitude","longitude","altitude(m)","heading","curvesize(m)","rotationdir","gimbalmode","gimbalpitchangle","actiontype1","actionparam1","actiontype2","actionparam2","actiontype3","actionparam3","actiontype4","actionparam4","actiontype5","actionparam5","actiontype6","actionparam6","actiontype7","actionparam7","actiontype8","actionparam8","actiontype9","actionparam9","actiontype10","actionparam10","actiontype11","actionparam11","actiontype12","actionparam12","actiontype13","actionparam13","actiontype14","actionparam14","actiontype15","actionparam15","altitudemode","speed(m/s)","poi_latitude","poi_longitude","poi_altitude(m)","poi_altitudemode","photo_timeinterval","photo_distinterval"])
        print(len(self.lat_long_coords))
        for ind,point in enumerate(self.lat_long_coords):
            if point[3] is True:
                speed = 0.8
                photo_timeinterval = 2
                photo_distinterval = 0
            else:
                speed = 0
                photo_timeinterval = -1
                photo_distinterval = -1

            df.loc[ind]=([point[0],point[1],point[2],self.bearings[ind],0.2,0,0,0,5,-90,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,-1,0,0,speed,0,0,0,0,photo_timeinterval,photo_distinterval])
        
        output_folder = self.config.config['io']['output_folder']
        specific_folder = self.config.config['io']['specific_folder_name']
        csv_path = self.config.config['io']['output_CSV_file_name'] + f'{flight_no}.csv'
        output_path = os.path.join(output_folder, specific_folder)
        csv_path = os.path.join(output_path, csv_path)
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        
        
        df.to_csv(csv_path, index=False)

    def save_transects(self,transects,flight_no):
        trans_generator = TransectGenerator(flight_no)
        for transect in transects.values():
            if len(transect) == 1:
                continue
            trans_generator.add_transect(transect)
        trans_generator.save()

    def save_dem_shp(self,folder_path):
        output_folder = self.config.config['io']['output_folder']
        specific_folder = self.config.config['io']['specific_folder_name']

        print(folder_path)

        shp_file_path = [path for path in os.listdir(folder_path) if path.endswith('.shp')][0]
        tif_file_path = [path for path in os.listdir(folder_path) if path.endswith('.tif')][0]
        shp_file_path = os.path.join(folder_path, shp_file_path)
        tif_file_path = os.path.join(folder_path, tif_file_path)
        shutil.copy(shp_file_path, os.path.join(output_folder, specific_folder))
        shutil.copy(tif_file_path, os.path.join(output_folder, specific_folder))


    def save_all(self,flight_no,transects,folder_path):
        self.create_csv(flight_no)
        self.save_transects(transects,flight_no)
        self.save_dem_shp(folder_path)

        
    

