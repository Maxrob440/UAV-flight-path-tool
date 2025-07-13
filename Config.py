import json
import os

class Config:
    '''
    Singleton class to manage configuration settings.
    This class loads configuration settings from a JSON file and provides methods to get, set, and save these settings.
    '''
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(Config, cls).__new__(cls)
        return cls._instance
    
    def __init__(self, config_path='config.json'):
        self.config_path = config_path
        self.config = self.load_config()

    def load_config(self)-> dict:
        '''
        Load configuration settings from a JSON file.
        If the file does not exist, return an empty dictionary.
        '''
        if os.path.exists(self.config_path):
            with open(self.config_path, 'r',encoding='UTF-8') as file:
                
                return json.load(file)
        else:
            self.set_default()
            print("Config file not found. A new one has been created.")
            return self.load_config()

    def save_config(self):
        '''
        Saves the current configuration settings to a JSON file.
        '''
        with open(self.config_path, 'w',encoding='UTF-8') as file:
            json.dump(self.config, file, indent=4)



    def set_default(self):
        '''
        Sets every part of the config file to a default value
        '''
        default_config = {
                "distances":{
                    "buffer_m": {
                        "type":"scale",
                        "default_value": "-20",
                        "min_value": "-100",
                        "max_value":"0",
                    },

                    "height_above_ground_m": {
                        "type":"scale",
                        "default_value":"25",
                        "min_value":"0",
                        "max_value":"100"
                    },
                    "transect_length_m": {
                        "type":"scale",
                        "default_value":"25",
                        "min_value":"0",
                        "max_value":"100"
                    },
                    "interpolation_distance_m": {
                        "type":"scale",
                        "default_value":"10",
                        "min_value":"0",
                        "max_value":"100"
                    },
                   
                    "human_height_above_ground_m":{
                        "type":"scale",
                        "default_value":"1.8",
                        "min_value":"0",
                        "max_value":"10"
                    },
                    "voxel_size_m":"10",
            },
            "random_point_generation":{
                "number_of_points_per_area": {
                        "type":"combobox",
                        "default_value":"8",
                        "values":[8,16]
                    },
                "distance_to_nearest_point_m": {
                    "type":"scale",
                    "default_value": "40",
                    "min_value": "0",
                    "max_value":"100",
                    },
                "attempts_to_find_point_in_area": {
                    'type':'hidden',
                    'default_value':1000
                    }
            },
            "hexagonal_grid_generation":{
                    "grid_size_m":{
                        "type":"scale",
                        "default_value":"100",
                        "min_value":"1",
                        "max_value":"400"
                    }
            },

            "current_map":{
                "folder_location":  "/Users/maxrobertson/Documents/GitHub/UAV_flight_path/Data/MGAT_01201/Base_Data",
                "human_location":""
            },
            "speed_related":{
                "A*_grid_size_m": {
                    "type":"scale",
                    "default_value": "40",
                    "min_value": "5",
                    "max_value": "200"
                },
                # "A*_grid_growth_multiplier": "10",
                "DVLOS_m": {
                    "type":"scale",
                    "default_value": "1",
                    "min_value": "0",
                    "max_value": "4"
                },
                "DVLOS_interpolation_m": {
                  "type": "scale",
                    "default_value": "10",
                    "min_value": "1",
                    "max_value": "100"  
                },
                "brute_force_cutoff":{
                    "type":"hidden",
                    "default_value":8
                },
                "maximum_recusive_depth": {
                    'type':'hidden',
                    'default_value':"3"
                    }
                },
            "io":{
                "output_folder": "OUTPUT",
                "graph_picture_name": "graph.png",
                "input_folder": "Data",
                "specific_folder_name": "Examples",
                "output_CSV_file_name": "PATH",
                "output_transect_file_name":"transects"

                },
            'point_creation':{
                'random_point_generation':{
                    'type':'checkbutton',
                    'default_value':False
                },
                'flyable_areas':{
                    'type':'checkbutton',
                    'default_value':False}
            },
            "ant_colony":{
                "number_of_ants": {
                    'type':'hidden',
                    'default_value':10
                },
                "number_of_iterations": {
                    'type':'hidden',
                    'default_value':100
                },
                "pheromone_evaporation_rate": {
                    'type':'hidden',
                    'default_value':0.5
                },
                "pheromone_deposit_amount": {
                    'type':'hidden',
                    'default_value': 1.0
                },
                "pheromone_weight": {
                    'type':'hidden',
                    'default_value':1
                },
                "distance_weight": {
                    'type':'hidden',
                    'default_value':1
                },
                "visibility_weight": {
                    'type':'hidden',
                    'default_value':1
                }
            },
            "clustering":{
                "clustering_method": {
                    'type':'combobox',
                    'default_value':"TSP",
                    'values' : ['TSP']
                },
                "points_per_cluster":{
                    'type':'scale',
                    'default_value':'8',
                    'min_value':'1',
                    'max_value':'20'
                }
            }
        }
        self.config=default_config
        self.save_config()
        self.load_config()
    
    def update_nested(self, keys, value):
        '''
        Updates a nested config key given a list of keys and a value.
        '''
        keys.append('value')
        d = self.config
        for key in keys[:-1]:
            d = d.setdefault(key, {})
        try:
            d[keys[-1]] = value
        except TypeError:
            keys.pop()
            d= self.config
            for key in keys[:-1]:
                d = d.setdefault(key, {})
            d[keys[-1]] = value

        
        self.save_config()
    
    def get_nested(self,key1,key2):
        value = self.config[key1][key2]
        if isinstance(value,dict):
            try:
                return self.config[key1][key2]['value']
            except KeyError:
                return self.config[key1][key2]['default_value']
        elif isinstance(value,(str,int)):
            return value

if __name__== "__main__":
    configer = Config()
    # print(configer.load_config())
    configer.set_default()
    configer.update_nested(['distances', 'buffer_m'], '-50')
    print(configer.config)
