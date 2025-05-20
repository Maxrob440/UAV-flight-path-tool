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
                    "buffer_m": "-30",
                    "distance_to_nearest_point_m": "40",
                    "attempts_to_find_point_in_area": "1000",
                    "height_above_ground_m": "25",
                    "transect_length_m": "25",
                    "interpolation_distance_m": "10",
                    "number_of_points_per_area": "8",
                    "human_height_above_ground_m":"4",
                    "grid_size_m":"100"
            },

            "current_map":{
                "folder_location": "",
                "human_location":""
            },
            "speed_related":{
                "A*_grid_size_m": "40",
                "A*_grid_growth_multiplier": "10",
                "DVLOS_m": "1",
                "brute_force_cutoff": "8",
                "maximum_recusive_depth": "3",
                },
            "io":{
                "output_folder": "OUTPUT",
                "graph_picture_name": "graph.png",
                "input_folder": "Data",
                "output_file_name": "PATH"
                },
            "ant_colony":{
                "number_of_ants": "10",
                "number_of_iterations": "100",
                "pheromone_evaporation_rate": "0.5",
                "pheromone_deposit_amount": "1.0",
                "pheromone_weight": "1.0",
                "distance_weight": "1.0",
                "visibility_weight": "1.0"
            },
        }
        self.config=default_config
        self.save_config()
        self.load_config()


if __name__== "__main__":
    configer = Config()
    # print(configer.load_config())
    configer.set_default()
