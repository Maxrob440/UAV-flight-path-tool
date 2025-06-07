from tkinter import *
from tkinter import ttk
from tkinter.filedialog import askdirectory
import os
import tkinter as tk
from Driver import Driver
import matplotlib.pyplot as plt
from PIL import Image, ImageTk
from Config import Config
from saveoutput import Converter

import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend for testing




class Gui:
    '''
    This class creates a GUI for the program. It allows the user to select a folder, load a shapefile, generate a buffer, and solve the TSP.'''
    def __init__(self):
        self.config = Config()
        self.config.set_default()
        self.folder_location = self.config.config['current_map']['folder_location']
        self.driver = None
        self.root = Tk()

        self.default_values_tk = {}
        self.frm = ttk.Frame(self.root, padding=10)
        self.frm.grid()

        self.terminal = ttk.Label(self.frm, text="[Terminal]")
    
    

    def check_necessities(self, method_name):
        necessities = {
            'select_folder': [],
            'load_shapefile': ['folder_location'],
            'add_to_terminal': [],
            'generate_picture': ['driver'],
            'save_output': ['driver'],
            'load_next_buffer': ['driver'],
            'load_next_standing_location': ['driver'],
            'generate_points': ['driver'],
            'generate_transects': ['driver',('driver','cities')],
            'solve_tsp': ['driver',('driver','cities')],
            'solve_transects': ['driver',('driver','transects')],
            'view_threed': ['driver'],
        }
    
        missing = []
        for key in necessities.get(method_name, []):
            if isinstance(key, tuple):
                obj= key[0]
                attr = key[1]
                value = getattr(self, obj, None)
                attr_value = getattr(value, attr, None)
                if attr_value is None or attr_value == []:
                    missing.append(attr)
                    break
                

            else:
                value = getattr(self, key, None)
                if value in [None, '']:
                    missing.append(key)    
                    break
        if missing:
            self.log_error(missing[0])
            # self.add_to_terminal(f"Missing {missing} for {method_name}")
            raise ValueError(f"Missing {missing} for {method_name}")
    
        return True

    def log_error(self, error):
        '''
        Logs the error to the terminal
        '''
        errors={
            'driver':'101: Driver not loaded',
            'folder_location':'102: Folder location not set',
            'cities':'104: Cities not generated',
            'transects':'103: Transects not generated',
            'no path':'105: No path generated',
        }
        if error in errors:
            self.add_to_terminal(errors[error])


    def select_folder(self):
        '''
        Function to select a folder using a file dialog. The selected folder is stored in the config file.
        '''
        self.check_necessities('select_folder')
        self.folder_location = askdirectory(initialdir="/Users/maxrobertson/Documents/")
        self.config.config['current_map']['folder_location'] = self.folder_location
        self.add_to_terminal("Folder selected")
        
        self.config.save_config()

    def load_shapefile(self,folder_location=None):
        '''
        Function to load a shapefile. It creates a Driver object and loads the shapefile. It also generates a buffer and a picture of the buffer.
        Expensive to run
        '''
        if folder_location is not None:
            self.folder_location = folder_location

        number_of_points = int(self.config.config['distances']['number_of_points_per_area'])
        if self.folder_location == '':
            if self.config.config['current_map']['folder_location'] == '':
                self.add_to_terminal("Invalid folder selected, please try again")
                self.add_to_terminal("Ensure folder contains one .tif file and one .shp file ")

                return
            else:
                self.folder_location = self.config.config['current_map']['folder_location']
                self.add_to_terminal("Loading shapefile, from previous load")
        self.check_necessities('load_shapefile')
        self.add_to_terminal("Loading shapefile")
        self.driver = Driver(self.folder_location,number_of_points)
        self.driver.buffer_coords=[]
        try:
            self.driver.load_shp_file()
        except (FileNotFoundError,ValueError):
            self.add_to_terminal("Shapefile not found, please try again")
            return
        self.driver.clean_buffers(len(self.driver.buffer_coords))
        
        self.generate_picture(cities=False)
        self.add_to_terminal("Shapefile loaded")
  
    def add_to_terminal(self, text):
        '''
        Adds text to the terminal label
        '''
        self.check_necessities('add_to_terminal')
        number_of_lines = int(self.terminal['text'].count('\n'))
        if number_of_lines > 8:
            self.terminal['text'] = self.terminal['text'].split('\n', 1)[1]

        # self.terminal['text'] += '\n' + text
        self.terminal['text'] = text
        self.terminal.update()

    def generate_picture(self,
                         buffer=True,
                         area=True,
                         standing_location=True,
                         cities=True,
                         path=False,
                         transects=False,
                         transect_path=False,
                         transect_all_points=False,
                         test=False):
        '''
        Generates a picture depending on the parameters given. It uses matplotlib to plot the points and lines.
        Saves the picture in the Output folder
        '''
        plt.clf()
        if buffer:
            buffer_points = self.driver.buffer_coords[self.driver.current_buffer]
            buffer_x = [coord[0] for coord in buffer_points]
            buffer_y = [coord[1] for coord in buffer_points]
            plt.plot(buffer_x, buffer_y, 'r-', label='Buffer')
        if area:
            for ind,point in enumerate(self.driver.area_coords):
                area_x = [coord[0] for coord in point]
                area_y = [coord[1] for coord in point]
                if ind == 0:
                    plt.plot(area_x, area_y, 'g-',label='Area')
                else:
                    plt.plot(area_x, area_y, 'g-')
        if cities:
            point_x = [coord[0] for coord in self.driver.cities]
            point_y = [coord[1] for coord in self.driver.cities]
            plt.scatter(point_x, point_y, marker='o', color='black', label='Cities',s=10)
        if path:
            path_x = [coord[0] for coord in self.driver.best_path_coords]
            path_y = [coord[1] for coord in self.driver.best_path_coords]
            plt.plot(path_x, path_y, 'b-',label='Best Path')
        if standing_location and self.driver.standing_locations:
            standing_x= [self.driver.standing_locations[self.driver.current_standing_id][0]]
            standing_y=[self.driver.standing_locations[self.driver.current_standing_id][1]]
            plt.scatter(standing_x,standing_y, color = 'blue',label='Human Location')

        if transects:
            for point in self.driver.best_path_coords:
                if len(self.driver.transects[point]) ==1: pass
                else:
                    transect = self.driver.transects[point]
                    transect_x = [x[0] for x in transect if x[0] is not None and x[1] is not None]
                    transect_y = [x[1] for x in transect if x[0] is not None and x[1] is not None]
                    # transect_x = transect[0][0], transect[1][0], transect[2][0]
                    # transect_y = transect[0][1], transect[1][1], transect[2][1]
                    plt.plot(transect_x, transect_y, 'orange')

        if transect_path:

            transect_path_x = [coord[0][0] for coord in self.driver.transect_path]
            transect_path_y = [coord[0][1] for coord in self.driver.transect_path]
            if transect_all_points:
                plt.plot(transect_path_x, transect_path_y, 'purple', label='Transect Path',marker='o')
            else:
                plt.plot(transect_path_x, transect_path_y, 'purple', label='Transect Path')
        
        output_path = self.config.config['io']['output_folder']
        graph_name = self.config.config['io']['graph_picture_name']
        if not os.path.exists(output_path):
            os.mkdir(output_path)
        plt.axis('off')
        plt.axis('equal')
        plt.legend(fontsize='12',
                loc='best')
        plt.savefig(os.path.join(output_path, graph_name))
        if not test:
            plt.close()
        
        self.update_image()
        if test:
            return plt.gcf()

    def save_output(self):
        '''
        Saves the output to a CSV file\n
        This is compataible with Flylichi\n
        Requires implementation of camera and speed operations
        '''
        self.check_necessities('save_output')
        if not self.driver.transect_path or not self.driver.best_path_coords:
            self.log_error('no path')
            return
        seen_interpolated = []
        
        height = float(self.config.config['distances']['height_above_ground_m'])
        if self.driver.transect_path:
            threed_coords = self.driver.pointcloudholder.interpolate_route(self.driver.transect_path)
            # threed_coords = [[x,y,self.driver.pointcloudholder.find_altitude((x,y),height),plot] for (x,y),plot in self.driver.transect_path]
        else:
            if not isinstance(self.driver.best_path_coords[0][1],bool):
                self.driver.best_path_coords = [[x,False]for x in self.driver.best_path_coords]
            threed_coords = self.driver.pointcloudholder.interpolate_route(self.driver.best_path_coords)

            # threed_coords = [[x,y,self.driver.pointcloudholder.find_altitude((x,y),height),plot] for (x,y),plot in self.driver.best_path_coords]
 
        # for point in threed_coords: #UGLY CODE fixing a bug elsewhere by removing duplicates
        #     if point not in seen_interpolated:
        #         seen_interpolated.append(point)
        saver = Converter(self.driver.folder_path)
        saver.convert_mercader_to_lat_long(threed_coords)
        saver.create_bearings()

        buffer_id = self.driver.current_buffer
        saver.save_all(buffer_id,self.driver.transects,self.driver.folder_path)
        # saver.create_csv(str(buffer_id))
        self.add_to_terminal("Output saved")

    def load_next_buffer(self):
        '''
        Cycles through all the buffers and regenerates the picture\n
        '''
        self.check_necessities('load_next_buffer')

        self.driver.current_buffer =(self.driver.current_buffer+ 1)%len(self.driver.buffer_coords)
        self.add_to_terminal("Buffer loaded")
        if len(self.driver.buffer_coords) == 1:
            self.add_to_terminal("Only one buffer present")
        self.generate_picture(cities = False)

    def load_next_standing_location(self):
        '''
        Cycles through all the standing locations and regenerates the picture\n'''
        self.check_necessities('load_next_standing_location')
        if len(self.driver.standing_locations) == 0:
            self.add_to_terminal("No standing locations present")
            return
        self.driver.current_standing_id = (self.driver.current_standing_id + 1) % len(self.driver.standing_locations)
        self.generate_picture(cities = False)

    def generate_points(self):
        '''
        Loads the standing locations and generates points around them with DVLOS\n
        '''
        self.check_necessities('generate_points')
        if self.driver is None:
            self.add_to_terminal("Please load a shapefile first")
            return
        self.driver.number_points = int(self.config.config['distances']['number_of_points_per_area'])

        self.driver.best_path_coords = []
        self.driver.pointcloudholder.dvlos_lines=[]
        self.driver.transect_path = []
        self.driver.transects = []
        self.add_to_terminal("Generating point cloud")
        self.driver.pointcloudholder.read_tif()
        self.add_to_terminal("Point cloud loaded")
        try:
            self.driver.load_standing_locations()
            self.config.config['current_map']['human_location'] = self.driver.standing_locations
            self.config.save_config()
            xystart = (self.driver.standing_locations[self.driver.current_standing_id][0],self.driver.standing_locations[self.driver.current_standing_id][1])
            self.add_to_terminal("Standing location loaded")
        except (ValueError,IndexError) as e: # value and index errors
            print(e)
            xystart=None
            self.add_to_terminal("Error loading standing locations, ensure a .txt file is present")

        self.add_to_terminal("Generating points")
        self.driver.generate_points_standard(xystart)
        self.add_to_terminal("Points generated")
        self.generate_picture(cities=True)

    def generate_transects(self):
        '''
        Generates transects from the best path coordinates\n
        '''
        try:
            self.check_necessities('generate_transects')

            best_path_coords_no_duplicates = [x for i, x in enumerate(self.driver.best_path_coords) if x not in self.driver.best_path_coords[:i]]
            self.driver.create_transects(best_path_coords_no_duplicates)
            self.add_to_terminal("Transects generated")
            self.generate_picture(cities = True,
                                transects = True)
        except ValueError as e:
            self.add_to_terminal("Error in transect generation")
            print(e)

    def solve_tsp(self):
        '''
        Solves the TSP problem and generates pictures\n
        '''
        self.check_necessities('solve_tsp')
        self.driver.transect_path=[]
        self.add_to_terminal("Solving TSP")
        self.driver.solve_tsp()
        self.add_to_terminal("TSP solved")
        seen=[]
        for point in self.driver.best_path_coords: #UGLY CODE fixing a bug elsewhere by removing duplicates
            if point not in seen:
                seen.append(point)
        self.driver.best_path_coords = seen
        self.generate_picture(cities = True,
                              path = True)
    
    def solve_transects(self):
        '''
        Solve the transect route with brute force and generate pictures\n
        '''
        self.check_necessities('solve_transects')
        if len(self.driver.transects) >= 30:
            self.add_to_terminal("Too many transects to solve, please reduce the number of points")
            return        


        self.add_to_terminal("Solving transect route")

        self.driver.solve_transect_route()
        self.add_to_terminal("Transect route solved")
        self.generate_picture(cities=True,
                              transect_path=True,
                              transect_all_points=False)


    def update_image(self):
        '''
        Loads the next image
        '''
        try:
            image_path = self.config.config['io']['graph_picture_name']
            output_path = self.config.config['io']['output_folder']
            img = Image.open(f"{output_path}/{image_path}")
            img = img.resize((400, 400))
            img_tk = ImageTk.PhotoImage(img)
            label_img = ttk.Label(self.frm, image=img_tk)
            label_img.grid(column=2, row=0, rowspan=8)
            label_img.image = img_tk  
        except Exception as e:
            print("Error loading image:", e)

    def view_threed(self):
        '''
        Generates a 3D view of the point cloud and the latest path generated\n
        '''
        self.check_necessities('view_threed')

        if not self.driver.transect_path and not self.driver.best_path_coords:
            self.log_error('no path')
            return

        self.add_to_terminal("Generating 3D view")
        
        if self.driver.standing_locations:
            standing_location = self.driver.standing_locations[self.driver.current_standing_id]
        else:
            standing_location =None
        if self.driver.transect_path:
            self.driver.pointcloudholder.show_point_cloud(
                cities=self.driver.cities,
                human_location=standing_location,
                dvlos = True,
                best_path_coords=self.driver.transect_path,
                buffer_coords=self.driver.buffer_coords[self.driver.current_buffer]) 
            return
        if not isinstance(self.driver.best_path_coords[0][1],bool):
            best_path_coords_bool = [[x,False]for x in self.driver.best_path_coords] #Required to have a boolean after each point to show camera movements 
        else:
            best_path_coords_bool = self.driver.best_path_coords
        self.driver.pointcloudholder.show_point_cloud(
            cities=self.driver.cities,
            human_location=standing_location,
            dvlos = True,
            best_path_coords=best_path_coords_bool,
            buffer_coords=self.driver.buffer_coords[self.driver.current_buffer]
            )
        self.add_to_terminal("3D view generated")


    def create_config_window(self):
        ''' 
        Pop up window to adjust the config file
        '''
        def save_config(config_window):
            '''
            Saves the config file with the new values
            '''

            for key, value in self.config.config.items():
                for key2,value2 in value.items():
                    if key2 in self.default_values_tk:
                        self.config.config[key][key2] = self.default_values_tk[key2].get()
            buffer=float(self.default_values_tk['buffer_m'].get())
            if buffer > 0:
                buffer = -buffer
            self.config.config['distances']['buffer_m'] =buffer
        

            self.config.save_config()
            config_window.destroy()
    
        def reset_config():
            '''
            Defaults the config file to the default values'''
            self.config.set_default()
            config_window.destroy()
            self.create_config_window()

        config_window = Toplevel(self.root)
        config_window.title("Configuration")
        config_window.geometry("400x900")
        
        ttk.Label(config_window, text="Configuration Options").grid(column=0, row=0)
        counter=0
        for key,value in self.config.config.items():
            
            ttk.Label(config_window, text=key).grid(column=0, row=counter+1)
            counter+=1

            for key2, value2 in value.items():
                self.default_values_tk[key2] = tk.StringVar(value = self.config.config[key][key2])
                ttk.Label(config_window, text=f"{key2}:").grid(column=0, row=counter+1)
                ttk.Entry(config_window, textvariable=self.default_values_tk[key2]).grid(column=1, row=counter+1)
                counter+=1

        ttk.Button(config_window, text="Save", command=lambda: save_config(config_window)).grid(column=0, row=counter+2)
        ttk.Button(config_window, text="Reset to Defaults", command=reset_config).grid(column=1, row=counter+2)

    def run_gui(self):
        '''
        Main function to run the GUI\n
        '''
        style = ttk.Style()
        # style.theme_use('default')


        ttk.Label(self.frm, text="Please Select the folder which contains .shp and .tif files:").grid(column=0, row=0)
        self.terminal.grid(column=0, row=1, rowspan=4)
        ttk.Button(self.frm, text="Browse", command=self.select_folder).grid(column=1, row=0)

        ttk.Button(self.frm, text="Generate Buffer", command=self.load_shapefile).grid(column=1, row=1)
        ttk.Button(self.frm,text="Cycle Buffer",command=self.load_next_buffer).grid(column=1, row=2)
        ttk.Button(self.frm,text="Generate points",command=self.generate_points).grid(column=1, row=3)
        ttk.Button(self.frm,text="Cycle Standing Location",command=self.load_next_standing_location).grid(column=1, row=4)
        ttk.Button(self.frm,text="Solve TSP",command=self.solve_tsp).grid(column=1, row=5)
        ttk.Button(self.frm,text="Generate Transects",command=self.generate_transects).grid(column=1, row=6)
        ttk.Button(self.frm,text="Solve Transects",command=self.solve_transects).grid(column=1, row=7)
        ttk.Button(self.frm,text="View 3D",command=self.view_threed).grid(column=1, row=8)
        ttk.Button(self.frm, text="adjust config", command=self.create_config_window).grid(column=0, row=9)

        ttk.Button(self.frm,text="Save output",command=self.save_output).grid(column=1, row=9)

        self.update_image()
        self.root.mainloop()

if __name__ == "__main__":
    gui = Gui()
    gui.run_gui()

