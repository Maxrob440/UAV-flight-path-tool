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



class Gui:
    '''
    This class creates a GUI for the program. It allows the user to select a folder, load a shapefile, generate a buffer, and solve the TSP.'''
    def __init__(self):
        self.config = Config()
        self.folder_location = self.config.config['current_map']['folder_location']
        self.driver = None
        self.root = None
        self.default_values_tk = {}
        self.frm = None



    def select_folder(self):
        '''
        Function to select a folder using a file dialog. The selected folder is stored in the config file.
        '''
        self.folder_location = askdirectory(initialdir="/Users/maxrobertson/Documents/")
        self.config.config['current_map']['folder_location'] = self.folder_location
        self.config.save_config()
        print('Folder selected: ' + self.folder_location)

    def load_shapefile(self):
        '''
        Function to load a shapefile. It creates a Driver object and loads the shapefile. It also generates a buffer and a picture of the buffer.
        Expensive to run
        '''
        number_of_points = int(self.config.config['distances']['number_of_points_per_area'])
        if self.folder_location == '':
            if self.config.config['current_map']['folder_location'] == '':
                print("Please select a folder first")
                return
            else:
                self.folder_location = self.config.config['current_map']['folder_location']
        self.driver = Driver(self.folder_location,number_of_points)
        self.driver.buffer_coords=[]
        self.driver.load_shp_file()
        self.driver.clean_buffers(len(self.driver.buffer_coords))
        
        self.generate_picture(cities=False)
  
        

    def generate_picture(self,
                         buffer=True,
                         area=True,
                         standing_location=True,
                         cities=True,
                         path=False,
                         transects=False,
                         transect_path=False,
                         transect_all_points=False):
        '''
        Generates a picture depending on the parameters given. It uses matplotlib to plot the points and lines.
        Saves the picture in the Output folder
        '''
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
                    transect_x = transect[0][0], transect[1][0], transect[2][0]
                    transect_y = transect[0][1], transect[1][1], transect[2][1]
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
        plt.close()
        self.update_image()

    def save_output(self):
        '''
        Saves the output to a CSV file\n
        This is compataible with Flylichi\n
        Requires implementation of camera and speed operations
        '''
        seen_interpolated = []
        
        height = float(self.config.config['distances']['height_above_ground_m'])
        if self.driver.transect_path:
            print(self.driver.transect_path)
            threed_coords = [[x,y,self.driver.pointcloudholder.find_altitude([x,y],height),plot] for (x,y),plot in self.driver.transect_path]
        else:
            threed_coords = [[x,y,self.driver.pointcloudholder.find_altitude([x,y],height),plot] for (x,y),plot in self.driver.best_path_coords]
        for point in threed_coords: #UGLY CODE fixing a bug elsewhere by removing duplicates
            if point not in seen_interpolated:
                seen_interpolated.append(point)

        saver = Converter(self.driver.folder_path)
        saver.convert_mercader_to_lat_long(seen_interpolated)
        saver.create_bearings()
        output_name = self.config.config['io']['output_file_name']
        buffer_id = self.driver.current_buffer
        saver.create_csv(output_name,str(buffer_id))

    def load_next_buffer(self):
        '''
        Cycles through all the buffers and regenerates the picture\n
        '''
        if self.driver is None:
            print("Please load a shapefile first")
            return
        self.driver.current_buffer =(self.driver.current_buffer+ 1)%len(self.driver.buffer_coords)
        self.generate_picture(cities = False)

    def load_next_standing_location(self):
        '''
        Cycles through all the standing locations and regenerates the picture\n'''
        if self.driver is None:
            print("Please load a shapefile first")
            return
        self.driver.current_standing_id = (self.driver.current_standing_id + 1) % len(self.driver.standing_locations)
        self.generate_points()
        self.generate_picture(cities = True)

    def generate_points(self):
        '''
        Loads the standing locations and generates points around them with DVLOS\n
        '''
        if self.driver is None:
            print("Please load a shapefile first")
            return
        self.driver.number_points = int(self.config.config['distances']['number_of_points_per_area'])

        self.driver.best_path_coords = []
        self.driver.pointcloudholder.dvlos_lines=[]
        self.driver.transect_path = []
        self.driver.transects = []
        try:
            self.driver.load_standing_locations()
            self.config.config['current_map']['human_location'] = self.driver.standing_locations
            xystart = (self.driver.standing_locations[self.driver.current_standing_id][0],self.driver.standing_locations[self.driver.current_standing_id][1])
            print('Standing location:', xystart)
        except Exception as e: # value and index errors
            xystart=None
            print('Error loading standing locations:', e)
        self.driver.pointcloudholder.read_tif()
        self.driver.generate_points(xystart)

        self.generate_picture(cities=True)

    def generate_transects(self):
        '''
        Generates transects from the best path coordinates\n
        '''
        if self.driver is None:
            print("Please load a shapefile first")
            return
        if self.driver.best_path_coords == []:
            print("Please generate the path first")
            return

        best_path_coords_no_duplicates = [x for i, x in enumerate(self.driver.best_path_coords) if x not in self.driver.best_path_coords[:i]]
        self.driver.create_transects(best_path_coords_no_duplicates)

        self.generate_picture(cities = True,
                              transects = True)

    def solve_tsp(self):
        '''
        Solves the TSP problem and generates pictures\n
        '''
        self.driver.transect_path=[]
        if self.driver is None:
            print("Please load a shapefile first")
            return
        self.driver.solve_tsp()
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
        if self.driver is None:
            print("Please load a shapefile first")
            return
        self.driver.solve_transect_route()

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
        if self.driver.standing_locations:
            standing_location = self.driver.standing_locations[self.driver.current_standing_id]
        else:
            standing_location =None
        if self.driver.transect_path:
            self.driver.pointcloudholder.show_point_cloud(
                cities=self.driver.cities,
                human_location=standing_location,
                dvlos = True,
                best_path_coords=self.driver.transect_path) 
            return   
        self.driver.pointcloudholder.show_point_cloud(
            cities=self.driver.cities,
            human_location=standing_location,
            dvlos = True,
            best_path_coords=self.driver.best_path_coords)


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
        config_window.geometry("400x750")
        
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
        self.root = Tk()
        self.frm = ttk.Frame(self.root, padding=10)
        self.frm.grid()
        style = ttk.Style()
        # style.theme_use('default')


        ttk.Label(self.frm, text="Please Select the folder which contains .shp and .tif files:").grid(column=0, row=0)
        ttk.Label(self.frm, text="[A box showing the process currently displayed in terminal]").grid(column=0, row=1)
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

