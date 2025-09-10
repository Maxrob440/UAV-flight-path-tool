from tkinter import *
from tkinter import ttk
from tkinter.filedialog import askdirectory, askopenfilename
from tkinter import font
import os
import tkinter as tk
import webbrowser

from PIL import Image, ImageTk
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from Driver import Driver
from Config import Config
from saveoutput import Converter
from widget_factory import WidgetFactory

if os.environ.get("MPLBACKEND", "").lower() != "agg":
    matplotlib.use("TkAgg")

class Gui:
    '''
    This class creates a GUI for the program. 
    It allows the user to select a folder, load a shapefile,
    generate a buffer, and solve the TSP.'''
    def __init__(self):
        self.config = Config()
        self.config.set_default()
        self.driver = None
        self.root = Tk()

        self.default_values_tk = {}
        self.frm = ttk.Frame(self.root, padding=10)
        self.frm.grid()

        self.terminal = ttk.Label(self.frm, text="[Terminal]")

    def get_help(self):
        '''
        Launches web browser and loads the github wiki help page
        '''
        webbrowser.open_new('https://github.com/Maxrob440/UAV_flight_path/blob/main/Rewrite/Docs/Config.md')


    def check_necessities(self, method_name:str)->bool:
        '''
        Checks that the necessary attributes are present for the method to run,
        if not will add a message to the terminal with instructions
        '''
        necessities = {
            'select_folder': [],
            'load_shapefile': [],
            'add_to_terminal': [],
            'generate_picture': ['driver'],
            'save_output': ['driver'],
            'load_next_buffer': ['driver'],
            'load_next_standing_location': ['driver'],
            'generate_points': ['driver'],
            'generate_transects': ['driver',('driver','cities'),('driver','clustered')],
            'solve_tsp': ['driver',('driver','cities')],
            'solve_transects': ['driver',('driver','transects')],
            'view_threed': ['driver',('driver','cities')],
            'create_clusters': ['driver',('driver','cities')],
            'cycle_cluster': ['driver',('driver','clustered')],
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
            'clustered':'106: Clusters not generated',
        }
        if error in errors:
            self.add_to_terminal(errors[error])


    def select_folder(self):
        '''
        Function to select a folder using a file dialog.
        The selected folder is stored in the config file.
        '''
        self.check_necessities('select_folder')
        path = askdirectory()
        print(path)
        self.config.update_nested(['current_map','folder_location'],
                                  path)
        
        self.add_to_terminal("Folder selected")

        # self.config.save_config()

    def load_shapefile(self,folder_location=None):
        '''
        Function to load a shapefile. It creates a Driver object and loads the shapefile. 
        It also generates a buffer and a picture of the buffer.
        Expensive to run
        '''
        if folder_location is None:
            folder_location = self.config.get_nested('current_map','folder_location')

        if folder_location == '':
            self.add_to_terminal("Invalid folder selected, please try again")
            return

        self.check_necessities('load_shapefile')
        self.add_to_terminal("Loading shapefile")

        self.driver = Driver()
        self.driver.buffer_coords=[]
        try:
            self.driver.load_shp_file(folder_location)
        except (FileNotFoundError,ValueError) as e:
            print(e)
            self.add_to_terminal("Shapefile not found, please try again")
            self.add_to_terminal(e)
            return
        self.driver.clean_buffers(len(self.driver.buffer_coords))
        self.driver.pointcloudholder.read_tif()
        self.add_to_terminal("Point cloud loaded")
       
        self.generate_picture(cities=False)
        self.add_to_terminal("Shapefile loaded")
    
    def load_standing_locations(self):
        path = askopenfilename(
        title="Select a file",
        # initialdir="~",                        # or Path.home()
        filetypes=[("Text files", "*.txt"),
                   ("Shapefiles", "*.shp"),
                   ("All files", "*.*"),
                   ],
    )

        
        try:
            self.driver.load_standing_locations(path=path)
            self.config.config['current_map']['human_location'] = self.driver.standing_locations
            self.config.save_config()
            
            self.add_to_terminal("Standing location loaded")
        except (ValueError,IndexError) as e: # value and index errors
            print(e)
            xystart=None
            self.add_to_terminal("Error loading standing locations, ensure a .txt file is present")
        self.generate_picture(cities=False)
        self.add_to_terminal("Standing locations loaded")
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
                         clusters=False,
                         path=False,
                         transects=False,
                         transect_path=False,
                         transect_all_points=False,
                         test=False,
                         flyable = True):
        '''
        Generates a picture depending on the parameters given. 
        It uses matplotlib to plot the points and lines.
        Saves the picture in the Output folder
        '''
        plt.clf()

        flyable = self.config.get_nested('point_creation','flyable_areas')
        if buffer is False and flyable is False:
            buffer = False
        else:
            buffer = not flyable
        if clusters:
            cluster_points = self.driver.clustered
            for ind,cluster in enumerate(cluster_points):

                opacity = 1
                if ind != self.driver.current_cluster:
                    opacity = 0.4
                cluster_x = [coord[0] for coord in cluster if coord in self.driver.cities]
                cluster_y = [coord[1] for coord in cluster if coord in self.driver.cities]
                plt.scatter(cluster_x,cluster_y,alpha=opacity)
        if buffer:
            buffer_points = self.driver.buffer_coords[self.driver.current_buffer]
            buffer_x = [coord[0] for coord in buffer_points]
            buffer_y = [coord[1] for coord in buffer_points]
            plt.plot(buffer_x, buffer_y, 'r-', label='Buffer')
        if flyable:
            flyable = self.driver.flyable_area[self.driver.current_flyable_area]
            print(flyable)
            flyable_x = [coord[0] for coord in flyable]
            flyable_y = [coord[1] for coord in flyable]
            plt.plot(flyable_x, flyable_y, 'cyan', label='Flyable Area')
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
            plt.scatter(standing_x,standing_y, marker = 'x',color = 'blue',label='Human Location')

        if transects:
            for _,transect in self.driver.transects[self.driver.current_cluster].items():
                if len(transect) == 1:
                    continue

                transect_x = [x[0] for x in transect if x[0] is not None and x[1] is not None]
                transect_y = [x[1] for x in transect if x[0] is not None and x[1] is not None]

                plt.plot(transect_x, transect_y, 'orange')

        if transect_path:
            transect_path_x = [coord[0][0] for coord in self.driver.transect_path]
            transect_path_y = [coord[0][1] for coord in self.driver.transect_path]

            if transect_all_points:
                plt.plot(transect_path_x, 
                         transect_path_y, 
                         'purple', 
                         label='Transect Path',
                         marker='o')
            else:
                plt.plot(transect_path_x, 
                         transect_path_y, 
                         'purple', 
                         label='Transect Path')

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
        pointcloud = self.driver.pointcloudholder
        self.check_necessities('save_output')
        if not self.driver.transect_path or not self.driver.best_path_coords:
            self.log_error('no path')
            return

        if self.driver.transect_path:
            threed_coords = pointcloud.interpolate_route(self.driver.transect_path)
        else:
            if not isinstance(self.driver.best_path_coords[0][1],bool):
                self.driver.best_path_coords = [[x,False]for x in self.driver.best_path_coords]
            threed_coords = pointcloud.interpolate_route(self.driver.best_path_coords)
        folder_path = self.config.get_nested('current_map','folder_location')
        possible_files = [file for file in os.listdir(folder_path) if file.endswith('.tif')]
        tif_path = possible_files[0]
        tif_path = os.path.join(folder_path, tif_path)
        saver = Converter(tif_path)
        saver.convert_mercader_to_lat_long(threed_coords)
        saver.create_bearings()

        buffer_id = self.driver.current_buffer
        saver.save_all(buffer_id,self.driver.transects,folder_path,self.driver.current_cluster)
        # saver.create_csv(str(buffer_id))
        self.add_to_terminal("Output saved")

    def load_next_buffer(self):
        '''
        Cycles through all the buffers and regenerates the picture\n
        '''
        self.check_necessities('load_next_buffer')

        self.driver.cycle_buffer()
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
        possible_standing_locations= len(self.driver.standing_locations)
        next_standing_id = (self.driver.current_standing_id+1) % possible_standing_locations
        self.driver.current_standing_id = next_standing_id
        self.generate_picture(cities = False)

    def load_points(self):
        self.driver.best_path_coords = []
        self.driver.pointcloudholder.dvlos_lines=[]
        self.driver.transect_path = []
        self.driver.transects = []
        path = askopenfilename(
        title="Select a file",
        filetypes=[("Text files", "*.txt"),
                   ("Shapefiles", "*.shp"),
                   ("All files", "*.*"),
                   ],
    )
        self.driver.load_points(path=path)
        self.add_to_terminal("Points loaded")
        self.generate_picture(cities=True)
    def generate_points(self):
        '''
        Loads the standing locations and generates points around them with DVLOS\n
        '''
        self.check_necessities('generate_points')

        self.driver.best_path_coords = []
        self.driver.pointcloudholder.dvlos_lines=[]
        self.driver.transect_path = []
        self.driver.transects = []

        self.add_to_terminal("Generating point cloud")
        xystart = None
        if self.driver.standing_locations:
            xystart = (self.driver.standing_locations[self.driver.current_standing_id][0],
                       self.driver.standing_locations[self.driver.current_standing_id][1])

        self.add_to_terminal("Generating points")
        self.driver.grid_distances = None
        if self.config.get_nested('point_creation','random_point_generation'):
            self.driver.generate_points_random(xystart)
        else:
            self.driver.generate_points_standard(xystart)
        self.add_to_terminal("Points generated")
        self.generate_picture(cities=True)
  

    def generate_transects(self):
        '''
        Generates transects from the best path coordinates\n
        '''
        try:
            self.check_necessities('generate_transects')

            best_path_coords_no_duplicates = [x for i, x in enumerate(self.driver.best_path_coords) 
                                              if x not in self.driver.best_path_coords[:i]]
            self.driver.best_path_coords = best_path_coords_no_duplicates
            self.driver.create_transects()
            self.add_to_terminal("Transects generated")
            self.generate_picture(cities = False,
                                transects = True,
                                clusters=True)
        except ValueError as e:
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
        for point in self.driver.best_path_coords: 
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
        self.add_to_terminal("Solving transect route")

        self.driver.solve_transect_route()
        self.add_to_terminal("Transect route solved")
        self.generate_picture(cities=False,
                              transect_path=True,
                              transect_all_points=False,
                              clusters=True)

    def create_clusters(self):
        '''
        Forms clusters for the cities based on the method selected in config
        '''
        self.check_necessities('create_clusters')
        self.driver.cluster_points()
        self.generate_picture(cities = False,
                              clusters=True)
        number_of_clusters = len(self.driver.clustered)
        self.add_to_terminal(f'{number_of_clusters} Clusters generated')

    def cycle_cluster(self):
        self.check_necessities('cycle_cluster')
        self.driver.cycle_cluster()
        self.generate_picture(cities = False,
                              clusters = True)
        self.add_to_terminal('Next cluster loaded')

    def update_image(self):
        '''
        Loads the next image
        '''
        try:
            image_path = self.config.config['io']['graph_picture_name']
            output_path = self.config.config['io']['output_folder']
            img = Image.open(f"{output_path}/{image_path}")
            img = img.resize((400, 400))
            self.img_tk = ImageTk.PhotoImage(img)
            label_img = ttk.Label(self.frm, image=self.img_tk)
            label_img.grid(column=2, row=0, rowspan=13)
            # label_img.image = self.img_tk  
        except Exception as e:
            print("Error loading image:", e)
            print('continuing without loading image, likely due to first installation')
            self.add_to_terminal(f"Error loading image, continuing without image,{e}")

    def view_threed(self):
        '''
        Generates a 3D view of the point cloud and the latest path generated\n
        '''
        self.check_necessities('view_threed')

        flyable = self.config.get_nested('point_creation','flyable_areas')
        if flyable:
            boundary = self.driver.flyable_area[self.driver.current_flyable_area]
        else:
            boundary = self.driver.buffer_coords[self.driver.current_buffer]
        
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
                buffer_coords=boundary) 
            return
        if not isinstance(self.driver.best_path_coords[0][1],bool):
            #Required to have a boolean after each point to show camera movements 
            best_path_coords_bool = [[x,False] for x in self.driver.best_path_coords] 
        else:
            best_path_coords_bool = self.driver.best_path_coords
        self.driver.pointcloudholder.show_point_cloud(
            cities=self.driver.cities,
            human_location=standing_location,
            dvlos = True,
            best_path_coords=best_path_coords_bool,
            buffer_coords=boundary
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
                for key2,_ in value.items():
                    if key2 in self.default_values_tk:
                        if isinstance(self.config.config[key][key2],dict):
                            self.config.update_nested([key,key2,'value'],self.default_values_tk[key2].get())
                            self.config.config[key][key2]['value'] = self.default_values_tk[key2].get()
                        else:
                            self.config.config[key][key2] = self.default_values_tk[key2].get()

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
        config_window.geometry("1400x400")

        widget_factory = WidgetFactory(config_window)
        ttk.Label(config_window, text="Configuration Options").grid(column=0, row=0)
        counter=0
        bold_font = font.Font(weight='bold')
        column=0
        for key,value in self.config.config.items():
            if counter>=8:
                column+=2
                counter=1
            counter+=1

            ttk.Label(config_window, text=key,font=bold_font).grid(column=column, row=counter)
            try:
                
                for key2, value2 in value.items():
                    if isinstance(value2,dict):
                        print(value2)
                        if value2['type'] == 'hidden':
                            continue
                        if value2['type'] == 'checkbutton':
                            if self.config.get_nested(key,key2) is False:
                                val = False
                            else:
                                val = True
                            self.default_values_tk[key2] = tk.BooleanVar(value = val)
                        else:
                            self.default_values_tk[key2] = tk.StringVar(value = self.config.get_nested(key,key2))

                        entry_type= value2.get('type', 'entry')
                        default_value = value2.get('default_value', '')
                        min_value = value2.get('min_value', None)
                        max_value = value2.get('max_value', None)
                        posssible_values = value2.get('values',None)
                        widget_factory.create_widget(entry_type,{'from': min_value,
                                                                'to': max_value,
                                                                'orient': 'horizontal',
                                                                'variable' :self.default_values_tk[key2],
                                                                'values':posssible_values}).grid(column=column+1, row=counter+1,sticky='w')
                    elif isinstance(value2,(int,str,float)):
                        self.default_values_tk[key2] = tk.StringVar(value = self.config.config[key][key2])
                        ttk.Entry(config_window, textvariable=self.default_values_tk[key2]).grid(column=column+1, row=counter+1,sticky='w')
                    ttk.Label(config_window, text=f"{key2}:").grid(column=column, row=counter+1)
                    counter+=1
            except Exception as e:
                print(f"Error creating config window for {key}: {e}")
                self.config.set_default()
                config_window.destroy()
                self.create_config_window()
        ttk.Button(config_window, text="Save", command=lambda: save_config(config_window)).grid(column=column, row=counter+2)
        ttk.Button(config_window, text="Reset to Defaults", command=reset_config).grid(column=column+1, row=counter+2)


    def select_point_on_map(self):
        """
        Opens a modal window with an interactive Matplotlib plot of the current map.
        Clicking on the map drops a point, stores it as a standing location,
        refreshes the main image, and closes the picker (you can change that behavior).
        """
        if not hasattr(self, "driver") or self.driver is None:
            self.add_to_terminal("101: Driver not loaded")
            return
        if not getattr(self.driver, "area_coords", None):
            self.add_to_terminal("Load shapefile first")
            return

        win = tk.Toplevel(self.root)
        win.title("Click to select a point")
        win.geometry("650x650")
        win.transient(self.root)
        win.grab_set()  # modal

        fig, ax = plt.subplots(figsize=(6, 6))
        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Flyable/buffer
        flyable_flag = self.config.get_nested('point_creation','flyable_areas')
        try:
            if flyable_flag and getattr(self.driver, "flyable_area", None):
                flyable = self.driver.flyable_area[self.driver.current_flyable_area]
                fx = [c[0] for c in flyable]; fy = [c[1] for c in flyable]
                ax.plot(fx, fy, linewidth=1, label="Flyable Area")
            elif getattr(self.driver, "buffer_coords", None):
                buf = self.driver.buffer_coords[self.driver.current_buffer]
                bx = [c[0] for c in buf]; by = [c[1] for c in buf]
                ax.plot(bx, by, linewidth=1, label="Buffer")
        except Exception:
            pass

        # Area boundary/polygons
        for idx, ring in enumerate(self.driver.area_coords):
            ax.plot([c[0] for c in ring], [c[1] for c in ring], 'g-', linewidth=1, label="Area" if idx == 0 else None)

        if getattr(self.driver, "cities", None):
            ax.scatter([c[0] for c in self.driver.cities], [c[1] for c in self.driver.cities], s=10, label="Cities")

        if getattr(self.driver, "standing_locations", None) and self.driver.standing_locations:
            sx, sy = self.driver.standing_locations[self.driver.current_standing_id]
            ax.scatter([sx], [sy], marker='x', s=60, label="Human Location")

        ax.set_aspect('equal', adjustable='datalim')
        ax.set_title("Click anywhere to select a point")
        ax.axis('off')
        ax.legend(loc="best", fontsize=8)
        fig.tight_layout()
        canvas.draw()

        def on_click(event):
            if event.inaxes is None or event.xdata is None or event.ydata is None:
                return
            x, y = float(event.xdata), float(event.ydata)

            ax.plot(x, y, 'rx', markersize=8)
            canvas.draw()

            if not hasattr(self.driver, "standing_locations") or self.driver.standing_locations is None:
                self.driver.standing_locations = []
            height=float(self.config.get_nested('distances','human_height_above_ground_m'))
            self.driver.standing_locations.append((x, y,self.driver.pointcloudholder.find_altitude((x,y),height)))
            self.driver.current_standing_id = len(self.driver.standing_locations) - 1

            self.config.config['current_map']['human_location'] = self.driver.standing_locations
            try:
                self.config.save_config()
            except Exception:
                pass

            self.generate_picture(cities=False)
            self.add_to_terminal(f"Standing location added: ({x:.3f}, {y:.3f})")
            win.destroy()

        cid = fig.canvas.mpl_connect("button_press_event", on_click)

        # Optional cancel button
        btn_frame = ttk.Frame(win)
        btn_frame.pack(fill=tk.X, padx=8, pady=6)
        ttk.Button(btn_frame, text="Cancel", command=win.destroy).pack(side=tk.RIGHT)

    def run_gui(self):
        '''
        Main function to run the GUI\n
        '''
        # style = ttk.Style()
        # style.theme_use('default')


        ttk.Label(self.frm, text="Created by Max Robertson").grid(column=0, row=0)
        ttk.Label(self.frm,text="To begin please select the folder that contains the boundary.shp and DTM.tif files").grid(column=0, row=1)
       
        self.terminal.grid(column=0, row=6, rowspan=2)
        ttk.Button(self.frm, text='Help',command = self.get_help).grid(column=0, row=8)
        ttk.Button(self.frm, text="Browse", command=self.select_folder).grid(column=1, row=0)

        ttk.Button(self.frm, text="Generate Buffer", command=self.load_shapefile).grid(column=1, row=1)
        ttk.Button(self.frm,text="Cycle Buffer",command=self.load_next_buffer).grid(column=1, row=2)
        ttk.Button(self.frm, text="Select Operator location on map", command=self.select_point_on_map).grid(column=0, row=3)
        ttk.Button(self.frm,text="Select Operator Locations",command=self.load_standing_locations).grid(column=1, row=3)
        ttk.Button(self.frm,text="Cycle Operator Location",command=self.load_next_standing_location).grid(column=1, row=4)
        ttk.Button(self.frm, text = "Load points", command=self.load_points).grid(column=0,row=5)
        ttk.Button(self.frm,text="Generate points",command=self.generate_points).grid(column=1, row=5)
        ttk.Button(self.frm,text="Solve TSP",command=self.solve_tsp).grid(column=1, row=6)
        ttk.Button(self.frm,text="Form Clusters",command = self.create_clusters).grid(column=1,row=7)
        ttk.Button(self.frm,text="Cycle Cluster",command = self.cycle_cluster).grid(column=1,row=8)
        ttk.Button(self.frm,text="Generate Transects",command=self.generate_transects).grid(column=1, row=9)
        ttk.Button(self.frm,text="Solve Transects",command=self.solve_transects).grid(column=1, row=10)
        ttk.Button(self.frm,text="View 3D",command=self.view_threed).grid(column=1, row=11)
        ttk.Button(self.frm, text="adjust config", command=self.create_config_window).grid(column=0, row=11)

        ttk.Button(self.frm,text="Save output",command=self.save_output).grid(column=1, row=13)

        self.update_image()
        self.root.title("UAV Flight Path Planner")
        self.root.mainloop()

if __name__ == "__main__":
    gui = Gui()
    gui.run_gui()
    # gui.create_config_window()
