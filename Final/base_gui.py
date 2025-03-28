from tkinter import *
from tkinter import ttk
from tkinter.filedialog import askopenfilename
from PIL import Image, ImageTk
import run  

# Global variables
dem_location = None
shapefile_location = None

# Create root window
root = Tk()
frm = ttk.Frame(root, padding=10)
frm.grid()

# StringVar to store input values
no_points_var = StringVar()

def select_dem_file():
    global dem_location  
    file_path = askopenfilename(filetypes=[("TIFF Files", "*.tif")])
    if file_path:
        dem_location = file_path
        print("Selected DEM File:", dem_location)  

def select_shapefile():
    global shapefile_location  
    file_path = askopenfilename(filetypes=[("Shapefiles", "*.shp")])
    if file_path:
        shapefile_location = file_path
        print("Selected Shapefile:", shapefile_location)  

def run_program(dimension):
    global dem_location, shapefile_location, no_points_var  
    no_points = clicked.get()
    
    try:
        no_points = int(no_points)  # Convert to integer
        print("Number of points to generate:", no_points)
    except ValueError:
        print("Invalid number of points entered.")
        return  # Exit function if invalid input

    print("Running program with DEM:", dem_location, "and Shapefile:", shapefile_location)
    main = run.Main()
    main.dem_location = dem_location
    main.shp_location = shapefile_location
    main.no_points = no_points  # Store number of points
    main.load_shp_file()
    if dimension == 2:
        main.visualize(twod=True, threed=False)
    elif dimension == 3:
        main.visualize(twod=False, threed=True)
    elif dimension is None:
        main.visualize(twod=False, threed=False)
    print('Finished running program')

def drop_down_options():
    label.config(text = clicked.get())
options = [
    8,
    16
]
clicked = StringVar()
clicked.set(options[0]) # default value
label = ttk.Label(frm, textvariable=clicked)
drop = OptionMenu(frm, clicked, *options, command=drop_down_options)


ttk.Label(frm, text="Please Select the DEM file (.tif):").grid(column=0, row=0)
ttk.Button(frm, text="Browse", command=select_dem_file).grid(column=1, row=0)

ttk.Label(frm, text="Please Select the shape file (.shp):").grid(column=0, row=1)
ttk.Button(frm, text="Browse", command=select_shapefile).grid(column=1, row=1)

ttk.Label(frm, text="How many points would you like to generate?").grid(column=0, row=2)
# no_points_entry = ttk.Entry(frm, textvariable=no_points_var)  # Use StringVar
# no_points_entry.grid(column=1, row=2)
drop = ttk.Combobox(frm, textvariable=clicked, values=[8, 16], state="readonly")  # Use ttk.Combobox for better UI
drop.grid(column=1, row=2)
# ttk.Button(frm, text="Select", command=drop_down_options).grid(column=1, row=2)



ttk.Label(frm, text = "2d/3d").grid(column=0, row=4)
ttk.Button(frm, text="2D", command=lambda: run_program(2)).grid(column=0, row=4)  # No need for lambda
ttk.Button(frm, text="3D", command=lambda: run_program(3)).grid(column=1, row=4)  # No need for lambda
ttk.Button(frm, text="Just output", command=lambda: run_program(None)).grid(column=2, row=4)  # No need for lambda

# Load and display image
try:
    img = Image.open("example picture.png")
    img = img.resize((200, 200))
    img_tk = ImageTk.PhotoImage(img)
    label_img = ttk.Label(frm, image=img_tk)
    label_img.grid(column=2, row=0, rowspan=4)
    label_img.image = img_tk  # Keep reference
except Exception as e:
    print("Error loading image:", e)

root.mainloop()

# Print final values after GUI closes
print("Final DEM Location:", dem_location)
print("Final Shapefile Location:", shapefile_location)
