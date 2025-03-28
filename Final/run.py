import os
import two_d_graphing
import cities
import numpy as np
import pointcloud
import saveoutput

class Main():
    def __init__(self):
        self.dem_location = None
        self.no_points = None
        self.shp_location = None
        self.outline = None
        self.buffer = None
        self.pointcloud = None
        self.twod = None
        self.threed = None
    def get_all_inputs(self):
        self.dem_location = self.dem_input()
        self.shp_location = self.shp_input()
        self.no_points = self.points_input()
        self.twod, self.threed = self.type_input()
        # self.dem_location = 'Data/T691/6_OFO_T691_dtm.tif'
        # self.shp_location = 'Data/T691/2_OFO_T691_bdy.shp'
        # self.no_points = 8

    def dem_input(self):
        '''Gets the file location for the DEM file'''
        while True:
            try:
                dem_location = input(
                    '''Enter the location of the DEM file:\n(a) for default location: Data/MGAT_01201/Base_Data/01201_dem.tif\nOr the route to the DEM file\n'''
                    )
                if dem_location == 'a':
                    dem_location= 'Data/MGAT_01201/Base_Data/01201_dem.tif'
                if os.path.exists(dem_location):
                    return dem_location
                else:
                    raise FileNotFoundError
            except FileNotFoundError:
                print('Please enter a valid file location')


    def points_input(self):
        '''Gets the number of points to generate'''
        while True:
            try:
                no_points = int(input('Enter the number of points: '))
                return no_points
            except ValueError:
                print('Please enter a valid number')

    def shp_input(self):
        '''Gets the file location for the shapefile'''
        while True:
            try:
                shp_location = input(
                    '''Enter the location of the shapefile:\n(a) for default location: Data/MGAT_01201/Base_Data/2_MGAT_01201_bdy.shp\nOr the route to the shapefile\n''')
                if shp_location == 'a':
                    shp_location = 'Data/MGAT_01201/Base_Data/2_MGAT_01201_bdy.shp'
                if os.path.exists(shp_location):
                    return shp_location
                else:
                    raise FileNotFoundError
            except FileNotFoundError:
                print('Please enter a valid file location')

    def type_input(self):
        '''Gets the method of viewing 3d/2d'''
        while True:
            type_input = input('Enter the type of view (2/3): ')
            if type_input == '2':
                return True,False
            elif type_input == '3':
                return False,True
            else:
                print('Please enter a valid type (2/3)')

    def load_shp_file(self):
        '''Loads the shapefile and creates a buffer inside it'''
        shp = cities.Shp_Opener(area_path=self.shp_location)
        self.outline = shp.get_area()
        print('Successfully loaded area shapefile, number of points: ',len(self.outline))
        self.buffer = shp.get_buffered_area(-30)
        print('Successfully loaded buffered area shapefile, number of points: ',len(self.buffer))

    def visualize(self,twod=True,threed=False):
                # shape = np.array([(2023854.8714770428, 5753817.783510998), (2023856.6475916533, 5753816.945746823), (2023858.3329843988, 5753815.937836503), (2023859.9114062735, 5753814.769497379), (2023861.3676395856, 5753813.451993504), (2023862.6876446742, 5753811.998027036), (2023863.8586952647, 5753810.421615783), (2023864.8695011646, 5753808.737958053), (2023865.710317114, 5753806.963286122), (2023866.373036739, 5753805.114709745), (2023866.8512707066, 5753803.210051195), (2023867.1404083252, 5753801.267673436), (2023867.237661996, 5753799.306303087), (2023867.1420940885, 5753797.344849876), (2023866.8546259806, 5753795.402224331), (2023866.378029175, 5753793.497155459), (2023865.7168985798, 5753791.648010181), (2023864.877608208, 5753789.872616255), (2023863.8682497246, 5753788.188090395), (2023862.6985544355, 5753786.610673249), (2023861.3797994666, 5753785.155572821), (2023819.046381466, 5753742.822154821), (2023817.5350904197, 5753741.457460812), (2023815.8928683428, 5753740.253510226), (2023814.1367384698, 5753739.222783184), (2023812.2849048006, 5753738.37596418), (2023744.317205041, 5753711.388789275), (2023715.559414576, 5753681.672405795), (2023714.219910855, 5753680.410042533), (2023712.7700270265, 5753679.276169475), (2023711.222042995, 5753678.28039005), (2023709.589069532, 5753677.431138089), (2023707.8849372335, 5753676.735606397), (2023706.1240793813, 5753676.199685833), (2023607.103135768, 5753650.977370006), (2023573.9571290135, 5753393.844711543), (2023567.6386409416, 5753327.500586792), (2023567.3560857142, 5753325.553987601), (2023566.8836806894, 5753323.644558533), (2023566.2259953064, 5753321.790768954), (2023565.3893911685, 5753320.010550039), (2023564.3819605084, 5753318.321121338), (2023563.2134479156, 5753316.738824214), (2023561.8951560792, 5753315.278963774), (2023560.4398364597, 5753313.955660833), (2023558.8615659487, 5753312.781715322), (2023557.1756107067, 5753311.768482481), (2023555.3982784972, 5753310.925763022), (2023553.546760948, 5753310.26170833), (2023551.6389672598, 5753309.782741617), (2023549.6933509766, 5753309.493495791), (2023547.7287314897, 5753309.396768644), (2023523.387016139, 5753309.396768644), (2023521.5144816982, 5753312.47917803), (2023511.1824559504, 5753313.552251908), (2023432.8656326523, 5753373.877372555), (2023431.250115839, 5753375.2648677565), (2023429.7912854447, 5753376.816270796), (2023428.5056810956, 5753378.513992494), (2023427.4078784515, 5753380.338784773), (2023391.424473152, 5753448.072253575), (2023390.3499572468, 5753450.460274784), (2023360.7165646465, 5753529.835433533), (2023360.03059677, 5753532.060306029), (2023359.607837407, 5753534.349822151), (2023356.415801088, 5753559.886112695), (2023302.8179988686, 5753600.765792354), (2023204.808580312, 5753639.9695597775), (2023202.924394447, 5753640.8391877385), (2023201.1404617806, 5753641.899373292), (2023199.4759881732, 5753643.138702456), (2023197.9488933855, 5753644.543832583), (2023196.575618155, 5753646.099636012), (2023195.3709471945, 5753647.789362925), (2023194.347850019, 5753649.594821685), (2023160.4811156206, 5753717.328290484), (2023159.7465721057, 5753718.980058152), (2023159.164174121, 5753720.6914035855), (2023158.738679699, 5753722.448345561), (2023142.8636479483, 5753803.940175211), (2023142.6045882958, 5753805.67003465), (2023142.497663854, 5753807.4159134375), (2023142.543692468, 5753809.164457674), (2023146.777034269, 5753869.489578325), (2023147.0099867717, 5753871.43632573), (2023147.43243364, 5753873.350909489), (2023148.0403150613, 5753875.214930011), (2023148.8277891553, 5753877.010473628), (2023149.7872881137, 5753878.720284751), (2023150.9095909307, 5753880.3279317), (2023152.1839120167, 5753881.817964618), (2023153.598004852, 5753883.176063945), (2023155.1382796783,5753884.389178031), (2023156.7899340987, 5753885.4456485715), (2023158.537095333, 5753886.33532264), (2023160.3629727587, 5753887.049650264), (2023162.250019272, 5753887.581766589), (2023237.2931836394, 5753904.821412457), (2023259.8448647903, 5753918.916213176), (2023261.74484434, 5753919.964856511), (2023263.74725714, 5753920.8014675975), (2023265.8285269064, 5753921.416196229), (2023267.9641488986, 5753921.80180463), (2023310.297566899, 5753927.09348188), (2023311.8581841134, 5753927.226749572), (2023472.7251725125, 5753934.635097721), (2023474.6271723893, 5753934.632153661), (2023476.52029173, 5753934.448547123), (2023478.3874091054, 5753934.08593865), (2023480.2116382495, 5753933.547607684), (2023481.9764807776, 5753932.838422905), (2023483.6659754002, 5753931.964798204), (2023517.150388648, 5753912.579085272), (2023581.5206145737, 5753926.778399815), (2023583.5208549122, 5753927.114311888), (2023585.5448313814, 5753927.24590818), (2023587.571728404, 5753927.171835292),(2023589.5807003656, 5753926.892855024), (2023591.5510860025, 5753926.411836549), (2023593.462620892, 5753925.733726898), (2023854.8714770428, 5753817.783510998)])
        # example_buffer = np.array([(2023847.2376638427, 5753799.297708445), (2023804.9042458422, 5753756.964290445), (2023732.9374352396, 5753728.389233293), (2023701.187371741, 5753695.580834346), (2023589.0038140398, 5753667.005777194), (2023554.0787441898, 5753396.071901993), (2023547.7287314897, 5753329.396768644), (2023523.387016139, 5753329.396768644), (2023445.0701928409, 5753389.721889292), (2023409.0867875414, 5753457.455358094), (2023379.453394941, 5753536.830516843), (2023375.22005314, 5753570.6972512435), (2023312.7782615907, 5753618.322346494), (2023212.2363938391, 5753658.539093595), (2023178.3696594406, 5753726.272562394), (2023162.49462769, 5753807.764392044), (2023166.727969491, 5753868.089512696), (2023245.044792789, 5753886.081215345), (2023270.4448435903, 5753901.956247096), (2023312.7782615907, 5753907.247924346), (2023473.6452499898, 5753914.656272494), (2023513.8619970912, 5753891.372892595), (2023585.828807692, 5753907.247924346), (2023847.2376638427, 5753799.297708445)])
        
        outline = np.array(self.outline)
        buffer = np.array(self.buffer)
        graph_to_show = two_d_graphing.Graph2D(outline,buffer)

        
        graph_to_show.plot() # Plots area shape
        graph_to_show.plot_buffer()# Plots buffer shape


        unfiltered_points = graph_to_show.generate_random_points(150)
        graph_to_show.remove_unneeded_points(unfiltered_points,self.no_points)
        
        graph_to_show.plot_points()
        graph_to_show.find_path(False)

        # graph_to_show.plot_separated_points()

        graph_to_show.create_transcets()
        graph_to_show.find_transect_route(twod)

        fastest_route = graph_to_show.transect_path
        if threed:
            self.threed_input(fastest_route)
        if twod:
            graph_to_show.show()
        if not threed and not twod:
            self.threed_input(fastest_route,show=False)

    def threed_input(self,fastest_route,show=True):
        '''Visualizes the 3D point cloud'''
        print('Loading point cloud')
        print('Creating 3D point cloud')
        self.pointcloud = pointcloud.PointCloud(self.dem_location)
        self.pointcloud.read_tif()
        self.pointcloud.form_kdtree()
        self.pointcloud.display_path(fastest_route)
        self.save_output(fastest_route)
        if show:
            self.pointcloud.show()
    
    def save_output(self,fastest_route):
        fastest_route = [[x,y,z] for x,y in fastest_route for z in [self.pointcloud.find_altitude([x,y],30)]]
        connections = [[i,i+1] for i in range(len(fastest_route)-1)]

        interpolated_route,_ = self.pointcloud.create_interpolated_line(fastest_route,connections,interpolate=20)
        interpolated_route_with_plot =[]
        seen_interpolated = []
        # //TODO fix this
        for point in interpolated_route: #UGLY CODE fixing a bug elsewhere by removing duplicates
            if point not in seen_interpolated:
                seen_interpolated.append(point)
        interpolated_route = seen_interpolated        
        for point in interpolated_route:
            if point in fastest_route:
                x,y,z = point
                interpolated_route_with_plot.append([x,y,z,True])
            else:
                x,y,z = point
                interpolated_route_with_plot.append([x,y,z,False])
        # interpolated_route_with_plot = [x,y,z,True for x,y,z in interpolated_route if [x,y,z] in fastest_route]

        print('Saving output')
        saver = saveoutput.Converter(self.dem_location)
        saver.convert_mercader_to_lat_long(interpolated_route_with_plot)
        saver.convert_into_correct_form()
        saver.create_csv()





    
if __name__ == "__main__":
    main = Main()

    main.get_all_inputs()
    main.load_shp_file()
    main.visualize(twod=True,threed=False)
    # print(main.dem_location,main.no_points,main.shp_location)