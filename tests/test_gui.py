from GUI import Gui
from Driver import Driver
from Config import Config
import os

def test_gui_initialization():
    gui = Gui()
    assert gui is not None
    assert gui.config is not None

def test_add_to_terminal():
    gui= Gui()
    gui.add_to_terminal("Test message")
    assert gui.terminal['text'] == "Test message"

def test_generate_picture_none():
    gui = Gui()
    output_path = gui.config.config['io']['output_folder']
    graph_name = gui.config.config['io']['graph_picture_name']
    if os.path.exists(os.path.join(output_path, graph_name)):
        os.remove(os.path.join(output_path, graph_name))
    gui.generate_picture(buffer=False,
                         area=False,
                         standing_location=False,
                         cities=False)

    assert os.path.exists(os.path.join(output_path, graph_name))
    

def test_generate_picture_with_all():
    gui = Gui()
    gui.driver = Driver('Fake path')
    gui.driver.buffer_coords = [[(0, 0), (1, 1)]]
    gui.driver.area_coords = [[(-1, 0), (1, 1)]]
    gui.driver.standing_locations = [(1,1)]
    gui.driver.cities = [(-1,-1)]
    
    output_path = gui.config.config['io']['output_folder']
    graph_name = gui.config.config['io']['graph_picture_name']
    if os.path.exists(os.path.join(output_path, graph_name)):
        os.remove(os.path.join(output_path, graph_name))
    fig = gui.generate_picture(buffer=True,
                         area=True,
                         standing_location=True,
                         cities=True,
                         test=True)

    assert fig is not None
    assert len(fig.axes[0].lines)==2
    assert len(fig.axes[0].collections)==2
    assert os.path.exists(os.path.join(output_path, graph_name))


