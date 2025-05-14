from GUI import Gui

def test_gui_initialization():
    gui = Gui()
    assert gui is not None
    assert gui.config is not None

def test_add_to_terminal():
    gui= Gui()
    gui.add_to_terminal("Test message")
    assert gui.terminal['text'] == "Test message"


