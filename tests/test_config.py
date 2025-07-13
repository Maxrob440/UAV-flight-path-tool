import os
from Config import Config

def test_create_config():
    config = Config()
    os.remove('config.json')
    config.load_config()
    possible = os.listdir()
    assert 'config.json' in possible, "Config file should be created if it does not exist"

def test_update_nested():
    config = Config()
    config.load_config()
    config.update_nested(['distances', 'height_above_ground_m'], '0')
    assert config.get_nested('distances', 'height_above_ground_m') == '0'

    assert isinstance(config.config['distances']['height_above_ground_m'],dict)

    assert config.config['distances']['height_above_ground_m']['default_value']=='25'
    assert config.config['distances']['height_above_ground_m']['value']=='0'

def test_update_nested_for_a_string():
    config= Config()
    config.load_config()
    config.update_nested(['current_map', 'folder_location'], 'HERE')
    assert config.get_nested('current_map', 'folder_location') == 'HERE'