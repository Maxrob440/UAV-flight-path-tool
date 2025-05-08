import os
from Config import Config

def test_create_config():
    config = Config()
    os.remove('config.json')
    config.load_config()
    possible = os.listdir()
    assert 'config.json' in possible, "Config file should be created if it does not exist"