from saveoutput import Converter
import pandas as pd

def test_convert_mercader_to_lat_long():
    pass

def test_create_bearings():
    converter = Converter("dummy.tif")
    converter.lat_long_coords=[(50.71496966354666, -3.5107196508254517,0,True),(51.49133881869516, -0.18442072115086947,0,True)]
    bearings = converter.create_bearings()
    assert len(bearings) == 2
    assert bearings[1] == 0
    assert round(bearings[0],0) == 68

def test_create_csv():
    converter = Converter("dummy.tif")
    converter.lat_long_coords=[(50.71496966354666, -3.5107196508254517,0,True),(51.49133881869516, -0.18442072115086947,0,False)]
    converter.bearings=[68,0]
    converter.create_csv("1")
    df = pd.read_csv("OUTPUT/Examples/PATH1.csv")
    assert len(df) == 2
    assert df.iloc[0]['latitude'] == 50.71496966354666
    assert round(df.iloc[0]['longitude'],5) == round(-3.5107196508254517,5)
    assert df.iloc[0]['altitude(m)'] == 0
    assert round(df.iloc[0]['heading'],0) == 68
    assert df.iloc[0]['photo_timeinterval'] ==2
    assert df.iloc[0]['speed(m/s)'] == 0.8
    
    assert df.iloc[1]['latitude'] == 51.49133881869516
    assert round(df.iloc[1]['longitude'],5) == round(-0.18442072115086947,5)
    assert df.iloc[1]['altitude(m)'] == 0
    assert round(df.iloc[1]['heading'],0) == 0
    assert df.iloc[1]['photo_timeinterval'] ==-1
    assert df.iloc[1]['speed(m/s)'] == 0


