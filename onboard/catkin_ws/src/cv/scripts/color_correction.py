# pyright: reportMissingImports=false, reportUnusedImport=false, reportUnboundVariable=false, reportUndefinedVariable=false
# flake8: noqa
"""
This script performs underwater color correction on images from a DepthAI camera.
It must be run on the DepthAI device itself, using a script node in the pipeline.
See depthai_spatial_detection.py for an example.
"""

"""
Reverse engineering notes:

Helpful links:
https://discuss.luxonis.com/d/825-get-depth-value-in-script-embeded-in-camera

node.io['cam_rgb'].get() is an lpb.ImgFrame object with the following attributes:
['Type', '__class__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__',
'__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__',
'__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__',
'__sizeof__', '__str__', '__subclasshook__', 'getCategory', 'getColorTemperature', 'getData',
'getExposureTime', 'getHeight', 'getInstanceNum', 'getLensPosition', 'getLensPositionRaw',
'getSensitivity', 'getSequenceNum', 'getTimestamp', 'getTimestampDevice', 'getType', 'getWidth',
'setCategory', 'setData', 'setHeight', 'setInstanceNum', 'setSequenceNum', 'setSize', 'setTimestamp',
'setType', 'setWidth']

len(getData()) is 519168
This is the size of the image, which is 416 * 416 * 3 = 519168 (height * width * channels)

getData() is a memoryview object (see https://docs.python.org/3/library/stdtypes.html#memoryview)
"""

channel_size = 416 * 416

def print(val):
    node.warn(str(val))

# Function to calculate the average of a list of integers
def calculate_average(channel_data):
    return sum(channel_data) // len(channel_data)

# Assuming `data` is your bytearray and `channel_size` is the size of each channel
def get_channel_averages(data):
    # Extract each channel's data
    channel_1 = data[:channel_size]
    channel_2 = data[channel_size:channel_size*2]
    channel_3 = data[channel_size*2:channel_size*3]

    # Calculate the average for each channel
    avg_channel_1 = calculate_average(channel_1)
    avg_channel_2 = calculate_average(channel_2)
    avg_channel_3 = calculate_average(channel_3)

    # Return the averages as a tuple
    return (avg_channel_1, avg_channel_2, avg_channel_3)

def get_correction(data: memoryview):
    avg_mat = get_channel_averages(data)
    print(avg_mat)

def set_color(data: memoryview, channel: 0 | 1 | 2):
    # blue = 0
    # green = 1
    # red = 2

    # Calculate start and end indices for the current channel
    start_index = channel_size * channel
    end_index = start_index + channel_size

    # Create a new bytearray with the required values
    new_data = bytearray(len(data))

    # Use slicing to set the values more efficiently
    new_data[:start_index] = b'\x00' * start_index
    new_data[start_index:end_index] = b'\xff' * channel_size
    new_data[end_index:] = b'\x00' * (len(data) - end_index)

    # Update the original data
    return new_data


while True:
    frame: 'lpb.ImgFrame' = node.io['cam_rgb'].get()
    data: memoryview = frame.getData()

    # new_data = set_color(data, channel=0)
    new_data = get_correction(data)
    # frame.setData(new_data)

    node.io['spatial_detection_network'].send(frame)

    print('Processed frame')
