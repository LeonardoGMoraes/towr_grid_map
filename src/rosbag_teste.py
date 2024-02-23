import rosbag
import numpy as np


bag = rosbag.Bag('/home/leo/mestrado_ws/src/legged_control/rosbag/gridmap/caixa.bag')
for topic, msg, t in bag.read_messages (topics=["/elevation_mapping/elevation_map"]):
    print(msg)



bag.close()

