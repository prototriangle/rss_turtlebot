#!/usr/bin/env python
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rss_map_image_server.srv import LoadMap, LoadMapResponse, LoadMapRequest
import rospy
import numpy as np
import cv2

seq = 0
map_frame = "map"
pub = None


def handle_load_map(req):
    global seq
    global map_frame
    global pub
    width = rospy.get_param("/map_image_params/width")
    height = rospy.get_param("/map_image_params/height")
    res = rospy.get_param("/map_image_params/resolution")
    name = req.name
    if name == "":
        name = rospy.get_param("/map_image_loader/init_map_name")
        print "Load Initial Map: " + name
    else:
        print "Load Map: " + name
    try:
        image = cv2.imread(name, cv2.IMREAD_GRAYSCALE)
        image = cv2.flip(image, 1)
        image = image * (100.0 / 255.0)
        image = np.clip(image, 0, 100)
        map = OccupancyGrid()
        map.data = image.flatten().tolist()
        map.info = MapMetaData(width=width, height=height, resolution=res)
        map.header.frame_id = map_frame
        map.header.seq = seq
        seq = seq + 1
        pub.publish(map)
        return LoadMapResponse(err="Success", map=map)
    except:
        return LoadMapResponse(err="Loading failed!")


def load_map_server():
    global pub
    rospy.init_node('load_map_server')
    s = rospy.Service('load_map', LoadMap, handle_load_map)
    pub = rospy.Publisher('map', OccupancyGrid, queue_size=2, latch=True)  # type: rospy.Publisher
    loadInit = rospy.get_param("/map_image_loader/load_init")  # type: bool
    rospy.loginfo("Ready to load maps")
    if loadInit:
        rospy.loginfo("Loading initial map...")
        req = LoadMapRequest()
        req.name = ""
        handle_load_map(req)
    rospy.spin()


if __name__ == "__main__":
    load_map_server()
