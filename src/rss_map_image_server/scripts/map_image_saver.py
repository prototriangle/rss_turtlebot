#!/usr/bin/env python
import datetime
from rss_map_image_server.srv import SaveMap, SaveMapResponse
import rospy
import numpy as np
import cv2

output_fname_prefix = "RSSMAP_"


def handle_save_map(req):
    width = rospy.get_param("/map_image_params/width")
    height = rospy.get_param("/map_image_params/height")
    img = np.array(req.map.data).reshape((width, height))
    img = img * (255.0 / 100.0)
    filename = req.name
    if filename == "":
        filename = output_fname_prefix + datetime.now().strftime("%m%d|%H%M") + ".bmp"
    try:
        print "Saving Map: " + req.name
        cv2.imwrite(filename, img)
        return SaveMapResponse("Success")
    except:
        return SaveMapResponse("Saving failed!")


def save_map_server():
    rospy.init_node('save_map_server')
    s = rospy.Service('save_map', SaveMap, handle_save_map)
    print "Ready to save maps"
    rospy.spin()


if __name__ == "__main__":
    save_map_server()
