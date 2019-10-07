#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import cv2
from datetime import datetime

width = 459
height = 459
res = 0.01

initial_map_fname = "RSSMapCleanScaled.bmp"
output_fname_prefix = "RSS_MAP_"


def map_save_callback(data):
    img = np.array(data.data).reshape((width, height))
    img = img * (255.0 / 100.0)
    cv2.imwrite(output_fname_prefix + datetime.now().strftime("%m%d|%H%M") + ".bmp", img)


if __name__ == '__main__':
    image = cv2.imread(initial_map_fname, cv2.IMREAD_GRAYSCALE)
    image = image * (100.0 / 255.0)
    image = np.clip(image, 0, 100)
    seq = 0
    initial_grid = OccupancyGrid()
    initial_grid.data = image.flatten().tolist()
    initial_grid.info = MapMetaData(height=height, width=width, resolution=res)
    initial_grid.header.seq = seq

    try:
        rospy.init_node('rss_map_image_server', anonymous=False)
        start_string = "Loading initial map... (%s)" % rospy.get_time()
        rospy.loginfo(start_string)
        rospy.Subscriber("/save_map", OccupancyGrid, map_save_callback)
        pub = rospy.Publisher('initial_map', OccupancyGrid, queue_size=10, latch=True)
        pub.publish(initial_grid)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

# np.savetxt('output.txt', img, fmt="%1u")
