#include "rss_grid_localization/MapHandler.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace nav_msgs;

namespace rss {

    void MapHandler::recCallback(const OccupancyGrid::ConstPtr &msg) {
        currentMap.valid = true;
        currentMap.grid = *msg;
        for (unsigned long i = 0; i < currentMap.grid.data.size(); i++) {
            if (currentMap.grid.data[i] < 50) { // probably empty
                currentMap.freeSpaceIndices.push_back(i);
            } else {
                currentMap.occupiedSpaceIndices.push_back(i);
            }
        }
        if (currentMap.freeSpaceIndices.empty()) {
            ROS_WARN("No free space on map!");
        }
    }

    Map MapHandler::currentMap;

}