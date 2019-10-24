#include "MapHandler.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace nav_msgs;

namespace rss {

    void MapHandler::recCallback(const OccupancyGrid::ConstPtr &msg) {
        currentMap.grid = *msg;
        currentMap.valid = true;
    }

    Map MapHandler::currentMap;

}