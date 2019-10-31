#ifndef RSS_LOCALIZATION_MAPHANDLER_H
#define RSS_LOCALIZATION_MAPHANDLER_H

#include "nav_msgs/OccupancyGrid.h"
#include "util.h"

using namespace nav_msgs;

namespace rss {

    class MapHandler {
    public:
        static Map currentMap;

        static void recCallback(const OccupancyGrid::ConstPtr &msg);
    };

}

#endif //RSS_LOCALIZATION_MAPHANDLER_H
