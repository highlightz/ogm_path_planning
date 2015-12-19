#ifndef OGMPP_MAP_LOADER_DEF
#define OGMPP_MAP_LOADER_DEF

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace ogmpp_map_loader
{
  class Map
  {
    private:
      nav_msgs::OccupancyGrid _ros_map;
      char **_map;
      long **_brushfire;
      float _resolution;
      std::string _map_topic;
      ros::NodeHandle _nh;
      ros::Subscriber _map_subscriber;

      bool _map_initialized;

    public:

      Map(void);

      void mapCallback(const nav_msgs::OccupancyGrid& map);

      bool isMapInitialized(void);

      float getResolution(void);

      std::pair<unsigned int, unsigned int> getMapSize();

      bool isUnoccupied(long x, long y);
      bool isOccupied(long x, long y);
      bool isUnknown(long x, long y);
      bool isValid(long x, long y);

      long getDistanceTransformation(long x, long y);
    // NOTE: Create function to check if two points' connection intersects
    // an obstacle
  };
}

#endif
