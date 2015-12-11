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
      unsigned char **_map;
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
  };
}

#endif
