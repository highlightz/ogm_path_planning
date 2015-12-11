#include "ogmpp_map_loader/ogmpp_map_loader.hpp"

namespace ogmpp_map_loader
{
  Map::Map(void):
    _map_initialized(false)
  {
    if(_nh.hasParam("map_topic"))
    {
      _nh.getParam("map_topic", _map_topic);
      _map_topic = "/" + _map_topic;
      ROS_INFO_STREAM("ogmpp_map_loader: Map topic: " << _map_topic); 
    }
    else
    {
      ROS_ERROR("ogmpp_map_loaded: No map topic defined. Assuming \"/map\"");
    }

    ros::spinOnce();
    _map_subscriber = _nh.subscribe(_map_topic.c_str(), 1, 
      &Map::mapCallback, this);
    ros::spinOnce();
  }

  void Map::mapCallback(const nav_msgs::OccupancyGrid& map)
  {
    _ros_map = map;

    _resolution = map.info.resolution;
    unsigned int width = map.info.width;
    unsigned int height = map.info.height;

    /**< NOTE: Must check if the map was allocated beforehand */

    _map = new char*[width];
    for(unsigned int i = 0 ; i < width ; i++)
    {
      _map[i] = new char[height];
      for(unsigned int j = 0 ; j < height ; j++)
      {
        _map[i][j] = map.data[j * width + i];
      }
    }

    ROS_INFO_STREAM("ogmpp_map_loader: Map initialized");
    _map_initialized = true;
  }

  bool Map::isMapInitialized(void)
  {
    return _map_initialized;
  }

  float Map::getResolution(void)
  {
    return _resolution;
  }

  std::pair<unsigned int, unsigned int> Map::getMapSize()
  {
    return std::pair<unsigned int, unsigned int>
      (_ros_map.info.width, _ros_map.info.height);
  }

  bool Map::isOccupied(unsigned int x, unsigned int y)
  {
    if (x >= _ros_map.info.width  || x < 0 || 
      y >= _ros_map.info.height || y < 0)
    {
      return false;
    }
    return _map[x][y] > 50 && !isUnknown(x, y);
  }
  bool Map::isUnoccupied(unsigned int x, unsigned int y)
  {
    if (x >= _ros_map.info.width || x < 0 || 
      y >= _ros_map.info.height || y < 0)
    {
      return false;
    }
    return _map[x][y] < 50 && !isUnknown(x, y);
  }
  bool Map::isUnknown(unsigned int x, unsigned int y)
  {
    if (x >= _ros_map.info.width || x < 0 || 
      y >= _ros_map.info.height || y < 0)
    {
      return false;
    }
    return _map[x][y] == 50 || _map[x][y] == -1;
  }

}

