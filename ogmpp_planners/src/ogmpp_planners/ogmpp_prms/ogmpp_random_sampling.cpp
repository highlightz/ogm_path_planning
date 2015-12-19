#include "ogmpp_planners/ogmpp_prms/ogmpp_random_sampling.hpp"

namespace ogmpp_planners
{
  namespace prms
  {

    std::map<std::string,double> RandomSampling::_fixParameters(
      std::map<std::string, double> parameters)
    {
      std::map<std::string, double> p, temp_map;
      double temp;

      // Add default values
      temp_map.insert(std::pair<std::string, double>
        ("random_sampling_samples", 1000));
      temp_map.insert(std::pair<std::string, double>
        ("random_sampling_min_neigh_dist", 0.7));
      temp_map.insert(std::pair<std::string, double>
        ("random_sampling_min_dist_from_wall", 0.3));

      // Check all for yaml and on-demand parameters
      for(std::map<std::string, double>::iterator it = temp_map.begin() ;
        it != temp_map.end() ; it++)
      {
        p.insert(std::pair<std::string, double>(it->first, it->second));
        if(_nh.hasParam(it->first))
        {
          _nh.getParam(it->first,temp);
          p[it->first] = temp;
        }
        if(parameters.find(it->first) != parameters.end())
        {
          p[it->first] = parameters[it->first];
        }
      }

      return p;
    }

    /**
     * @brief Creates the random sampling graph
     * @param map [ogmpp_map_loader&] The map
     * @param begin [ogmpp_graph::Cell] The starting cell
     * @param end [ogmpp_graph::Cell] The ending cell
     */
    ogmpp_graph::Graph RandomSampling::_createGraph(
        ogmpp_map_loader::Map& map,
        ogmpp_graph::Cell begin, 
        ogmpp_graph::Cell end,
        std::map<std::string, double> parameters)
    {
      
      ogmpp_graph::Graph _g;
      _g.clean();

      if(!map.isUnoccupied(begin.x, begin.y) || 
        !map.isUnoccupied(end.x, end.y))
      {
        ROS_ERROR_STREAM(
          "ogmpp_random_sampling: Robot or goal pose is not unoccupied");
        return _g;
      }

      std::pair<unsigned int, unsigned int> size = map.getMapSize();
      unsigned int w = size.first;
      unsigned int h = size.second;
      
      std::map<std::string, double> p = _fixParameters(parameters);

      int samples = p["random_sampling_samples"];
      double neigh_dist = p["random_sampling_min_neigh_dist"];
      double min_dist_from_wall = p["random_sampling_min_dist_from_wall"];
    
      neigh_dist /= map.getResolution();
      min_dist_from_wall /= map.getResolution();

      _g = ogmpp_graph::Graph(map.getResolution());

      _g.addNode(begin);
      _g.addNode(end);

      // Check if robot and goal are close
      if(begin.distanceFrom(end) <= neigh_dist)
      {
        _g.makeNeighbor(begin, end);
        return _g;
      }

      std::map<unsigned long, ogmpp_graph::Node*>::iterator n_it;

      while(samples > 0)
      {

        long x = rand() % w;
        long y = rand() % h;

        if( map.isOccupied(x, y) || map.isUnknown(x,y) ||
          map.getDistanceTransformation(x, y) < min_dist_from_wall)
        {
          continue;
        }

        samples --;

        ogmpp_graph::Cell tc(x, y);
 
        std::map<unsigned long, ogmpp_graph::Node*> nodes = _g.getNodes();

        _g.addNode(tc);

        for(n_it = nodes.begin() ; n_it != nodes.end() ; n_it++)
        {
          if(tc.distanceFrom(n_it->second->getPose()) < neigh_dist)
          {
            _g.makeNeighbor(tc, n_it->second->getPose());
          }
        }
      }
      return _g;
    }
  }
}
