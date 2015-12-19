#include "ogmpp_planners/ogmpp_prms/ogmpp_gaussian_sampling.hpp"

#include <random>

namespace ogmpp_planners
{
  namespace prms
  {
    /**
     * @brief Creates the random sampling graph
     * @param map [ogmpp_map_loader&] The map
     * @param begin [ogmpp_graph::Cell] The starting cell
     * @param end [ogmpp_graph::Cell] The ending cell
     */
    ogmpp_graph::Graph GaussianSampling::_createGraph(
        ogmpp_map_loader::Map& map,
        ogmpp_graph::Cell begin, 
        ogmpp_graph::Cell end)
    {
      
      ogmpp_graph::Graph _g;
      _g.clean();

      if(!map.isUnoccupied(begin.x, begin.y) || 
        !map.isUnoccupied(end.x, end.y))
      {
        ROS_ERROR_STREAM(
          "ogmpp_gaussian_sampling: Robot or goal pose is not unoccupied");
        return _g;
      }

      std::pair<unsigned int, unsigned int> size = map.getMapSize();
      unsigned int w = size.first;
      unsigned int h = size.second;
      int samples = 100;
      double neigh_dist = 1.0;
      double min_dist_from_wall = 0.5;
      double std = 1.0;

      if(_nh.hasParam("gaussian_sampling_samples"))
        _nh.getParam("gaussian_sampling_samples", samples);
      if(_nh.hasParam("gaussian_sampling_min_neigh_dist"))
        _nh.getParam("gaussian_sampling_min_neigh_dist", neigh_dist);
      if(_nh.hasParam("gaussian_sampling_min_dist_from_wall"))
        _nh.getParam("gaussian_sampling_min_dist_from_wall", min_dist_from_wall);
      if(_nh.hasParam("gaussian_sampling_standard_deviation"))
        _nh.getParam("gaussian_sampling_standard_deviation", std);

      neigh_dist /= map.getResolution();
      min_dist_from_wall /= map.getResolution();
      std /= map.getResolution();
    
      // Initialize the normal distribution
      std::default_random_engine generator;
      std::normal_distribution<double> distribution(0.0, std);

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

        long x_2 = distribution(generator) + x;
        long y_2 = distribution(generator) + y;

        if(!map.isValid(x, y) && map.isValid(x_2, y_2))
        {
          x = x_2;
          y = y_2;
        }
        else if(map.isValid(x, y) && !map.isValid(x_2, y_2))
        {
        }
        else
          continue;
        
        if(map.getDistanceTransformation(x, y) < min_dist_from_wall)
          continue;

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
