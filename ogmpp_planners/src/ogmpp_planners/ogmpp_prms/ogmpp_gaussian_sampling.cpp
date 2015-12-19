#include "ogmpp_planners/ogmpp_prms/ogmpp_gaussian_sampling.hpp"

#include <random>

namespace ogmpp_planners
{
  namespace prms
  {

    std::map<std::string,double> GaussianSampling::_fixParameters(
      std::map<std::string, double> parameters)
    {
      std::map<std::string, double> p, temp_map;
      double temp;

      // Add default values
      temp_map.insert(std::pair<std::string, double>
        ("gaussian_sampling_samples", 1000));
      temp_map.insert(std::pair<std::string, double>
        ("gaussian_sampling_min_neigh_dist", 0.7));
      temp_map.insert(std::pair<std::string, double>
        ("gaussian_sampling_min_dist_from_wall", 0.3));
      temp_map.insert(std::pair<std::string, double>
        ("gaussian_sampling_standard_deviation", 2.5));

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
    ogmpp_graph::Graph GaussianSampling::_createGraph(
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
          "ogmpp_gaussian_sampling: Robot or goal pose is not unoccupied");
        return _g;
      }

      std::pair<unsigned int, unsigned int> size = map.getMapSize();
      unsigned int w = size.first;
      unsigned int h = size.second;
 
      std::map<std::string, double> p = _fixParameters(parameters);

      int samples = p["gaussian_sampling_samples"];
      double neigh_dist = p["gaussian_sampling_min_neigh_dist"];
      double min_dist_from_wall = p["gaussian_sampling_min_dist_from_wall"];
      double std = p["gaussian_sampling_standard_deviation"];

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
