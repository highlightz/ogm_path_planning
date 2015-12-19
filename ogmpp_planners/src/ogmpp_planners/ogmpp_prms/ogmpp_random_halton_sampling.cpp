#include "ogmpp_planners/ogmpp_prms/ogmpp_random_halton_sampling.hpp"

namespace ogmpp_planners
{
  namespace prms
  {

    float RandomHaltonSampling::createHaltonSample(int id, int prime)
    {
      float h = 0.0;
      float f = 1.0 / prime;
      float fct = 1.0;
      while( id > 0 )
      {
        fct *= f;
        h += (id % prime) * fct;
        id /= prime;
      }
      return h;
    }

    std::map<std::string,double> RandomHaltonSampling::_fixParameters(
      std::map<std::string, double> parameters)
    {
      std::map<std::string, double> p, temp_map;
      double temp;

      // Add default values
      temp_map.insert(std::pair<std::string, double>
        ("random_halton_sampling_samples", 1000));
      temp_map.insert(std::pair<std::string, double>
        ("random_halton_sampling_min_neigh_dist", 0.7));
      temp_map.insert(std::pair<std::string, double>
        ("random_halton_sampling_min_dist_from_wall", 0.3));
      temp_map.insert(std::pair<std::string, double>
        ("random_halton_sampling_randomness_radius", 0.1));

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
     * @brief Creates the halton sampling graph
     * @param map [ogmpp_map_loader&] The map
     * @param begin [ogmpp_graph::Cell] The starting cell
     * @param end [ogmpp_graph::Cell] The ending cell
     */
    ogmpp_graph::Graph RandomHaltonSampling::_createGraph(
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
          "ogmpp_random_halton_sampling: Robot or goal pose is not unoccupied");
        return _g;
      }

      std::pair<unsigned int, unsigned int> size = map.getMapSize();
      unsigned int w = size.first;
      unsigned int h = size.second;

      std::map<std::string, double> p = _fixParameters(parameters);

      int samples = p["random_halton_sampling_samples"];
      double neigh_dist = p["random_halton_sampling_min_neigh_dist"];
      double min_dist_from_wall = p["random_halton_sampling_min_dist_from_wall"];
      double randomness_radius = p["random_halton_sampling_randomness_radius"];

      int init_samples = samples;
      neigh_dist /= map.getResolution();
      min_dist_from_wall /= map.getResolution();
      randomness_radius /= map.getResolution();

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

      long counter = 0;

      while(samples > 0)
      {
        long x = createHaltonSample(counter, 2) * w;
        long y = createHaltonSample(counter, 3) * h;
        counter ++;

        long x_corr = (rand() % 10000) / 10000.0 * randomness_radius * 2.0 - 
          randomness_radius;
        long y_corr = (rand() % 10000) / 10000.0 * randomness_radius * 2.0 - 
          randomness_radius;
        
        x += x_corr;
        y += y_corr;

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
