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

    /**
     * @brief Creates the halton sampling graph
     * @param map [ogmpp_map_loader&] The map
     * @param begin [ogmpp_graph::Cell] The starting cell
     * @param end [ogmpp_graph::Cell] The ending cell
     */
    ogmpp_graph::Graph RandomHaltonSampling::_createGraph(
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
          "ogmpp_random_halton_sampling: Robot or goal pose is not unoccupied");
        return _g;
      }

      std::pair<unsigned int, unsigned int> size = map.getMapSize();
      unsigned int w = size.first;
      unsigned int h = size.second;
      int samples = 100;
      int init_samples = 100;
      double neigh_dist = 1.0;
      double min_dist_from_wall = 0.5;
      double randomness_radius = 0.1;

      if(_nh.hasParam("random_halton_sampling_samples"))
        _nh.getParam("random_halton_sampling_samples", samples);
      if(_nh.hasParam("random_halton_sampling_min_neigh_dist"))
        _nh.getParam("random_halton_sampling_min_neigh_dist", neigh_dist);
       if(_nh.hasParam("random_halton_sampling_min_dist_from_wall"))
        _nh.getParam("random_halton_sampling_min_dist_from_wall", min_dist_from_wall);
       if(_nh.hasParam("random_halton_sampling_randomness_radius"))
        _nh.getParam("random_halton_sampling_randomness_radius", randomness_radius);
     
      init_samples = samples;
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
