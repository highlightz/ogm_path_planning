#include "ogmpp_planners/ogmpp_prms/ogmpp_cell_based_sampling.hpp"

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
    ogmpp_graph::Graph CellBasedSampling::_createGraph(
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
          "ogmpp_random_sampling: Robot or goal pose is not unoccupied");
        return _g;
      }

      std::pair<unsigned int, unsigned int> size = map.getMapSize();
      unsigned int w = size.first;
      unsigned int h = size.second;
      int steps = 6;
      double neigh_dist = 1.0;
      double min_dist_from_wall = 0.5;

      if(_nh.hasParam("cell_based_sampling_steps"))
        _nh.getParam("cell_based_sampling_steps", steps);
      if(_nh.hasParam("cell_based_sampling_min_neigh_dist"))
        _nh.getParam("cell_based_sampling_min_neigh_dist", neigh_dist);
       if(_nh.hasParam("cell_based_sampling_min_dist_from_wall"))
        _nh.getParam("cell_based_sampling_min_dist_from_wall", min_dist_from_wall);
     
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

      int it_counter = 0;
      
      while(it_counter < steps)
      {
        for(unsigned int i = 0 ; i < pow(2, it_counter) ; i++)
        {
          for(unsigned int j = 0 ; j < pow(2, it_counter) ; j++)
          {
            long x = rand() % w;
            long y = rand() % h;

            if( map.isOccupied(x, y) || map.isUnknown(x,y) ||
              map.getDistanceTransformation(x, y) < min_dist_from_wall)
            {
              continue;
            }

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
        }
        it_counter ++;
      }
      return _g;
    }
  }
}
