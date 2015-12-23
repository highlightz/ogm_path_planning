#include "ogmpp_planners/ogmpp_rrts/ogmpp_simple_rrt.hpp"

namespace ogmpp_planners
{
  namespace rrts
  {

    std::map<std::string,double> SimpleRRT::_fixParameters(
      std::map<std::string, double> parameters)
    {
      std::map<std::string, double> p, temp_map;
      double temp;

      // Add default values
      temp_map.insert(std::pair<std::string, double>
        ("simple_rrt_samples", 1000));
      temp_map.insert(std::pair<std::string, double>
        ("simple_rrt_min_dist_from_wall", 0.3));
      temp_map.insert(std::pair<std::string, double>
        ("simple_rrt_max_expansion", 0.5));
      temp_map.insert(std::pair<std::string, double>
        ("simple_rrt_distance_from_target", 0.1));

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
    ogmpp_graph::Graph SimpleRRT::_createGraph(
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
          "simple_rrt: Robot or goal pose is not unoccupied");
        return _g;
      }

      std::pair<unsigned int, unsigned int> size = map.getMapSize();
      unsigned int w = size.first;
      unsigned int h = size.second;
      
      std::map<std::string, double> p = _fixParameters(parameters);

      int samples = p["simple_rrt_samples"];
      double max_expansion = p["simple_rrt_max_expansion"];
      double min_dist_from_wall = p["simple_rrt_min_dist_from_wall"];
      double target_dist = p["simple_rrt_distance_from_target"];
    
      max_expansion /= map.getResolution();
      min_dist_from_wall /= map.getResolution();
      target_dist /= map.getResolution();

      _g = ogmpp_graph::Graph(map.getResolution());

      _g.addNode(begin);

      // Check if robot and goal are close
      if(begin.distanceFrom(end) <= target_dist)
      {
        _g.makeNeighbor(begin, end);
        return _g;
      }

      std::map<unsigned long, ogmpp_graph::Node*>::iterator n_it;

      bool reached = false;
      while(!reached && samples > 0)
      {
        //sleep(1);
        long x = rand() % w;
        long y = rand() % h;
        ogmpp_graph::Cell tc(x, y);
        // Find closest node
        std::map<unsigned long, ogmpp_graph::Node*> nodes = _g.getNodes();
        double min_dist = std::numeric_limits<float>::infinity();
        double temp_dist = 0;
        ogmpp_graph::Node *closest_node, *new_node;
        for(n_it = nodes.begin() ; n_it != nodes.end() ; n_it++)
        {
          temp_dist = tc.distanceFrom(n_it->second->getPose());
          if(temp_dist < min_dist)
          {
            min_dist = temp_dist;
            closest_node = n_it->second;
          }
        }
        // Sample towards this direction
        double direction = -atan2(
          closest_node->getPose().y - y, 
          closest_node->getPose().x - x);
        long expansion = (rand() % 10000) / 10000.0 * max_expansion;

        long x_new = closest_node->getPose().x - cos(direction) * expansion;
        long y_new = closest_node->getPose().y + sin(direction) * expansion;
        if( !map.isValid(x_new, y_new) )
          continue;
        if(map.getDistanceTransformation(x_new, y_new) < min_dist_from_wall)
          continue;
        
        samples --;
        ogmpp_graph::Cell new_cell(x_new, y_new);

        _g.addNode(new_cell);
        _g.makeNeighbor(new_cell, closest_node->getPose());
        // Check if this neigbor is close to the target
        if(new_cell.distanceFrom(end) <= target_dist)
        {
          _g.addNode(end);
          _g.makeNeighbor(new_cell, end);
          reached = true;
        }
      }
      return _g;
    }
  }
}
