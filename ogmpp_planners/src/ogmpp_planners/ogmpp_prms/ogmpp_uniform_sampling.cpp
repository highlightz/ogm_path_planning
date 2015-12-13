#include "ogmpp_planners/ogmpp_prms/ogmpp_uniform_sampling.hpp"

namespace ogmpp_planners
{
  namespace prms
  {
    /**
     * @brief Creates the uniform sampling graph
     * @param map [ogmpp_map_loader&] The map
     * @param begin [ogmpp_graph::Cell] The starting cell
     * @param end [ogmpp_graph::Cell] The ending cell
     */
    ogmpp_graph::Graph UniformSampling::_createGraph(
        ogmpp_map_loader::Map& map,
        ogmpp_graph::Cell begin, 
        ogmpp_graph::Cell end)
    {
      std::pair<unsigned int, unsigned int> size = map.getMapSize();
      unsigned int w = size.first;
      unsigned int h = size.second;
      unsigned int step = 15; // NOTE: This should be in a param

      ogmpp_graph::Graph _g;
      _g.clean();
      _g = ogmpp_graph::Graph(map.getResolution());

      int x = 0;
      int y = 0;

      while(x < w)
      {
        y = 0;
        while(y < h)
        {
          if(map.isUnoccupied(x, y) && 
            map.getDistanceTransformation(x, y) > step / 2)
          {
            _g.addNode(ogmpp_graph::Cell(x, y));

            // Make connections
            if(x - step >= 0 && y - step >= 0)
              if(_g.getNode(ogmpp_graph::Cell(x - step, y - step)) != NULL)
                _g.makeNeighbor(
                  ogmpp_graph::Cell(x, y),
                  ogmpp_graph::Cell(x - step, y - step));
            if(y - step >= 0)
              if(_g.getNode(ogmpp_graph::Cell(x, y - step)) != NULL)
                _g.makeNeighbor(
                  ogmpp_graph::Cell(x, y),
                  ogmpp_graph::Cell(x, y - step));
            if(x - step >= 0 && y + step < h)
              if(_g.getNode(ogmpp_graph::Cell(x - step, y + step)) != NULL)
                _g.makeNeighbor(
                  ogmpp_graph::Cell(x, y),
                  ogmpp_graph::Cell(x - step, y + step));
            if(x - step >= 0)
              if(_g.getNode(ogmpp_graph::Cell(x - step, y)) != NULL)
                _g.makeNeighbor(
                  ogmpp_graph::Cell(x, y),
                  ogmpp_graph::Cell(x - step, y));
          }
          y += step;
        }
        x += step;
      }

      // Add robot and goal poses
      if(!map.isUnoccupied(begin.x, begin.y) || 
        !map.isUnoccupied(end.x, end.y))
      {
        ROS_ERROR_STREAM(
          "ogmpp_uniform_sampling: Robot or goal pose is not unoccupied");
        _g.clean();
        return _g;
      }
      _g.addNode(begin);
      _g.addNode(end);
      ogmpp_graph::Cell base_cell_begin = ogmpp_graph::Cell(
        (begin.x / step) * step,
        (begin.y / step) * step);
      for(int i = -2 ; i <= 2; i++)
      {
        for(int j = -2 ; j <= 2 ; j++)
        {
          long xx = base_cell_begin.x + i * step;
          long yy = base_cell_begin.y + j * step;
          if( xx < 0 || xx >= w || yy < 0 || yy >= h) 
            continue;
          if(_g.getNode(ogmpp_graph::Cell(xx,yy)) != NULL)
          {
            _g.makeNeighbor(ogmpp_graph::Cell(xx, yy), begin);
          }
        }
      }
      ogmpp_graph::Cell base_cell_end = ogmpp_graph::Cell(
        (end.x / step) * step,
        (end.y / step) * step);
      for(int i = -1 ; i <= 1; i++)
      {
        for(int j = -1 ; j <= 1 ; j++)
        {
          long xx = base_cell_end.x + i * step;
          long yy = base_cell_end.y + j * step;
          if( xx < 0 || xx >= w || yy < 0 || yy >= h) 
            continue;
          if(_g.getNode(ogmpp_graph::Cell(xx,yy)) != NULL)
          {
            _g.makeNeighbor(ogmpp_graph::Cell(xx, yy), end);
          }
        }
      }

      return _g;
    }
  }
}
