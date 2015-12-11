#include "ogmpp_planners/ogmpp_prms/ogmpp_uniform_sampling.hpp"

namespace ogmpp_planners
{
  namespace prms{

    UniformSampling::UniformSampling(void)
    {
    }

    std::vector<ogmpp_graph::Cell> 
      UniformSampling::createPath(
        ogmpp_map_loader::Map& map,
        ogmpp_graph::Node begin, 
        ogmpp_graph::Node end)
    {
      std::vector<ogmpp_graph::Cell> ret;

      std::pair<unsigned int, unsigned int> size = map.getMapSize();
      unsigned int w = size.first;
      unsigned int h = size.second;
      unsigned int step = 10;

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
      _g.visualize();
      return ret;
    }

  }
}
