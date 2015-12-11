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

      _g.clean();
      _g = ogmpp_graph::Graph(map.getResolution());

      _g.addNode(ogmpp_graph::Cell(10,10));
      _g.addNode(ogmpp_graph::Cell(20,60));
      _g.addNode(ogmpp_graph::Cell(30,50));
      _g.addNode(ogmpp_graph::Cell(10,40));
      _g.addNode(ogmpp_graph::Cell(50,20));

      _g.makeNeighbor(ogmpp_graph::Cell(10,10), ogmpp_graph::Cell(30,50));
      _g.makeNeighbor(ogmpp_graph::Cell(20,60), ogmpp_graph::Cell(50,20));

      return ret;
    }

  }
}
