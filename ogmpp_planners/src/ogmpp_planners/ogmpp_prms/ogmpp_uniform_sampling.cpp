#include "ogmpp_planners/ogmpp_prms/ogmpp_uniform_sampling.hpp"

namespace ogmpp_planners
{
  namespace prms{

    UniformSampling::UniformSampling(void)
    {
      _g.clean();
      _g = ogmpp_graph::Graph(0.2);

      _g.addNode(ogmpp_graph::Cell(10,10));
      _g.addNode(ogmpp_graph::Cell(20,60));
      _g.addNode(ogmpp_graph::Cell(30,50));
      _g.addNode(ogmpp_graph::Cell(10,40));
      _g.addNode(ogmpp_graph::Cell(50,20));
      _g.makeNeighbor(ogmpp_graph::Cell(10,10), ogmpp_graph::Cell(30,50));
      _g.makeNeighbor(ogmpp_graph::Cell(20,60), ogmpp_graph::Cell(50,20));
      _g.print();
    }

  }
}
