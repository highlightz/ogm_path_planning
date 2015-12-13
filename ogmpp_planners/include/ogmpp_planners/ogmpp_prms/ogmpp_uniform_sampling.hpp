#ifndef OGMPP_UNIFORM_SAMPLING_DEF
#define OGMPP_UNIFORM_SAMPLING_DEF

#include "ogmpp_planners/ogmpp_abstract_planner.hpp"

namespace ogmpp_planners
{
  namespace prms
  {

    class UniformSampling: public OgmppAbstractPlanner
    {
      private:
        ogmpp_graph::Graph _createGraph(
            ogmpp_map_loader::Map &map, 
            ogmpp_graph::Cell begin, 
            ogmpp_graph::Cell end);
    };

  }
}

#endif
