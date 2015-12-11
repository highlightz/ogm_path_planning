#ifndef OGMPP_UNIFORM_SAMPLING_DEF
#define OGMPP_UNIFORM_SAMPLING_DEF

#include "ogmpp_graph/ogmpp_graph.hpp"
#include "ogmpp_map_loader/ogmpp_map_loader.hpp"

namespace ogmpp_planners
{
  namespace prms
  {

    class UniformSampling
    {
      private:

        ogmpp_graph::Graph _g;

      public:

        UniformSampling(void);

        void createPath(ogmpp_map_loader::Map &map, 
          ogmpp_graph::Node begin, ogmpp_graph::Node end);
    };

  }
}

#endif
