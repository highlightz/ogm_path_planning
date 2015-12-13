#ifndef OGMPP_PLANNER_ABSTRACT_DEF
#define OGMPP_PLANNER_ABSTRACT_DEF

#include "ogmpp_graph/ogmpp_graph.hpp"
#include "ogmpp_map_loader/ogmpp_map_loader.hpp"
#include "ogmpp_search_algorithms/ogmpp_search_algorithms.hpp"

namespace ogmpp_planners
{

  class OgmppAbstractPlanner
  {
    private:

      // TODO: Erase this and add a visualizer
      ogmpp_graph::Graph _g;

      virtual ogmpp_graph::Graph _createGraph(
        ogmpp_map_loader::Map &map,
        ogmpp_graph::Cell begin,
        ogmpp_graph::Cell end) = 0;

      virtual std::vector<ogmpp_graph::Cell> _fixPath(
        ogmpp_graph::Graph& g,
        ogmpp_graph::Cell begin,
        ogmpp_graph::Cell end);

      void _visualize(
        ogmpp_graph::Graph& g,
        ogmpp_graph::Cell begin,
        ogmpp_graph::Cell end,
        std::vector<ogmpp_graph::Cell>& path);

    public:

      std::vector<ogmpp_graph::Cell>  
        createPath(
          ogmpp_map_loader::Map &map, 
          ogmpp_graph::Cell begin, 
          ogmpp_graph::Cell end);
  };

}



#endif

