#include "ogmpp_planners/ogmpp_abstract_planner.hpp"

namespace ogmpp_planners
{
  std::vector<ogmpp_graph::Cell> OgmppAbstractPlanner::_fixPath(
    ogmpp_graph::Graph& g,
    ogmpp_graph::Cell begin,
    ogmpp_graph::Cell end)
  {
    // Find the path
    return ogmpp_search_algorithms::SearchAlgorithms::aStarSearch(
      g, 
      begin, 
      end);
  }

  void OgmppAbstractPlanner::_visualize(
    ogmpp_graph::Graph& g,
    ogmpp_graph::Cell begin,
    ogmpp_graph::Cell end,
    std::vector<ogmpp_graph::Cell>& path)
  {
    // On demand visualize
    g.visualize(begin, end, path);
  }

  std::vector<ogmpp_graph::Cell> OgmppAbstractPlanner::createPath(
      ogmpp_map_loader::Map& map,
      ogmpp_graph::Cell begin, 
      ogmpp_graph::Cell end)
    {
      _g.clean();
      _g = _createGraph(map, begin, end);
      std::vector<ogmpp_graph::Cell> path = _fixPath(_g, begin, end);
      _visualize(_g, begin, end, path);
      return path;
    }
}
