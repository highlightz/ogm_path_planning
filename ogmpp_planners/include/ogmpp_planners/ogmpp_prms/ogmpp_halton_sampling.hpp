#ifndef OGMPP_HALTON_SAMPLING_DEF
#define OGMPP_HALTON_SAMPLING_DEF

#include "ogmpp_planners/ogmpp_abstract_planner.hpp"

namespace ogmpp_planners
{
  namespace prms
  {

    /**
     * @class HaltonSampling
     * Inherits publicly from OgmppAbstractPlanner
     */
    class HaltonSampling: public OgmppAbstractPlanner
    {
      private:

        /**
         * @brief Creates the halton sampling graph
         * @param map [ogmpp_map_loader&] The map
         * @param begin [ogmpp_graph::Cell] The starting cell
         * @param end [ogmpp_graph::Cell] The ending cell
         */
        ogmpp_graph::Graph _createGraph(
            ogmpp_map_loader::Map &map, 
            ogmpp_graph::Cell begin, 
            ogmpp_graph::Cell end,
            std::map<std::string, double> parameters);

        float createHaltonSample(int id, int prime);

        std::map<std::string,double> _fixParameters(
          std::map<std::string, double> parameters);

    };

  }
}

#endif
