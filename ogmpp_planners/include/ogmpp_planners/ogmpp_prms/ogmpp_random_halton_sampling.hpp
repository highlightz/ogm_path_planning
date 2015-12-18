#ifndef OGMPP_RANDOM_HALTON_SAMPLING_DEF
#define OGMPP_RANDOM_HALTON_SAMPLING_DEF

#include "ogmpp_planners/ogmpp_abstract_planner.hpp"

namespace ogmpp_planners
{
  namespace prms
  {

    /**
     * @class HaltonSampling
     * Inherits publicly from OgmppAbstractPlanner
     */
    class RandomHaltonSampling: public OgmppAbstractPlanner
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
            ogmpp_graph::Cell end);

        float createHaltonSample(int id, int prime);
    };

  }
}

#endif
