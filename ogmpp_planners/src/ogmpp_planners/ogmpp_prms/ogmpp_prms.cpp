#include "ogmpp_planners/ogmpp_prms/ogmpp_prms.hpp"

namespace ogmpp_planners
{
  namespace prms
  {

    ProbabilisticRoadmaps::ProbabilisticRoadmaps(void)
    {
      // This should be a service
      _subscriber = _nh.subscribe(
        "/path_planners/prms/uniform", 1, // Topic should go in yaml
        &ProbabilisticRoadmaps::uniformCallback, this);

      ROS_INFO_STREAM("ogmpp_planners::prms: Callbacks initialized");
    }

    void ProbabilisticRoadmaps::uniformCallback(
      const ogmpp_communications::OgmppPathPlanningMsg& p)
    {
      // Check if map is initialized
      ROS_ERROR("AA");
      ogmpp_graph::Cell begin(p.begin.x, p.begin.y);
      ogmpp_graph::Cell end(p.end.x, p.end.y);
      _uniform_sampling.createPath(_map, begin, end);
    }

  }
}
