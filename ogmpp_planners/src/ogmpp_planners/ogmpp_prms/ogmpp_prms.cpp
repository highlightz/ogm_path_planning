#include "ogmpp_planners/ogmpp_prms/ogmpp_prms.hpp"

namespace ogmpp_planners
{
  namespace prms
  {

    ProbabilisticRoadmaps::ProbabilisticRoadmaps(void)
    {
      _path_planning_server = _nh.advertiseService(
        "/path_planners/prms/uniform",
        &ProbabilisticRoadmaps::uniformCallback, this);
    }

    bool
     ProbabilisticRoadmaps::uniformCallback(
      ogmpp_communications::OgmppPathPlanningSrv::Request& req,
      ogmpp_communications::OgmppPathPlanningSrv::Response& res)
    {
      // Check if map is initialized
      if(_map.isMapInitialized() == false)
      {
        ROS_WARN_STREAM(
          "ogmpp_planners::prms: Map is not initialized... waiting...");
        while(_map.isMapInitialized() == false)
        {
          usleep(100000);
        }
      }
      ogmpp_graph::Cell begin(req.data.begin.x, req.data.begin.y);
      ogmpp_graph::Cell end(req.data.end.x, req.data.end.y);

      std::vector<ogmpp_graph::Cell> p =
        _uniform_sampling.createPath(_map, begin, end);

      nav_msgs::Path path;
      res.path = path;
      res.error = "";

      return true;
    }

  }
}