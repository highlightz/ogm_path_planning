#ifndef OGMPP_PRMS_DEF
#define OGMPP_PRMS_DEF

#include "ogmpp_planners/ogmpp_prms/ogmpp_uniform_sampling.hpp"
#include "ogmpp_communications/OgmppPathPlanningMsg.h"
#include "ogmpp_communications/OgmppPathPlanningSrv.h"

namespace ogmpp_planners
{
  namespace prms
  {

    class ProbabilisticRoadmaps
    {
      private:

        ros::NodeHandle _nh;
        ogmpp_map_loader::Map _map;
        UniformSampling _uniform_sampling; 
        ros::ServiceServer _path_planning_server;

      public:

        ProbabilisticRoadmaps(void);

        bool uniformCallback(
          ogmpp_communications::OgmppPathPlanningSrv::Request& req,
          ogmpp_communications::OgmppPathPlanningSrv::Response& res);

    };

  }
}

#endif
