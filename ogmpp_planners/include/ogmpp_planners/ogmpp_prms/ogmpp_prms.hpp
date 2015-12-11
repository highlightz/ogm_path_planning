#ifndef OGMPP_PRMS_DEF
#define OGMPP_PRMS_DEF

#include "ogmpp_planners/ogmpp_prms/ogmpp_uniform_sampling.hpp"
#include "ogmpp_communications/OgmppPathPlanningMsg.h"

namespace ogmpp_planners
{
  namespace prms
  {

    class ProbabilisticRoadmaps
    {
      private:

        ros::NodeHandle _nh;
        ogmpp_map_loader::Map _map;
        ros::Subscriber _subscriber;
        UniformSampling _uniform_sampling; 

      public:
        
        ProbabilisticRoadmaps(void);

        void uniformCallback(const ogmpp_communications::OgmppPathPlanningMsg& p);


    };

  }
}

#endif
