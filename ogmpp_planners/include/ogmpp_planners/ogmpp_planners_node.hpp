#ifndef OGMPP_PLANNERS_NODE_DEF
#define OGMPP_PLANNERS_NODE_DEF

#include "ogmpp_planners/ogmpp_prms/ogmpp_uniform_sampling.hpp"
#include "ogmpp_planners/ogmpp_prms/ogmpp_random_sampling.hpp"
#include "ogmpp_planners/ogmpp_prms/ogmpp_halton_sampling.hpp"

#include "ogmpp_communications/OgmppPathPlanningMsg.h"
#include "ogmpp_communications/OgmppPathPlanningSrv.h"


/**< The generic ogmpp planners namespace */
namespace ogmpp_planners
{
  class OgmppPlanners
  {
    private:
      /**< Holds the map of the environment */
      ogmpp_map_loader::Map _map;
      ros::NodeHandle _nh;
      ros::ServiceServer _server_path_planning;
      /**< Sampling PRM objects */
      prms::UniformSampling _uniform_sampling; 
      prms::RandomSampling _random_sampling; 
      prms::HaltonSampling _halton_sampling; 

      bool planCallback(
        ogmpp_communications::OgmppPathPlanningSrv::Request& req,
        ogmpp_communications::OgmppPathPlanningSrv::Response& res);

    public:
      OgmppPlanners(void);
  };

}



#endif

