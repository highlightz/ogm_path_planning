#ifndef OGMPP_PRMS_DEF
#define OGMPP_PRMS_DEF

#include "ogmpp_planners/ogmpp_prms/ogmpp_uniform_sampling.hpp"
#include "ogmpp_planners/ogmpp_prms/ogmpp_random_sampling.hpp"

#include "ogmpp_communications/OgmppPathPlanningMsg.h"
#include "ogmpp_communications/OgmppPathPlanningSrv.h"

namespace ogmpp_planners
{
  namespace prms
  {

    /**
     * @class ProbabilisticRoadmaps
     * @brief Holds all PRM implementation
     */
    class ProbabilisticRoadmaps
    {
      private:

        /**< The ROS NodeHandle */
        ros::NodeHandle _nh;

        /**< Holds the map of the environment */
        ogmpp_map_loader::Map _map;

        /**< The PRM services server */
        ros::ServiceServer _server_uniform;
        ros::ServiceServer _server_random;

        /**< Uniform sampling PRM object */
        UniformSampling _uniform_sampling; 
        RandomSampling _random_sampling; 

      public:

        /**
         * @brief Default constructor. Initializes the ROS communications.
         */
        ProbabilisticRoadmaps(void);

        // TODO: Create a single callback and add the type with ENUM?
        /**
         * @brief The callback for the uniform sampling method
         */
        bool uniformCallback(
          ogmpp_communications::OgmppPathPlanningSrv::Request& req,
          ogmpp_communications::OgmppPathPlanningSrv::Response& res);

        bool randomCallback(
          ogmpp_communications::OgmppPathPlanningSrv::Request& req,
          ogmpp_communications::OgmppPathPlanningSrv::Response& res);


    };

  }
}

#endif
