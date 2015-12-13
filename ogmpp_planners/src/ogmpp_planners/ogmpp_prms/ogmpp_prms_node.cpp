#include "ogmpp_planners/ogmpp_prms/ogmpp_prms.hpp"

/**
 * @brief The main class of PRMs
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ogmpp_prms_node");

  ogmpp_planners::prms::ProbabilisticRoadmaps pr;

  ros::spin();
  return 0;
}
