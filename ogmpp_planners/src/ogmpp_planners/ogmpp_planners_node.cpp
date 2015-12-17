#include "ogmpp_planners/ogmpp_prms/ogmpp_prms.hpp"

/**
 * @brief The main class of planners
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ogmpp_planners_node");

  ogmpp_planners::prms::ProbabilisticRoadmaps pr;

  ros::spin();
  return 0;
}
