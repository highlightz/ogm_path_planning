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
      // Pass the coords in pixels
      ogmpp_graph::Cell begin(
        req.data.begin.x / _map.getResolution(), 
        req.data.begin.y / _map.getResolution() );
      ogmpp_graph::Cell end(
        req.data.end.x / _map.getResolution(), 
        req.data.end.y / _map.getResolution());

      std::vector<ogmpp_graph::Cell> p =
        _uniform_sampling.createPath(_map, begin, end);

      nav_msgs::Path path;
      path.header.frame_id = "map";
      path.header.stamp = ros::Time::now();

      for(unsigned int i = 0 ; i < p.size() ; i++)
      {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = p[i].x * _map.getResolution();
        pose.pose.position.y = p[i].y * _map.getResolution();
        path.poses.push_back(pose);
      }

      res.path = path;

      // TODO: Add path publisher here?

      res.error = "";

      return true;
    }

  }
}
