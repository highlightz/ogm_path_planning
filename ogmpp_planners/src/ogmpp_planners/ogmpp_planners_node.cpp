#include "ogmpp_planners/ogmpp_planners_node.hpp"

namespace ogmpp_planners
{
  OgmppPlanners::OgmppPlanners(void)
  {
    _server_path_planning = _nh.advertiseService(
      "/ogmpp_path_planners/plan",
      &OgmppPlanners::planCallback, this);

    _path_publisher = _nh.advertise<nav_msgs::Path>(
      "/ogmpp_path_planners/path", 10);

  }

  bool OgmppPlanners::planCallback(
    ogmpp_communications::OgmppPathPlanningSrv::Request& req,
    ogmpp_communications::OgmppPathPlanningSrv::Response& res)
  {

    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_;
    pose_.pose.position.x = req.data.begin.x;
    pose_.pose.position.y = req.data.begin.y;
    path.poses.push_back(pose_);

    // Just erase the previous path
    _path_publisher.publish(path);
    path.poses.clear();

    // Check if map is initialized
    if(_map.isMapInitialized() == false)
    {
      ROS_WARN_STREAM(
        "ogmpp_plannerss: Map is not initialized... waiting...");
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

    std::vector<ogmpp_graph::Cell> p;

    if(req.method == "uniform_prm")
      p = _uniform_sampling.createPath(_map, begin, end);
    else if(req.method == "random_prm")
      p = _random_sampling.createPath(_map, begin, end);
    else if(req.method == "halton_prm")
      p = _halton_sampling.createPath(_map, begin, end);
    else
    {
      res.error = "Invalid path planning method";
      return true;
    }
  
    for(unsigned int i = 0 ; i < p.size() ; i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = p[i].x * _map.getResolution();
      pose.pose.position.y = p[i].y * _map.getResolution();
      path.poses.push_back(pose);
    }

    if(path.poses.size() == 0)
    {
      res.error = "Could not create the path via " + req.method;
    }
    else
    {
      _path_publisher.publish(path);
    }

    res.path = path;
    res.error = "";
    return true;
  }

}

/**
 * @brief The main class of planners
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ogmpp_planners_node");
  srand (time(NULL));

  ogmpp_planners::OgmppPlanners planners;

  ros::spin();
  return 0;
}