#include <iostream>
#include "ogmpp_graph/ogmpp_graph.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ogmpp_prms");
  ogmpp_graph::Graph g(0.2);
  g.addNode(ogmpp_graph::Cell(10,10));
  g.addNode(ogmpp_graph::Cell(20,60));
  g.addNode(ogmpp_graph::Cell(30,50));
  g.addNode(ogmpp_graph::Cell(10,40));
  g.addNode(ogmpp_graph::Cell(50,20));
  g.makeNeighbor(ogmpp_graph::Cell(10,10), ogmpp_graph::Cell(30,50));
  g.print();
  ros::spin();
  return 0;
}
