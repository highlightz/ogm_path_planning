#include <iostream>
#include "ogmpp_graph/ogmpp_graph.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ogmpp_prms");
  ogmpp_graph::Graph g(0.2);
  g.addNode(ogmpp_graph::Cell(10,10));
  g.addNode(ogmpp_graph::Cell(20,20));
  g.addNode(ogmpp_graph::Cell(30,30));
  g.addNode(ogmpp_graph::Cell(40,40));
  g.addNode(ogmpp_graph::Cell(50,50));
  g.makeNeighbor(ogmpp_graph::Cell(10,10), ogmpp_graph::Cell(30,30));
  g.print();
  ros::spin();
  return 0;
}
