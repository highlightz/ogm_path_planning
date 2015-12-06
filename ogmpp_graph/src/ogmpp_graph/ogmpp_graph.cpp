#include "ogmpp_graph.hpp"

namespace ogmpp_graph
{

  /**
   * @brief Default constructor
   */
  Graph::Graph(void)
  {

  }

  /**
   * @brief Initializes the Graph with a root node
   * @param cell [Cell] The coordinates to create the first node
   */
  Graph::Graph(Cell cell)
  {
    Node *n = new Node(cell);
    nodes.insert( std::pair<unsigned long, Node*>(n->getId(), n) );
  }

  void Graph::print(void)
  {

  }
}
