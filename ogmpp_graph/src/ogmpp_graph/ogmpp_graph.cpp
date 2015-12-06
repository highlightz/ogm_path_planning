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

  /** 
   * @brief Prints information about the whole graph
   */
  void Graph::print(void)
  {
    for ( nodes_it it = nodes.begin() ; it != nodes.end() ; it++)
    {
      std::cout << "------------" << std::endl;
      it->second->print();
    } 
  }

  /**
   * @brief Deallocates the allocated nodes and clears the graph
   */
  void Graph::clean(void)
  {
    for (nodes_it it = nodes.begin() ; it != nodes.end() ; it++)
    {
      delete it->second;
    }
    nodes.clear();
  }

  /**
   * @brief Removes a cell from the graph. It also removes it from all its 
   * neighbors
   * @param cell [Cell] The cell to be removed
   */
  void Graph::removeNode(Cell cell)
  {
    for (nodes_it it = nodes.begin() ; it != nodes.end() ; it++)
    {
      it->second->removeNeighbor(cell);
    }
    nodes.erase(Node::createCantorPairing(cell));
  }

}
