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

  /**
   * @brief Adds a node in the graph without assigning neighbors
   * @param c [Cell] The new node's coordinates
   * @return Node * : The new node
   */
  Node* Graph::addNode(Cell cell)
  {
    Node *n = new Node(cell);
    nodes.insert( std::pair<unsigned long, Node*>(n->getId(), n) );
  }

  /**
   * @brief Creates a neighboring relation between two cells. If one of them
   * or both do not exist as nodes, it creates them
   * @param cp [Cell] The first cell
   * @param cq [Cell] The second cell
   * @param first_is_parent [bool] True if the first is the parent of the second
   */
  void Graph::makeNeighbor(Cell cp, Cell cq, bool first_is_parent)
  {
    Node *np, *nq;
    // Check if cp exists
    if (nodes.find(Node::createCantorPairing(cp)) == nodes.end())
    {
      np = addNode(cp);
    }
    else
    {
      np = nodes[Node::createCantorPairing(cp)];
    }

    // Check if cp exists
    if (nodes.find(Node::createCantorPairing(cp)) == nodes.end())
    {
      nq = addNode(cp);
    }
    else
    {
      nq = nodes[Node::createCantorPairing(cq)];
    }

    // Weight is the eucledian distance and the reverse neighborhood is 
    // created
    nq->addNeighbor(np, first_is_parent, -1, true);
  }

}
