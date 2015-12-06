#include "ogmpp_node.hpp"

namespace ogmpp_graph
{

  /**
   * @brief Creates a unique id from two single values. It will be used
   * for easily searching the neighbors
   * @param cell [Cell] The cell whose coordinates will produce the Cantor
   * pairing value
   * @return unsinged long : The unique value
   */
  unsigned long Node::createCantorPairing(Cell cell)
  {
    // Added 0.5 for rounding reasons
    // Added 1 to consider id = 0 uninitialized
    return 0.5*(cell.x + cell.y)*(cell.x + cell.y + 1) + cell.y + 0.5 + 1;
  }

  /**
   * @brief Default constructor
   */
  Node::Node(void)
  {
    // Uninitialized id
    _id = 0;
    // Uninitialized parent id
    _parent_id = 0;
  }

  /**
   * @brief Initializes a node with a pose
   * @param pose [Cell] The node's coordinates
   */
  Node::Node(Cell pose, unsigned long parent_id)
  {
    this->_parent_id = parent_id;
    this->_pose = pose;
    // id is now initialized
    this->_id = Node::createCantorPairing(pose);
  }

  /**
   * @brief Returns the node's id
   * @return unsigned long : The node's id
   */
  unsigned long Node::getId(void)
  {
    return _id;
  }

  /**
   * @brief Returns the node's pose
   * @return Cell : The node's pose
   */
  Cell Node::getPose(void)
  {
    return _pose;
  }

  /**
   * @brief Prints information about the node
   */
  void Node::print(void)
  {
    std::cout << "ID = " << _id << std::endl;
    std::cout << "Pose = ";
    _pose.print();
    std::cout << std::endl;
    std::cout << "Neighbors:" << std::endl;
    for (neigh_it it = _neighbors.begin() ; it != _neighbors.end() ; it++)
    {
      std::cout << "\t" << it->first << " " << _weights[it->first] << " ";
      it->second->getPose().print();
      std::cout << std::endl;
    }
  }

  /**
   * @brief Adds a neighbor giving a node
   * @param node [Node*] The neighboring node
   * @param is_parent [bool] True if it is its parent
   * @param weight [float] Optional parameter for user-defined weights. If 
   * the value is -1 the Eucledian distance will be used
   * @param reverse_neighborhood [bool] If true the neighborhood is created 
   * both ways
   */
  void Node::addNeighbor(
    Node *node, 
    bool is_parent, 
    float weight, 
    bool reverse_neighborhood)
  {
    // Add the node here as neighbor
    _neighbors.insert( std::pair<unsigned long, Node*>(node->getId(), node) );
    if (is_parent)
    {
      _parent_id = node->getId();
    }
    float w = weight;
    if (w < 0)
    {
      w = this->_pose.distanceFrom(node->getPose());
    }
    _weights.insert( std::pair<unsigned long, float>(node->getId(), w) );

    // Add the reverse neighborhood
    if (reverse_neighborhood)
    {
      node->addNeighbor(this, !is_parent, w, false);
    }
  }

  /**
   * @brief Removes a node from the neighbors list
   * @param node [Node*] The node to be erased
   */
  void Node::removeNeighbor(Node *node)
  {
    _neighbors.erase(node->getId());
    _weights.erase(node->getId());
  }

  /**
   * @brief Removes a node from the neighbors list
   * @param cell [Cell] The cell to be erased
   */
  void Node::removeNeighbor(Cell cell)
  {
    _neighbors.erase(Node::createCantorPairing(cell));
    _weights.erase(Node::createCantorPairing(cell));
  }
}
