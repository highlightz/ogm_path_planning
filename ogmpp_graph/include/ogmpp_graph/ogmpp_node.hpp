#ifndef OGMPP_NODE_DEF
#define OGMPP_NODE_DEF

#include "ogmpp_cell.hpp"

/**
 * @brief The OGMPP Graph related namespace
 */
namespace ogmpp_graph
{

  /**
   * @class Node
   * @brief Handles a node, along with its connections
   * Each node's id is derived from it's coordinates
   */
  class Node
  {
    
    // Typedef for easier iteration of the map
    typedef std::map<unsigned long, Node*>::iterator neigh_it;
    
    private:

      /**< The node's id */
      unsigned long _id;
      /**< The node's neighbors */
      std::map<unsigned long, Node*> _neighbors;
      /**< The connections' weights */
      std::map<unsigned long, float> _weights;
      /**< The node's pose */
      Cell _pose;
      /**< The node who created the current one */
      unsigned long _parent_id;

      /**
       * @brief Creates a unique id from two single values. It will be used
       * for easily searching the neighbors
       * @param cell [Cell] The cell whose coordinates will produce the Cantor
       * pairing value
       * @return unsinged long : The unique value
       */
      static unsigned long createCantorPairing(Cell cell);

    public:
      /**
       * @brief Default constructor
       */
      Node(void);

      /**
       * @brief Initializes a node with a pose
       * @param pose [Cell] The node's coordinates
       * @param parent_id [unsigned long] The parent's id. Default value is 0
       */
      Node(Cell pose, unsigned long parent_id = 0);

      /**
       * @brief Initializes a node with a pose
       * @param pose [Cell] The node's coordinates
       */
      void setNeighbor(Cell pose);

      /**
       * @brief Returns the node's id
       * @return unsigned long : The node's id
       */
      unsigned long getId(void);

      /**
       * @brief Returns the node's pose
       * @return Cell : The node's pose
       */
      Cell getPose(void);

      /**
       * @brief Prints information about the node
       */
      void print(void);

      /**
       * @brief Adds a neighbor giving a node
       * @param node [Node*] The neighboring node
       * @param is_parent [bool] True if it is its parent
       * @param weight [float] Optional parameter for user-defined weights. If 
       * the value is -1 the Eucledian distance will be used
       * @param reverse_neighborhood [bool] If true the neighborhood is created 
       * both ways
       */
      void addNeighbor(
        Node *node, 
        bool is_parent = true, 
        float weight = -1,
        bool reverse_neighborhood = false);

      // TODO
      void removeNeighbor(Node *node);
      void removeNeighbor(Cell cell);

  };

}

#endif
