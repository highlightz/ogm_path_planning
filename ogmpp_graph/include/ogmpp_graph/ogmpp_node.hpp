#ifndef OGMPP_NODE_DEF
#define OGMPP_NODE_DEF

#include <iostream>
#include <vector>

/**
 * @brief The OGMPP Graph related namespace
 */
namespace ogmpp_graph
{

  /**
   * @class Node
   * @brief Handles a node, along with its connections
   */
  class Node
  {
    private:
      /**< The node's neighbors */
      std::vector<Node*> _neighbors;
      /**< The connections' weights */
      std::vector<float> _weights;
    public:
      /**
       * @brief Default constructor
       */
      Node(void);
  };

}

#endif
