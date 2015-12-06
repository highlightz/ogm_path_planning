#ifndef OGMPP_GRAPH_DEF
#define OGMPP_GRAPH_DEF

#include "ogmpp_node.hpp"

namespace ogmpp_graph
{
  /**
   * @class Graph
   * @brief Handles a Graph which contains nodes
   */
  class Graph
  {
    private:
      /**< The graph's nodes */
      std::map<unsigned long, Node*> nodes;

    public:
      /**
       * @brief Default constructor
       */
      Graph(void);

      /**
       * @brief Initializes the Graph with a root node
       * @param cell [Cell] The coordinates to create the first node
       */
      Graph(Cell cell);

      /** 
       * @brief Prints information about the whole graph
       */
      void print(void);

      // TODO
      void clean(void){}
      // TODO
      void makeNeighbor(Cell cp, Cell cq){}
      // TODO
      void addNode(Cell c){}
      // TODO
      void removeNode(Cell c){}
  };

}

#endif
