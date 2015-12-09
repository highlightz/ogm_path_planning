#ifndef OGMPP_CELL_DEF
#define OGMPP_CELL_DEF

#include <iostream>
#include <vector>
#include <map>
#include <cmath>

#include <ros/ros.h>

/**
 * @brief The OGMPP Graph related namespace
 */
namespace ogmpp_graph
{

  /**
   * @class Cell
   * @brief Handles an OGM cell
   */
  class Cell
  {
    public:
      /**< The cell's x coordinate */
      unsigned int x;
      /**< The cell's y coordinate */
      unsigned int y;

      /**
       * @brief Default constructor
       */
      Cell(void);
    
      /**
       * @brief Initializes a cell with its coordinates
       * @param x [unsinged int] The x coordinate
       * @param y [unsinged int] The y coordinate
       */
      Cell(unsigned int x,unsigned int y);

      /**
       * @brief Calculates the distance between two cells
       * @param cell [const Cell&] The other cell
       * @return float: The eucledian distance between the two cells
       */
      float distanceFrom(const Cell& cell);

      /**
       * @brief Calculates the square distance between two cells
       * @param cell [const Cell&] The other cell
       * @return float: The square of the eucledian distance between the two cells
       */
      float sqDistanceFrom(const Cell& cell);

      /**
       * @brief Prints the Cell's coordinates
       */
      void print(void);

      /**
       * @brief Overloading of == operator between two cells
       */
      bool operator==(const Cell& c);

  };

}

#endif
