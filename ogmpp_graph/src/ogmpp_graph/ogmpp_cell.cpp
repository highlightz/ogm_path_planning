#include "ogmpp_cell.hpp"

namespace ogmpp_graph
{
  /**
   * @brief Default constructor
   */
  Cell::Cell(void)
  {

  }

  /**
   * @brief Initializes a cell with its coordinates
   * @param x [unsinged int] The x coordinate
   * @param y [unsinged int] The y coordinate
   */
  Cell::Cell(unsigned int x, unsigned int y)
  {
    this->x = x;
    this->y = y;
  }

  /**
   * @brief Calculates the distance between two cells
   * @param cell [const Cell&] The other cell
   * @return float: The eucledian distance between the two cells
   */
  float Cell::distanceFrom(const Cell& cell)
  {
    return sqrt( pow(float(x) - cell.x, 2) + pow(float(y) - cell.y, 2) );
  }


  /**
   * @brief Calculates the square distance between two cells
   * @param cell [const Cell&] The other cell
   * @return float: The square of eucledian distance between the two cells
   */
  float Cell::sqDistanceFrom(const Cell& cell)
  {
    return pow(float(x) - cell.x, 2) + pow(float(y) - cell.y, 2);
  }

  /**
   * @brief Prints the Cell's coordinates
   */
  void Cell::print(void)
  {
    ROS_INFO_STREAM( "[" << x << "," << y << "]" );
  }

  /**
   * @brief Overloading of == operator between two cells
   */
  bool Cell::operator==(const Cell& c)
  {
    return (this->x == c.x) && (this->y == c.y);
  }

}
