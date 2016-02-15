/**
 * @file /cost_map_core/include/cost_map_core/operations.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_core_OPERATIONS_HPP_
#define cost_map_core_OPERATIONS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "CostMap.hpp"
#include <queue>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Helpers
*****************************************************************************/

struct CellData {
  /**
   * @brief  Constructor for CellData objects
   * @param  d The distance to the nearest obstacle, used for ordering in the priority queue
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(double d, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
      distance_(d), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  double distance_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};
/**
 * @brief Provide an ordering between CellData objects in the priority queue
 * @return We want the lowest distance to have the highest priority... so this returns true if a has higher priority than b
 */
inline bool operator<(const CellData &a, const CellData &b)
{
  return a.distance_ > b.distance_;
}

/*****************************************************************************
** Inflation Function
*****************************************************************************/


/*****************************************************************************
** Inflation
*****************************************************************************/

class Inflate {
public:
  void operator()(const std::string& layer_source,
               const std::string& layer_destination,
               const float& inflation_radius,
               const float& inscribed_radius,
               CostMap& cost_map
               );

  /**
   * @brief Function which can compute costs for the inflation layer.
   */
  class CostGenerator {
  public:
    CostGenerator(const float& inscribed_radius,
                  const float& resolution,
                  const float& weight);

    /** @brief  Given a distance, compute a cost.
     *
     * @param  distance The distance from an obstacle in cells
     * @return A cost value for the distance
     **/
    unsigned char operator()(float &distance) const;
  private:
    float inscribed_radius_, resolution_, weight_;
  };

protected:

private:

  /**
   * @brief  Given an index of a cell in the costmap, place it into a priority queue for obstacle inflation
   * @param  data_destination the costs (will be read and superimposed on this)
   * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
   * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
   * @param  src_x The x index of the obstacle point inflation started at
   * @param  src_y The y index of the obstacle point inflation started at
   */
  void enqueue(cost_map::Matrix& data_destination,
               unsigned int mx, unsigned int my,
               unsigned int src_x, unsigned int src_y
               );

  /**
   * @brief  Lookup pre-computed distances
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  double distanceLookup(int mx, int my, int src_x, int src_y);

  /**
   * @brief  Lookup pre-computed costs
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  unsigned char costLookup(int mx, int my, int src_x, int src_y);
  void computeCaches(const CostGenerator& compute_cost);

  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> seen_;
  Eigen::MatrixXf cached_distances_;
  cost_map::Matrix cached_costs_;
  std::priority_queue<CellData> inflation_queue_;
  unsigned int cell_inflation_radius_; /// size of the inflation radius in cells
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map_core

#endif /* cost_map_core_OPERATIONS_HPP_ */
