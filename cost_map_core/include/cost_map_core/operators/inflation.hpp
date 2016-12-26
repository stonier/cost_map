/**
 * @file /cost_map_core/include/cost_map_core/operators/inflation.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_core_INFLATION_HPP_
#define cost_map_core_INFLATION_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../cost_map.hpp"
#include <limits>
#include <queue>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Inflation Function
*****************************************************************************/

/**
 * @brief Function which can compute costs for the inflation layer.
 *
 * Inherit from this to generate your own inflation
 * functions.
 */
class InflationComputer {
public:
  InflationComputer() {};
  virtual ~InflationComputer() {};

  /** @brief  Given a distance, compute a cost.
   *
   * @param  distance The distance from an obstacle in cells
   * @return A cost value for the distance
   **/
  virtual unsigned char operator()(const float &distance) const = 0;
};

/**
 * @brief Function which can compute costs for the inflation layer.
 *
 * This class provides a default inflation function which works like the
 * ROS inflation layer. Inherit from this to generate your own inflation
 * functions.
 */
class ROSInflationComputer : public InflationComputer {
public:
  ROSInflationComputer(const float& inscribed_radius, const float& weight);

  virtual ~ROSInflationComputer() {};

  /** @brief  Given a distance, compute a cost.
   *
   * @param  distance The metric distance from an obstacle (distance = cell_distance*resolution)
   * @return A cost value for the distance
   **/
  virtual unsigned char operator()(const float &distance) const;
private:
  float inscribed_radius_, weight_;
};

/*****************************************************************************
** Inflation
*****************************************************************************/

class Inflate {
public:
  Inflate() : cell_inflation_radius_(std::numeric_limits<unsigned int>::max())
  {
  };

  /**
   * @brief Inflate...
   *
   * @param layer_source
   * @param layer_destination
   * @param inflation_radius
   * @param inscribed_radius
   * @param cost_map
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  void operator()(const std::string layer_source,
               const std::string layer_destination,
               const float& inflation_radius,
               const InflationComputer& inflation_computer,
               CostMap& cost_map
               );

private:
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
    /**
     * @brief Provide an ordering between CellData objects in the priority queue
     * @return We want the lowest distance to have the highest priority... so this returns true if a has higher priority than b
     */
    friend bool operator<(const CellData &a, const CellData &b) {
      return a.distance_ > b.distance_;
    }
    double distance_;
    unsigned int x_, y_;
    unsigned int src_x_, src_y_;
  };

  /**
   * @brief  Given an index of a cell in the costmap, place it into a priority queue for obstacle inflation
   * @param  data_destination the costs (will be read and superimposed on this)
   * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
   * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
   * @param  src_x The x index of the obstacle point inflation started at
   * @param  src_y The y index of the obstacle point inflation started at
   */
  void enqueue(const cost_map::Matrix& data_source,
               cost_map::Matrix& data_destination,
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
  void computeCaches(const float& resolution, const InflationComputer& compute_cost);

  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> seen_;
  Eigen::MatrixXf cached_distances_;
  cost_map::Matrix cached_costs_;
  std::priority_queue<CellData> inflation_queue_;
  unsigned int cell_inflation_radius_; /// size of the inflation radius in cells
};

/*****************************************************************************
** Deflation
*****************************************************************************/

/**
 * @brief Functor to strip the inflation layer from a cost map.
 */
class Deflate {
public:
  Deflate(const bool& do_not_strip_inscribed_region=false);

  /**
   * @brief Deflate...
   *
   * @param layer_source
   * @param layer_destination
   * @param cost_map
   * @throw std::out_of_range if no map layer with name `layer` is present.
   */
  void operator()(const std::string layer_source,
                  const std::string layer_destination,
                  CostMap& cost_map
                 );

private:
  bool do_not_strip_inscribed_region;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace cost_map_core

#endif /* cost_map_core_INFLATION_HPP_ */
