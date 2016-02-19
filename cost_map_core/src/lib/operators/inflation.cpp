/**
 * @file /cost_map_core/src/lib/inflation.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../../../include/cost_map_core/operators/inflation.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cost_map {

/*****************************************************************************
** Implementation
*****************************************************************************/

void Inflate::operator()(const std::string layer_source,
                         const std::string layer_destination,
                         const float& inflation_radius,
                         const InflationComputer& inflation_computer,
                         CostMap& cost_map
                        ) {
  //unsigned char* master_array = cost_map...

  unsigned int size_x = cost_map.getSize().x();
  unsigned int size_y = cost_map.getSize().y();
  unsigned int size = size_x*size_y;
  const cost_map::Matrix& data_source = cost_map.get(layer_source);
  cost_map.add(layer_destination, data_source);

  // Rebuild internals
  seen_.resize(size_x, size_y);
  seen_.setConstant(false);
  unsigned int new_cell_inflation_radius = static_cast<unsigned int>(std::max(0.0, std::ceil(inflation_radius / cost_map.getResolution())));
  if (new_cell_inflation_radius != cell_inflation_radius_ ) {
    cell_inflation_radius_ = new_cell_inflation_radius;
    computeCaches(cost_map.getResolution(), inflation_computer);
  }

  // eigen is default column storage, so iterate over the rows most quickly
  // if we want to make it robust, check for data_source.IsRowMajor
  for (unsigned int j = 0, number_of_rows = data_source.rows(), number_of_columns = data_source.cols(); j < number_of_columns; ++j) {
    for (unsigned int i = 0; i < number_of_rows; ++i) {
      unsigned char cost = data_source(i, j);
      if (cost == LETHAL_OBSTACLE) {
        unsigned int index = j*number_of_rows + i;
        enqueue(cost_map.get(layer_source), cost_map.get(layer_destination), i, j, i, j);
      }
    }
  }
  while (!inflation_queue_.empty())
  {
    // get the highest priority cell and pop it off the priority queue
    const CellData& current_cell = inflation_queue_.top();

    unsigned int mx = current_cell.x_;
    unsigned int my = current_cell.y_;
    unsigned int sx = current_cell.src_x_;
    unsigned int sy = current_cell.src_y_;

    // pop once we have our cell info
    inflation_queue_.pop();

    // attempt to put the neighbors of the current cell onto the queue
    if (mx > 0) {
      enqueue(cost_map.get(layer_source), cost_map.get(layer_destination), mx - 1, my, sx, sy);
    }
    if (my > 0) {
      enqueue(cost_map.get(layer_source), cost_map.get(layer_destination), mx, my - 1, sx, sy);
    }
    if (mx < size_x - 1) {
      enqueue(cost_map.get(layer_source), cost_map.get(layer_destination), mx + 1, my, sx, sy);
    }
    if (my < size_y - 1) {
      enqueue(cost_map.get(layer_source), cost_map.get(layer_destination), mx, my + 1, sx, sy);
    }
  }
}

void Inflate::enqueue(const cost_map::Matrix& data_source,
                      cost_map::Matrix& data_destination,
                      unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y
                      )
{
  // set the cost of the cell being inserted
  if (!seen_(mx, my))
  {
    // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the queue if it is within the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_) {
      return;
    }

    // assign the cost associated with the distance from an obstacle to the cell
    unsigned char cost = costLookup(mx, my, src_x, src_y);
    unsigned char old_cost = data_source(mx, my);

    if (old_cost == NO_INFORMATION && cost >= INSCRIBED_OBSTACLE)
      data_destination(mx, my) = cost;
    else
      data_destination(mx, my) = std::max(old_cost, cost);

    // push the cell data onto the queue and mark
    seen_(mx, my) = true;
    CellData data(distance, mx, my, src_x, src_y);
    inflation_queue_.push(data);
  }
}

double Inflate::distanceLookup(int mx, int my, int src_x, int src_y)
{
  unsigned int dx = abs(mx - src_x);
  unsigned int dy = abs(my - src_y);
  return cached_distances_(dx, dy);
}

unsigned char Inflate::costLookup(int mx, int my, int src_x, int src_y)
{
  unsigned int dx = abs(mx - src_x);
  unsigned int dy = abs(my - src_y);
  return cached_costs_(dx, dy);
}

void Inflate::computeCaches(const float& resolution, const InflationComputer& compute_cost)
{
  cached_costs_.resize(cell_inflation_radius_ + 2, cell_inflation_radius_ + 2);
  cached_distances_.resize(cell_inflation_radius_ + 2, cell_inflation_radius_ + 2);

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i) {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j) {
      cached_distances_(i,j) = std::hypot(i, j);
    }
  }

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i) {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j) {
      cached_costs_(i, j) = compute_cost(resolution*cached_distances_(i, j));
    }
  }
}

/*****************************************************************************
** Inflation Computers
*****************************************************************************/

ROSInflationComputer::ROSInflationComputer(
    const float& inscribed_radius,
    const float& weight)
: inscribed_radius_(inscribed_radius)
, weight_(weight)
{
}

unsigned char ROSInflationComputer::operator()(const float &distance) const {
  unsigned char cost = 0;
  if (distance == 0.0)
    cost = LETHAL_OBSTACLE;
  else if (distance <= inscribed_radius_)
    cost = INSCRIBED_OBSTACLE;
  else
  {
    // make sure cost falls off by Euclidean distance
    double factor = std::exp(-1.0 * weight_ * (distance - inscribed_radius_));
    cost = (unsigned char)((INSCRIBED_OBSTACLE - 1) * factor);
  }
  return cost;
}

/*****************************************************************************
** Deflation
*****************************************************************************/

Deflate::Deflate(const bool& do_not_strip_inscribed_region)
: do_not_strip_inscribed_region(do_not_strip_inscribed_region)
{
}

void Deflate::operator()(const std::string layer_source,
                         const std::string layer_destination,
                         CostMap& cost_map
                         )
{
  // make a call on the data, just to check that the layer is there
  // will throw std::out_of_range if not
  cost_map::Matrix data_source = cost_map.get(layer_source);
  // add a layer filled with NO_INFORMATION
  cost_map.add(layer_destination);
  cost_map::Matrix& data_destination = cost_map.get(layer_destination);

  unsigned char cost_threshold = do_not_strip_inscribed_region ? cost_map::INSCRIBED_OBSTACLE : cost_map::LETHAL_OBSTACLE;

  // eigen is by default column major - iterate with the row index changing fastest
  for (unsigned int j = 0, number_of_rows = data_source.rows(), number_of_columns = data_source.cols(); j < number_of_columns; ++j) {
    for (unsigned int i = 0; i < number_of_rows; ++i) {
      unsigned char cost = data_source(i, j);
      data_destination(i, j) = (cost >= cost_threshold ) ? cost : cost_map::FREE_SPACE;
    }
  }
}


/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace cost_map
