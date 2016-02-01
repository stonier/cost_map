/*
 * SubmapGeometry.cpp
 *
 *  Created on: Aug 18, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <cost_map_core/CostMapMath.hpp>
#include "../../include/cost_map_core/SubmapGeometry.hpp"


namespace cost_map {

SubmapGeometry::SubmapGeometry(const CostMap& gridMap, const Position& position,
                                     const Length& length, bool& isSuccess)
    : gridMap_(gridMap)
{
  isSuccess = getSubmapInformation(startIndex_, size_, position_, length_,
                                   requestedIndexInSubmap_, position, length, gridMap_.getLength(),
                                   gridMap_.getPosition(), gridMap_.getResolution(),
                                   gridMap_.getSize(), gridMap_.getStartIndex());
}

SubmapGeometry::~SubmapGeometry()
{
}

const CostMap& SubmapGeometry::getGridMap() const
{
  return gridMap_;
}

const Length& SubmapGeometry::getLength() const
{
  return length_;
}

const Position& SubmapGeometry::getPosition() const
{
  return position_;
}

const Index& SubmapGeometry::getRequestedIndexInSubmap() const
{
  return requestedIndexInSubmap_;
}

const Size& SubmapGeometry::getSize() const
{
  return size_;
}

double SubmapGeometry::getResolution() const
{
  return gridMap_.getResolution();
}

const Index& SubmapGeometry::getStartIndex() const
{
  return startIndex_;
}

} /* namespace cost_map */
