/**
 * @file /cost_map_core/src/lib/iterators/polygon_iterator.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_core/iterators/polygon_iterator.hpp>
#include <grid_map_core/GridMapMath.hpp>


using namespace std;

namespace cost_map {

PolygonIterator::PolygonIterator(const cost_map::CostMap& gridMap, const cost_map::Polygon& polygon)
    : polygon_(polygon)
{
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  Index submapStartIndex;
  Size submapBufferSize;
  findSubmapParameters(polygon, submapStartIndex, submapBufferSize);
  internalIterator_ = std::shared_ptr<SubmapIterator>(new SubmapIterator(gridMap, submapStartIndex, submapBufferSize));
  if(!isInside()) ++(*this);
}

PolygonIterator& PolygonIterator::operator =(const PolygonIterator& other)
{
  polygon_ = other.polygon_;
  internalIterator_ = other.internalIterator_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool PolygonIterator::operator !=(const PolygonIterator& other) const
{
  return (internalIterator_ != other.internalIterator_);
}

const Index& PolygonIterator::operator *() const
{
  return *(*internalIterator_);
}

PolygonIterator& PolygonIterator::operator ++()
{
  ++(*internalIterator_);
  if (internalIterator_->isPastEnd()) return *this;

  for ( ; !internalIterator_->isPastEnd(); ++(*internalIterator_)) {
    if (isInside()) break;
  }

  return *this;
}

bool PolygonIterator::isPastEnd() const
{
  return internalIterator_->isPastEnd();
}

bool PolygonIterator::isInside() const
{
  Position position;
  grid_map::getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  return polygon_.isInside(position);
}

void PolygonIterator::findSubmapParameters(const cost_map::Polygon& polygon, Index& startIndex, Size& bufferSize) const
{
  Position topLeft = polygon_.getVertices()[0];
  Position bottomRight = topLeft;
  for (const auto& vertex : polygon_.getVertices()) {
    topLeft = topLeft.array().max(vertex.array());
    bottomRight = bottomRight.array().min(vertex.array());
  }
  grid_map::limitPositionToRange(topLeft, mapLength_, mapPosition_);
  grid_map::limitPositionToRange(bottomRight, mapLength_, mapPosition_);
  grid_map::getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  Index endIndex;
  grid_map::getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  bufferSize = grid_map::getSubmapSizeFromCornerIndeces(startIndex, endIndex, bufferSize_, bufferStartIndex_);
}

} /* namespace cost_map */

