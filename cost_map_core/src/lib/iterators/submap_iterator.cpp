/**
 * @file /cost_map_core/src/lib/iterators/spiral_iterator.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <grid_map_core/GridMapMath.hpp>
#include "../../../include/cost_map_core/iterators/submap_iterator.hpp"


using namespace std;

namespace cost_map {

SubmapIterator::SubmapIterator(const cost_map::SubmapGeometry& submap)
    : SubmapIterator(submap.getGridMap(), submap.getStartIndex(), submap.getSize())
{
}

SubmapIterator::SubmapIterator(const cost_map::CostMap& gridMap,
                               const cost_map::BufferRegion& bufferRegion)
    : SubmapIterator(gridMap, bufferRegion.getStartIndex(), bufferRegion.getSize())
{
}


SubmapIterator::SubmapIterator(const cost_map::CostMap& gridMap, const Index& submapStartIndex,
                               const Size& submapSize)
{
  size_ = gridMap.getSize();
  startIndex_ = gridMap.getStartIndex();
  index_ = submapStartIndex;
  submapSize_ = submapSize;
  submapStartIndex_ = submapStartIndex;
  submapIndex_.setZero();
  isPastEnd_ = false;
}

SubmapIterator::SubmapIterator(const SubmapIterator* other)
{
  size_ = other->size_;
  startIndex_ = other->startIndex_;
  submapSize_ = other->submapSize_;
  submapStartIndex_ = other->submapStartIndex_;
  index_ = other->index_;
  submapIndex_ = other->submapIndex_;
  isPastEnd_ = other->isPastEnd_;
}

SubmapIterator& SubmapIterator::operator =(const SubmapIterator& other)
{
  size_ = other.size_;
  startIndex_ = other.startIndex_;
  submapSize_ = other.submapSize_;
  submapStartIndex_ = other.submapStartIndex_;
  index_ = other.index_;
  submapIndex_ = other.submapIndex_;
  isPastEnd_ = other.isPastEnd_;
  return *this;
}

bool SubmapIterator::operator !=(const SubmapIterator& other) const
{
  return (index_ != other.index_).any();
}

const Eigen::Array2i& SubmapIterator::operator *() const
{
  return index_;
}

const Eigen::Array2i& SubmapIterator::getSubmapIndex() const
{
  return submapIndex_;
}

SubmapIterator& SubmapIterator::operator ++()
{
  isPastEnd_ = !grid_map::incrementIndexForSubmap(submapIndex_, index_, submapStartIndex_,
                                         submapSize_, size_, startIndex_);
  return *this;
}

bool SubmapIterator::isPastEnd() const
{
  return isPastEnd_;
}

} /* namespace cost_map */

